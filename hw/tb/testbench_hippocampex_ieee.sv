// testbench_hippocampex_ieee: the big regression on hippocampex_top (no AXI).
// Groups:
//   A basic           : first event of each of the 8 contexts, exact cues
//   B noise           : 0-13 flipped bits on the Blue/7 cue
//   C moving cue      : step the cue along Blue, then Green at 4
//   D context switch  : event 7 in Blue vs Red and Blue vs Magenta, fast switching
//   E edge cases      : end of a chain, event with no links
//   F longer chain    : 6-event Yellow chain from 0 and from 3
//   G throughput      : 10 back-to-back recalls
//
// CSV outputs (used for the paper plots):
//   1. test_summary.csv        - Per-test pass/fail + latency breakdown
//   2. prediction_detail.csv   - Per-event expected vs actual voltages
//   3. noise_sweep.csv         - Bit error rate vs recognition/prediction
//   4. voltage_heatmap.csv     - every event with a nonzero voltage, per test
//   5. ca1_ordering.csv        - Temporal ordering data per test
//   6. context_isolation.csv   - Cross-context leakage measurements
//   7. temporal_integration.csv- Past/Present/Future event integration
//   8. latency_breakdown.csv   - Per-stage pipeline latency analysis

`timescale 1ns/1ps

module testbench_hippocampex_ieee;

    localparam HV_WIDTH       = 128;
    localparam NUM_CONTEXTS   = 16;
    localparam NUM_EVENTS     = 64;
    localparam DIST_WIDTH     = 8;
    localparam MAX_MISMATCH   = 13;
    localparam MAX_STEPS      = 8;
    localparam CNTR_WIDTH     = 6;
    localparam VOLTAGE_WIDTH  = 8;
    localparam DECAY_STEP     = 4;
    localparam CLK_PERIOD     = 10;
    localparam CLK_FREQ_MHZ   = 100;

    // MCAM tag values
    localparam VP  = 8'h40;  // present
    localparam VF1 = 8'h80;  // next
    localparam VF2 = 8'hC0;  // one after next

    // contexts and the chain each one learns
    localparam CTX_BLUE    = 0;   // 7->5->3->2
    localparam CTX_MAGENTA = 1;   // 7->5->3->1
    localparam CTX_GREEN   = 2;   // 2->4->6->8
    localparam CTX_RED     = 3;   // 7->9->4
    localparam CTX_CYAN    = 4;   // 10->11->12->13->14
    localparam CTX_YELLOW  = 5;   // 0->1->2->3->4->5
    localparam CTX_ORANGE  = 6;   // 120->...->123, remapped below
    localparam CTX_PURPLE  = 7;   // 124->...->127, remapped below

    localparam EVT_0=0,  EVT_1=1,  EVT_2=2,  EVT_3=3,  EVT_4=4;
    localparam EVT_5=5,  EVT_6=6,  EVT_7=7,  EVT_8=8,  EVT_9=9;
    localparam EVT_10=10, EVT_11=11, EVT_12=12, EVT_13=13, EVT_14=14, EVT_15=15;
    // Orange and Purple were written for 128 events (slots 120-127). With 64
    // events those slots don't exist, so the names now point at 16-23.
    localparam EVT_120=16, EVT_121=17, EVT_122=18, EVT_123=19;
    localparam EVT_124=20, EVT_125=21, EVT_126=22, EVT_127=23;

    localparam MAX_TESTS = 64;

    typedef struct {
        string  test_name;
        string  description;
        string  group;
        int     start_cycle;
        int     end_cycle;
        int     dgec_latency;
        int     ca3_latency;
        int     ca1_latency;
        int     total_latency;
        real    latency_us;
        int     passed;
        int     failed;
    } test_metrics_t;

    test_metrics_t test_results [MAX_TESTS];
    int current_test_num = 0;
    int total_tests_passed = 0;
    int total_tests_failed = 0;
    int total_checks_passed = 0;
    int total_checks_failed = 0;
    int inference_count = 0;  // inference_id in the CSVs

    logic clk, rst_n;
    logic i_valid;
    logic [HV_WIDTH-1:0] i_context_hv;
    logic [HV_WIDTH-1:0] i_event_hv;

    logic i_train_mode;
    logic i_train_wr_context;
    logic [$clog2(NUM_CONTEXTS)-1:0] i_train_context_addr;
    logic [HV_WIDTH-1:0] i_train_context_data;
    logic i_train_wr_event;
    logic [$clog2(NUM_EVENTS)-1:0] i_train_event_addr;
    logic [HV_WIDTH-1:0] i_train_event_data;

    logic i_crossbar_we;
    logic i_crossbar_wtype;
    logic [$clog2(NUM_EVENTS)-1:0] i_crossbar_waddr1;
    logic [$clog2(NUM_EVENTS)-1:0] i_crossbar_waddr2_assoc;
    logic [$clog2(NUM_CONTEXTS)-1:0] i_crossbar_waddr2_gate;
    logic i_crossbar_wdata;

    logic i_mcam_we;
    logic [$clog2(NUM_EVENTS)-1:0] i_mcam_event_addr;
    logic [$clog2(NUM_CONTEXTS)-1:0] i_mcam_context_addr;
    logic [CNTR_WIDTH-1:0] i_mcam_data;

    logic [VOLTAGE_WIDTH-1:0] o_event_voltages [NUM_EVENTS];
    logic [VOLTAGE_WIDTH-1:0] o_past_voltages [NUM_EVENTS];
    logic [VOLTAGE_WIDTH-1:0] o_merged_voltages [NUM_EVENTS];
    logic [NUM_EVENTS-1:0]    o_ordered_onehot;
    logic [CNTR_WIDTH-1:0]    o_timestamps [NUM_EVENTS];
    logic [NUM_CONTEXTS-1:0]  o_context_onehot;
    logic [NUM_EVENTS-1:0]    o_event_onehot;
    logic [$clog2(NUM_CONTEXTS)-1:0] o_context_idx;
    logic [$clog2(NUM_EVENTS)-1:0]   o_event_idx;
    logic o_valid, o_ready;

    int cycle_count       = 0;
    int dgec_start_cycle  = 0;
    int ca3_start_cycle   = 0;
    int ca1_start_cycle   = 0;

    // copies of the trained HVs, reused as queries
    logic [HV_WIDTH-1:0] stored_context_hvs [NUM_CONTEXTS];
    logic [HV_WIDTH-1:0] stored_event_hvs [NUM_EVENTS];

    integer fd_summary, fd_prediction, fd_noise, fd_heatmap;
    integer fd_ca1, fd_isolation, fd_temporal, fd_latency;

    hippocampex_top #(
        .HV_WIDTH(HV_WIDTH),
        .NUM_CONTEXTS(NUM_CONTEXTS),
        .NUM_EVENTS(NUM_EVENTS),
        .DIST_WIDTH(DIST_WIDTH),
        .MAX_MISMATCH(MAX_MISMATCH),
        .MAX_STEPS(MAX_STEPS),
        .CNTR_WIDTH(CNTR_WIDTH),
        .VOLTAGE_WIDTH(VOLTAGE_WIDTH),
        .DECAY_STEP(DECAY_STEP)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .i_valid(i_valid),
        .i_context_hv(i_context_hv),
        .i_event_hv(i_event_hv),
        .i_train_mode(i_train_mode),
        .i_train_wr_context(i_train_wr_context),
        .i_train_context_addr(i_train_context_addr),
        .i_train_context_data(i_train_context_data),
        .i_train_wr_event(i_train_wr_event),
        .i_train_event_addr(i_train_event_addr),
        .i_train_event_data(i_train_event_data),
        .i_crossbar_we(i_crossbar_we),
        .i_crossbar_wtype(i_crossbar_wtype),
        .i_crossbar_waddr1(i_crossbar_waddr1),
        .i_crossbar_waddr2_assoc(i_crossbar_waddr2_assoc),
        .i_crossbar_waddr2_gate(i_crossbar_waddr2_gate),
        .i_crossbar_wdata(i_crossbar_wdata),
        .i_mcam_we(i_mcam_we),
        .i_mcam_event_addr(i_mcam_event_addr),
        .i_mcam_context_addr(i_mcam_context_addr),
        .i_mcam_data(i_mcam_data),
        .o_event_voltages(o_event_voltages),
        .o_past_voltages(o_past_voltages),
        .o_merged_voltages(o_merged_voltages),
        .o_ordered_onehot(o_ordered_onehot),
        .o_timestamps(o_timestamps),
        .o_context_onehot(o_context_onehot),
        .o_event_onehot(o_event_onehot),
        .o_context_idx(o_context_idx),
        .o_event_idx(o_event_idx),
        .o_valid(o_valid),
        .o_ready(o_ready)
    );

    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    always @(posedge clk) begin
        if (rst_n) cycle_count <= cycle_count + 1;
        else cycle_count <= 0;
    end

    function automatic logic [HV_WIDTH-1:0] gen_random_hv();
        logic [HV_WIDTH-1:0] hv;
        int ones_count;
        hv = '0;
        ones_count = HV_WIDTH / 10;  // at most 12 ones; repeats make it fewer
        for (int i = 0; i < ones_count; i++) begin
            int pos = $urandom_range(0, HV_WIDTH-1);
            hv[pos] = 1'b1;
        end
        return hv;
    endfunction

    function automatic logic [HV_WIDTH-1:0] flip_bits(logic [HV_WIDTH-1:0] hv, int n);
        logic [HV_WIDTH-1:0] result;
        result = hv;
        for (int i = 0; i < n; i++) begin
            int pos = $urandom_range(0, HV_WIDTH-1);
            result[pos] = ~result[pos];
        end
        return result;
    endfunction

    function automatic string get_ctx_name(int ctx);
        case (ctx)
            CTX_BLUE:    return "Blue";
            CTX_MAGENTA: return "Magenta";
            CTX_GREEN:   return "Green";
            CTX_RED:     return "Red";
            CTX_CYAN:    return "Cyan";
            CTX_YELLOW:  return "Yellow";
            CTX_ORANGE:  return "Orange";
            CTX_PURPLE:  return "Purple";
            default:     return $sformatf("Ctx%0d", ctx);
        endcase
    endfunction

    function automatic string get_voltage_role(logic [7:0] v);
        if (v == VP)  return "Present";
        if (v == VF1) return "Future1";
        if (v == VF2) return "Future2";
        if (v == 0)   return "Inactive";
        return $sformatf("Decay_%02h", v);
    endfunction

    // CSV files, see the list at the top
    task open_csv_files();
        fd_summary = $fopen("test_summary.csv", "w");
        $fwrite(fd_summary, "test_num,test_name,group,result,passed,failed,dgec_lat,ca3_lat,ca1_lat,total_lat,latency_us\n");
        
        fd_prediction = $fopen("prediction_detail.csv", "w");
        $fwrite(fd_prediction, "inference_id,test_name,context,context_idx,anchor_event,target_event,expected_voltage,actual_voltage,match,role,timestamp\n");
        
        fd_noise = $fopen("noise_sweep.csv", "w");
        $fwrite(fd_noise, "test_num,noise_bits,ber_pct,ctx_detected,ctx_correct,evt_detected,evt_correct,present_ok,f1_ok,f2_ok,total_correct\n");
        
        fd_heatmap = $fopen("voltage_heatmap.csv", "w");
        $fwrite(fd_heatmap, "test_name,inference_id,event_id,mcam_voltage,past_voltage,merged_voltage,timestamp,role,active\n");
        
        fd_ca1 = $fopen("ca1_ordering.csv", "w");
        $fwrite(fd_ca1, "test_name,inference_id,event_id,merged_voltage,timestamp,ordered_bit,temporal_position\n");
        
        fd_isolation = $fopen("context_isolation.csv", "w");
        $fwrite(fd_isolation, "test_num,anchor_event,ctx_a_name,ctx_a_idx,ctx_b_name,ctx_b_idx,a_present,a_f1,a_f2,b_present,b_f1,b_f2,isolation_pass\n");
        
        fd_temporal = $fopen("temporal_integration.csv", "w");
        $fwrite(fd_temporal, "inference_id,test_name,context,anchor,past_events,present_event,future1_event,future2_event,past_voltages,present_voltage,f1_voltage,f2_voltage\n");
        
        fd_latency = $fopen("latency_breakdown.csv", "w");
        $fwrite(fd_latency, "inference_id,test_name,dgec_cycles,ca3_cycles,ca1_cycles,total_cycles,dgec_us,ca3_us,ca1_us,total_us\n");
        
        $display(" [CSV] All 8 CSV files opened for IEEE paper data collection");
    endtask

    task close_csv_files();
        $fclose(fd_summary);
        $fclose(fd_prediction);
        $fclose(fd_noise);
        $fclose(fd_heatmap);
        $fclose(fd_ca1);
        $fclose(fd_isolation);
        $fclose(fd_temporal);
        $fclose(fd_latency);
        $display(" [CSV] All CSV files closed");
    endtask

    // one heatmap row per event with any nonzero voltage
    task dump_voltage_heatmap(input string test_name);
        int active_count;
        active_count = 0;
        for (int i = 0; i < NUM_EVENTS; i++) begin
            if (o_event_voltages[i] != 0 || o_past_voltages[i] != 0 || o_merged_voltages[i] != 0) begin
                $fwrite(fd_heatmap, "%s,%0d,%0d,%0d,%0d,%0d,%0d,%s,%0d\n",
                        test_name, inference_count, i, 
                        o_event_voltages[i], o_past_voltages[i], o_merged_voltages[i], 
                        o_timestamps[i], get_voltage_role(o_event_voltages[i]), 1);
                active_count++;
            end
        end
        $display("   [HEATMAP] %0d active events recorded", active_count);
    endtask

    // rows in event-index order; pos counts active events and is not the CA1 rank
    task dump_ca1_ordering(input string test_name);
        int pos;
        pos = 0;
        for (int i = 0; i < NUM_EVENTS; i++) begin
            if (o_merged_voltages[i] != 0) begin
                $fwrite(fd_ca1, "%s,%0d,%0d,%0d,%0d,%0d,%0d\n",
                        test_name, inference_count, i, 
                        o_merged_voltages[i], o_timestamps[i], 
                        o_ordered_onehot[i], pos);
                pos++;
            end
        end
    endtask

    task log_prediction(input string test_name, input int ctx, input int anchor, 
                       input int target, input logic [7:0] expected, input string role);
        logic [7:0] actual;
        int match;
        actual = o_event_voltages[target];
        match = (actual === expected) ? 1 : 0;
        $fwrite(fd_prediction, "%0d,%s,%s,%0d,%0d,%0d,%0d,%0d,%0d,%s,%0d\n",
                inference_count, test_name, get_ctx_name(ctx), ctx, anchor, target,
                expected, actual, match, role, o_timestamps[target]);
    endtask

    task log_temporal_integration(input string test_name, input int ctx, input int anchor,
                                  input int exp_present, input int exp_f1, input int exp_f2);
        string past_events_str, past_voltages_str;
        int f1_found, f2_found;
        
        past_events_str = "";
        past_voltages_str = "";
        f1_found = -1;
        f2_found = -1;
        
        // past = trace above 0 but below VP, so fresh traces are left out
        for (int i = 0; i < NUM_EVENTS; i++) begin
            if (o_past_voltages[i] > 0 && o_past_voltages[i] < VP) begin
                if (past_events_str != "") past_events_str = {past_events_str, ";"};
                past_events_str = {past_events_str, $sformatf("%0d", i)};
                if (past_voltages_str != "") past_voltages_str = {past_voltages_str, ";"};
                past_voltages_str = {past_voltages_str, $sformatf("%0d", o_past_voltages[i])};
            end
            if (o_event_voltages[i] == VF1) f1_found = i;
            if (o_event_voltages[i] == VF2) f2_found = i;
        end
        
        if (past_events_str == "") past_events_str = "none";
        if (past_voltages_str == "") past_voltages_str = "0";
        
        $fwrite(fd_temporal, "%0d,%s,%s,%0d,%s,%0d,%0d,%0d,%s,%0d,%0d,%0d\n",
                inference_count, test_name, get_ctx_name(ctx), anchor,
                past_events_str, exp_present, f1_found, f2_found,
                past_voltages_str, o_event_voltages[exp_present], 
                (f1_found >= 0) ? o_event_voltages[f1_found] : 0,
                (f2_found >= 0) ? o_event_voltages[f2_found] : 0);
    endtask

    task log_latency(input string test_name);
        real dgec_us, ca3_us, ca1_us, total_us;
        int dgec_lat, ca3_lat, ca1_lat, total_lat;
        
        dgec_lat = test_results[current_test_num-1].dgec_latency;
        ca3_lat = test_results[current_test_num-1].ca3_latency;
        ca1_lat = test_results[current_test_num-1].ca1_latency;
        total_lat = dgec_lat + ca3_lat + ca1_lat;
        
        dgec_us = real'(dgec_lat) * (1000.0 / real'(CLK_FREQ_MHZ));
        ca3_us = real'(ca3_lat) * (1000.0 / real'(CLK_FREQ_MHZ));
        ca1_us = real'(ca1_lat) * (1000.0 / real'(CLK_FREQ_MHZ));
        total_us = real'(total_lat) * (1000.0 / real'(CLK_FREQ_MHZ));
        
        $fwrite(fd_latency, "%0d,%s,%0d,%0d,%0d,%0d,%.3f,%.3f,%.3f,%.3f\n",
                inference_count, test_name, dgec_lat, ca3_lat, ca1_lat, total_lat,
                dgec_us, ca3_us, ca1_us, total_us);
    endtask

    task log_context_isolation(input int anchor, input int ctx_a, input int ctx_b,
                               input int a_present, input int a_f1, input int a_f2,
                               input int b_present, input int b_f1, input int b_f2);
        int isolation_pass;
        // isolated = the two contexts disagree on F1 or F2
        isolation_pass = (a_f1 != b_f1 || a_f2 != b_f2) ? 1 : 0;
        
        $fwrite(fd_isolation, "%0d,%0d,%s,%0d,%s,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
                current_test_num, anchor, 
                get_ctx_name(ctx_a), ctx_a, get_ctx_name(ctx_b), ctx_b,
                a_present, a_f1, a_f2, b_present, b_f1, b_f2, isolation_pass);
    endtask

    task start_test(input string name, input string desc, input string group);
        current_test_num++;
        test_results[current_test_num-1].test_name    = name;
        test_results[current_test_num-1].description  = desc;
        test_results[current_test_num-1].group        = group;
        test_results[current_test_num-1].start_cycle  = cycle_count;
        test_results[current_test_num-1].passed       = 0;
        test_results[current_test_num-1].failed       = 0;
        $display("");
        $display("################################################################################");
        $display("# TEST %0d [%s]: %s", current_test_num, group, name);
        $display("# %s", desc);
        $display("# Start Cycle: %0d", cycle_count);
        $display("################################################################################");
    endtask

    task end_test();
        test_results[current_test_num-1].end_cycle = cycle_count;
        test_results[current_test_num-1].total_latency =
            test_results[current_test_num-1].end_cycle -
            test_results[current_test_num-1].start_cycle;
        test_results[current_test_num-1].latency_us =
            real'(test_results[current_test_num-1].total_latency) *
            (1000.0 / real'(CLK_FREQ_MHZ));

        total_checks_passed += test_results[current_test_num-1].passed;
        total_checks_failed += test_results[current_test_num-1].failed;

        if (test_results[current_test_num-1].failed == 0) begin
            total_tests_passed++;
            $display(" >> TEST %0d: PASSED (%0d checks)", current_test_num, test_results[current_test_num-1].passed);
        end else begin
            total_tests_failed++;
            $display(" >> TEST %0d: FAILED (%0d passed, %0d failed)",
                     current_test_num, test_results[current_test_num-1].passed,
                     test_results[current_test_num-1].failed);
        end

        $fwrite(fd_summary, "%0d,%s,%s,%s,%0d,%0d,%0d,%0d,%0d,%0d,%.2f\n",
                current_test_num, test_results[current_test_num-1].test_name,
                test_results[current_test_num-1].group,
                (test_results[current_test_num-1].failed == 0) ? "PASS" : "FAIL",
                test_results[current_test_num-1].passed,
                test_results[current_test_num-1].failed,
                test_results[current_test_num-1].dgec_latency,
                test_results[current_test_num-1].ca3_latency,
                test_results[current_test_num-1].ca1_latency,
                test_results[current_test_num-1].total_latency,
                test_results[current_test_num-1].latency_us);
    endtask

    // give every slot a random HV first, so unused slots hold known values
    task init_all_mem_slots();
        logic [HV_WIDTH-1:0] random_hv;
        begin
            for (int i = 0; i < NUM_CONTEXTS; i++) begin
                @(posedge clk);
                random_hv = gen_random_hv();
                i_train_mode = 1;
                i_train_wr_context = 1;
                i_train_context_addr = i;
                i_train_context_data = random_hv;
                @(posedge clk);
                i_train_wr_context = 0;
                i_train_mode = 0;
            end
            for (int i = 0; i < NUM_EVENTS; i++) begin
                @(posedge clk);
                random_hv = gen_random_hv();
                i_train_mode = 1;
                i_train_wr_event = 1;
                i_train_event_addr = i;
                i_train_event_data = random_hv;
                @(posedge clk);
                i_train_wr_event = 0;
                i_train_mode = 0;
            end
            $display(" [INIT] %0d context + %0d event slots initialized", NUM_CONTEXTS, NUM_EVENTS);
        end
    endtask

    task reset_dut();
        begin
            rst_n = 0;
            i_train_mode = 0;
            i_valid = 0;
            i_context_hv = '0;
            i_event_hv = '0;
            i_train_wr_context = 0;
            i_train_context_addr = 0;
            i_train_context_data = '0;
            i_train_wr_event = 0;
            i_train_event_addr = 0;
            i_train_event_data = '0;
            i_crossbar_we = 0;
            i_crossbar_wtype = 0;
            i_crossbar_waddr1 = 0;
            i_crossbar_waddr2_assoc = 0;
            i_crossbar_waddr2_gate = 0;
            i_crossbar_wdata = 0;
            i_mcam_we = 0;
            i_mcam_event_addr = 0;
            i_mcam_context_addr = 0;
            i_mcam_data = 0;
            repeat(5) @(posedge clk);
            rst_n = 1;
            repeat(2) @(posedge clk);
            init_all_mem_slots();
            $display(" [RESET] Complete at cycle %0d", cycle_count);
        end
    endtask

    task train_context_hv(input int addr, input logic [HV_WIDTH-1:0] hv, input string name);
        begin
            @(posedge clk);
            i_train_mode = 1;
            i_train_wr_context = 1;
            i_train_context_addr = addr;
            i_train_context_data = hv;
            stored_context_hvs[addr] = hv;
            @(posedge clk);
            i_train_wr_context = 0;
            i_train_mode = 0;
            $display("   EC: Context[%0d] '%s' stored", addr, name);
        end
    endtask

    task train_event_hv(input int addr, input logic [HV_WIDTH-1:0] hv);
        begin
            @(posedge clk);
            i_train_mode = 1;
            i_train_wr_event = 1;
            i_train_event_addr = addr;
            i_train_event_data = hv;
            stored_event_hvs[addr] = hv;
            @(posedge clk);
            i_train_wr_event = 0;
            i_train_mode = 0;
        end
    endtask

    task train_crossbar_assoc(input int from_evt, input int to_evt);
        begin
            @(posedge clk);
            i_train_mode = 1;
            i_crossbar_we = 1;
            i_crossbar_wtype = 0;
            i_crossbar_waddr1 = from_evt;
            i_crossbar_waddr2_assoc = to_evt;
            i_crossbar_wdata = 1;
            @(posedge clk);
            i_crossbar_we = 0;
            i_train_mode = 0;
        end
    endtask

    task train_crossbar_gate(input int evt, input int ctx);
        begin
            @(posedge clk);
            i_train_mode = 1;
            i_crossbar_we = 1;
            i_crossbar_wtype = 1;
            i_crossbar_waddr1 = evt;
            i_crossbar_waddr2_gate = ctx;
            i_crossbar_wdata = 1;
            @(posedge clk);
            i_crossbar_we = 0;
            i_train_mode = 0;
        end
    endtask

    task train_mcam(input int evt, input int ctx, input int t_code, input string desc);
        begin
            @(posedge clk);
            i_train_mode = 1;
            i_mcam_we = 1;
            i_mcam_event_addr = evt;
            i_mcam_context_addr = ctx;
            i_mcam_data = t_code;
            @(posedge clk);
            i_mcam_we = 0;
            i_train_mode = 0;
        end
    endtask

    // one query; stage times come from the control FSM state
    task run_inference(input logic [HV_WIDTH-1:0] ctx_hv, input logic [HV_WIDTH-1:0] evt_hv, input string test_name);
        begin
            inference_count++;
            $display("   [INF #%0d] %s", inference_count, test_name);
            i_train_mode = 0;
            @(posedge clk);
            wait(o_ready);
            i_context_hv = ctx_hv;
            i_event_hv = evt_hv;
            i_valid = 1;
            @(posedge clk);
            i_valid = 0;
            dgec_start_cycle = cycle_count;

            wait(dut.u_control_fsm.state == dut.u_control_fsm.CA3_INFER);
            ca3_start_cycle = cycle_count;
            test_results[current_test_num-1].dgec_latency = ca3_start_cycle - dgec_start_cycle;

            wait(dut.u_control_fsm.state == dut.u_control_fsm.CA1_INFER);
            ca1_start_cycle = cycle_count;
            test_results[current_test_num-1].ca3_latency = ca1_start_cycle - ca3_start_cycle;

            wait(o_valid);
            test_results[current_test_num-1].ca1_latency = cycle_count - ca1_start_cycle;

            $display("   Pipeline: DG/EC=%0d CA3=%0d CA1=%0d total=%0d cycles",
                     test_results[current_test_num-1].dgec_latency,
                     test_results[current_test_num-1].ca3_latency,
                     test_results[current_test_num-1].ca1_latency,
                     cycle_count - dgec_start_cycle);
            
            log_latency(test_name);
            
            @(posedge clk);
        end
    endtask

    task verify_prediction(input int evt, input logic [7:0] exp_voltage, input string desc);
        begin
            if (o_event_voltages[evt] === exp_voltage) begin
                $display("   [PASS] E%0d = 0x%02h | %s", evt, o_event_voltages[evt], desc);
                test_results[current_test_num-1].passed++;
            end else begin
                $display("   [FAIL] E%0d = 0x%02h (exp 0x%02h) | %s",
                         evt, o_event_voltages[evt], exp_voltage, desc);
                test_results[current_test_num-1].failed++;
            end
        end
    endtask

    task verify_and_log_prediction(input string test_name, input int ctx, input int anchor,
                                   input int target, input logic [7:0] exp_voltage, input string role);
        begin
            log_prediction(test_name, ctx, anchor, target, exp_voltage, role);
            
            if (o_event_voltages[target] === exp_voltage) begin
                $display("   [PASS] E%0d = 0x%02h | %s", target, o_event_voltages[target], role);
                test_results[current_test_num-1].passed++;
            end else begin
                $display("   [FAIL] E%0d = 0x%02h (exp 0x%02h) | %s",
                         target, o_event_voltages[target], exp_voltage, role);
                test_results[current_test_num-1].failed++;
            end
        end
    endtask

    task verify_order(input int evt_future, input int evt_present, input int evt_past, input string desc);
        begin
            if (o_timestamps[evt_future] < o_timestamps[evt_present] &&
                o_timestamps[evt_present] < o_timestamps[evt_past]) begin
                $display("   [PASS] Order: E%0d < E%0d < E%0d | %s",
                         evt_future, evt_present, evt_past, desc);
                test_results[current_test_num-1].passed++;
            end else begin
                $display("   [FAIL] Order: E%0d=%0d E%0d=%0d E%0d=%0d | %s",
                         evt_future, o_timestamps[evt_future],
                         evt_present, o_timestamps[evt_present],
                         evt_past, o_timestamps[evt_past], desc);
                test_results[current_test_num-1].failed++;
            end
        end
    endtask

    task print_predictions();
        int active_count;
        begin
            active_count = 0;
            $display("   +-------+--------+--------+--------+-----------+----------+");
            $display("   | Event | MCAM   | Past   | Merged | Timestamp | Role     |");
            $display("   +-------+--------+--------+--------+-----------+----------+");
            for (int i = 0; i < NUM_EVENTS; i++) begin
                if (o_event_voltages[i] != 0 || o_past_voltages[i] != 0) begin
                    $display("   | %5d | 0x%02h | 0x%02h | 0x%02h |   0x%02h  | %-8s |",
                             i, o_event_voltages[i], o_past_voltages[i],
                             o_merged_voltages[i], o_timestamps[i],
                             get_voltage_role(o_event_voltages[i]));
                    active_count++;
                end
            end
            $display("   +-------+--------+--------+--------+-----------+----------+");
            $display("   Active events: %0d", active_count);
        end
    endtask

    initial begin
     
        $display("");
        $display("================================================================================");
        $display("  HIPPOCAMPEX IEEE VERIFICATION SUITE");
        $display("================================================================================");

        open_csv_files();
        reset_dut();

        // training
        $display("");
        $display("========================= TRAINING PHASE ======================================");

        $display(" STEP 1: Store Context Hypervectors");
        train_context_hv(CTX_BLUE,    gen_random_hv(), "Blue");
        train_context_hv(CTX_MAGENTA, gen_random_hv(), "Magenta");
        train_context_hv(CTX_GREEN,   gen_random_hv(), "Green");
        train_context_hv(CTX_RED,     gen_random_hv(), "Red");
        train_context_hv(CTX_CYAN,    gen_random_hv(), "Cyan");
        train_context_hv(CTX_YELLOW,  gen_random_hv(), "Yellow");
        train_context_hv(CTX_ORANGE,  gen_random_hv(), "Orange");
        train_context_hv(CTX_PURPLE,  gen_random_hv(), "Purple");

        $display(" STEP 2: Store Event Hypervectors");
        train_event_hv(EVT_0, gen_random_hv());
        train_event_hv(EVT_1, gen_random_hv());
        train_event_hv(EVT_2, gen_random_hv());
        train_event_hv(EVT_3, gen_random_hv());
        train_event_hv(EVT_4, gen_random_hv());
        train_event_hv(EVT_5, gen_random_hv());
        train_event_hv(EVT_6, gen_random_hv());
        train_event_hv(EVT_7, gen_random_hv());
        train_event_hv(EVT_8, gen_random_hv());
        train_event_hv(EVT_9, gen_random_hv());
        train_event_hv(EVT_10, gen_random_hv());
        train_event_hv(EVT_11, gen_random_hv());
        train_event_hv(EVT_12, gen_random_hv());
        train_event_hv(EVT_13, gen_random_hv());
        train_event_hv(EVT_14, gen_random_hv());
        train_event_hv(EVT_15, gen_random_hv());
        train_event_hv(EVT_120, gen_random_hv());
        train_event_hv(EVT_121, gen_random_hv());
        train_event_hv(EVT_122, gen_random_hv());
        train_event_hv(EVT_123, gen_random_hv());
        train_event_hv(EVT_124, gen_random_hv());
        train_event_hv(EVT_125, gen_random_hv());
        train_event_hv(EVT_126, gen_random_hv());
        train_event_hv(EVT_127, gen_random_hv());
        $display("   24 event HVs stored");

        $display(" STEP 3: Train CA3 Crossbar and MCAM Sequences");

        $display("   Blue: 7->5->3->2 (prime descending)");
        train_crossbar_assoc(EVT_7, EVT_5);
        train_crossbar_assoc(EVT_5, EVT_3);
        train_crossbar_assoc(EVT_3, EVT_2);
        train_crossbar_gate(EVT_7, CTX_BLUE);
        train_crossbar_gate(EVT_5, CTX_BLUE);
        train_crossbar_gate(EVT_3, CTX_BLUE);
        train_crossbar_gate(EVT_2, CTX_BLUE);
        train_mcam(EVT_7, CTX_BLUE, 0, "first");
        train_mcam(EVT_5, CTX_BLUE, 1, "second");
        train_mcam(EVT_3, CTX_BLUE, 2, "third");
        train_mcam(EVT_2, CTX_BLUE, 3, "fourth");

        $display("   Magenta: 7->5->3->1 (odd descending)");
        train_crossbar_assoc(EVT_3, EVT_1);
        train_crossbar_gate(EVT_7, CTX_MAGENTA);
        train_crossbar_gate(EVT_5, CTX_MAGENTA);
        train_crossbar_gate(EVT_3, CTX_MAGENTA);
        train_crossbar_gate(EVT_1, CTX_MAGENTA);
        train_mcam(EVT_7, CTX_MAGENTA, 0, "first");
        train_mcam(EVT_5, CTX_MAGENTA, 1, "second");
        train_mcam(EVT_3, CTX_MAGENTA, 2, "third");
        train_mcam(EVT_1, CTX_MAGENTA, 3, "fourth");

        $display("   Green: 2->4->6->8 (even ascending)");
        train_crossbar_assoc(EVT_2, EVT_4);
        train_crossbar_assoc(EVT_4, EVT_6);
        train_crossbar_assoc(EVT_6, EVT_8);
        train_crossbar_gate(EVT_2, CTX_GREEN);
        train_crossbar_gate(EVT_4, CTX_GREEN);
        train_crossbar_gate(EVT_6, CTX_GREEN);
        train_crossbar_gate(EVT_8, CTX_GREEN);
        train_mcam(EVT_2, CTX_GREEN, 0, "first");
        train_mcam(EVT_4, CTX_GREEN, 1, "second");
        train_mcam(EVT_6, CTX_GREEN, 2, "third");
        train_mcam(EVT_8, CTX_GREEN, 3, "fourth");

        $display("   Red: 7->9->4");
        train_crossbar_assoc(EVT_7, EVT_9);
        train_crossbar_assoc(EVT_9, EVT_4);
        train_crossbar_gate(EVT_7, CTX_RED);
        train_crossbar_gate(EVT_9, CTX_RED);
        train_crossbar_gate(EVT_4, CTX_RED);
        train_mcam(EVT_7, CTX_RED, 0, "first");
        train_mcam(EVT_9, CTX_RED, 1, "second");
        train_mcam(EVT_4, CTX_RED, 2, "third");

        $display("   Cyan: 10->11->12->13->14");
        train_crossbar_assoc(EVT_10, EVT_11);
        train_crossbar_assoc(EVT_11, EVT_12);
        train_crossbar_assoc(EVT_12, EVT_13);
        train_crossbar_assoc(EVT_13, EVT_14);
        train_crossbar_gate(EVT_10, CTX_CYAN);
        train_crossbar_gate(EVT_11, CTX_CYAN);
        train_crossbar_gate(EVT_12, CTX_CYAN);
        train_crossbar_gate(EVT_13, CTX_CYAN);
        train_crossbar_gate(EVT_14, CTX_CYAN);
        train_mcam(EVT_10, CTX_CYAN, 0, "first");
        train_mcam(EVT_11, CTX_CYAN, 1, "second");
        train_mcam(EVT_12, CTX_CYAN, 2, "third");
        train_mcam(EVT_13, CTX_CYAN, 3, "fourth");
        train_mcam(EVT_14, CTX_CYAN, 4, "fifth");

        $display("   Yellow: 0->1->2->3->4->5");
        train_crossbar_assoc(EVT_0, EVT_1);
        train_crossbar_assoc(EVT_1, EVT_2);
        train_crossbar_assoc(EVT_2, EVT_3);
        train_crossbar_assoc(EVT_3, EVT_4);
        train_crossbar_assoc(EVT_4, EVT_5);
        train_crossbar_gate(EVT_0, CTX_YELLOW);
        train_crossbar_gate(EVT_1, CTX_YELLOW);
        train_crossbar_gate(EVT_2, CTX_YELLOW);
        train_crossbar_gate(EVT_3, CTX_YELLOW);
        train_crossbar_gate(EVT_4, CTX_YELLOW);
        train_crossbar_gate(EVT_5, CTX_YELLOW);
        train_mcam(EVT_0, CTX_YELLOW, 0, "first");
        train_mcam(EVT_1, CTX_YELLOW, 1, "second");
        train_mcam(EVT_2, CTX_YELLOW, 2, "third");
        train_mcam(EVT_3, CTX_YELLOW, 3, "fourth");
        train_mcam(EVT_4, CTX_YELLOW, 4, "fifth");
        train_mcam(EVT_5, CTX_YELLOW, 5, "sixth");

        // Orange and Purple use the remapped slots 16-23
        $display("   Orange: 120->121->122->123");
        train_crossbar_assoc(EVT_120, EVT_121);
        train_crossbar_assoc(EVT_121, EVT_122);
        train_crossbar_assoc(EVT_122, EVT_123);
        train_crossbar_gate(EVT_120, CTX_ORANGE);
        train_crossbar_gate(EVT_121, CTX_ORANGE);
        train_crossbar_gate(EVT_122, CTX_ORANGE);
        train_crossbar_gate(EVT_123, CTX_ORANGE);
        train_mcam(EVT_120, CTX_ORANGE, 0, "first");
        train_mcam(EVT_121, CTX_ORANGE, 1, "second");
        train_mcam(EVT_122, CTX_ORANGE, 2, "third");
        train_mcam(EVT_123, CTX_ORANGE, 3, "fourth");

        $display("   Purple: 124->125->126->127");
        train_crossbar_assoc(EVT_124, EVT_125);
        train_crossbar_assoc(EVT_125, EVT_126);
        train_crossbar_assoc(EVT_126, EVT_127);
        train_crossbar_gate(EVT_124, CTX_PURPLE);
        train_crossbar_gate(EVT_125, CTX_PURPLE);
        train_crossbar_gate(EVT_126, CTX_PURPLE);
        train_crossbar_gate(EVT_127, CTX_PURPLE);
        train_mcam(EVT_124, CTX_PURPLE, 0, "first");
        train_mcam(EVT_125, CTX_PURPLE, 1, "second");
        train_mcam(EVT_126, CTX_PURPLE, 2, "third");
        train_mcam(EVT_127, CTX_PURPLE, 3, "fourth");

        $display(" [TRAIN] All 8 contexts trained");
        $display("=========================================================================");
        repeat(5) @(posedge clk);

        // A: cue the first event of every context
        start_test("A1_Blue", "Blue ctx 7->5->3->2 (prime desc), anchor=7", "GroupA");
        run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7], "Blue/E7");
        print_predictions();
        verify_and_log_prediction("A1_Blue", CTX_BLUE, EVT_7, EVT_7, VP,    "Present");
        verify_and_log_prediction("A1_Blue", CTX_BLUE, EVT_7, EVT_5, VF1,   "Future1");
        verify_and_log_prediction("A1_Blue", CTX_BLUE, EVT_7, EVT_3, VF2,   "Future2");
        verify_and_log_prediction("A1_Blue", CTX_BLUE, EVT_7, EVT_2, 8'h00, "Beyond");
        log_temporal_integration("A1_Blue", CTX_BLUE, EVT_7, EVT_7, EVT_5, EVT_3);
        verify_order(EVT_3, EVT_5, EVT_7, "F2 < F1 < Present timestamps");
        dump_voltage_heatmap("A1_Blue");
        dump_ca1_ordering("A1_Blue");
        repeat(10) @(posedge clk);
        end_test();

        start_test("A2_Magenta", "Magenta ctx 7->5->3->1 (odd desc), anchor=7", "GroupA");
        run_inference(stored_context_hvs[CTX_MAGENTA], stored_event_hvs[EVT_7], "Magenta/E7");
        print_predictions();
        verify_and_log_prediction("A2_Magenta", CTX_MAGENTA, EVT_7, EVT_7, VP,    "Present");
        verify_and_log_prediction("A2_Magenta", CTX_MAGENTA, EVT_7, EVT_5, VF1,   "Future1");
        verify_and_log_prediction("A2_Magenta", CTX_MAGENTA, EVT_7, EVT_3, VF2,   "Future2");
        verify_and_log_prediction("A2_Magenta", CTX_MAGENTA, EVT_7, EVT_1, 8'h00, "Beyond");
        verify_and_log_prediction("A2_Magenta", CTX_MAGENTA, EVT_7, EVT_2, 8'h00, "NotInCtx");
        log_temporal_integration("A2_Magenta", CTX_MAGENTA, EVT_7, EVT_7, EVT_5, EVT_3);
        dump_voltage_heatmap("A2_Magenta");
        dump_ca1_ordering("A2_Magenta");
        repeat(10) @(posedge clk);
        end_test();

        start_test("A3_Green", "Green ctx 2->4->6->8 (even asc), anchor=2", "GroupA");
        run_inference(stored_context_hvs[CTX_GREEN], stored_event_hvs[EVT_2], "Green/E2");
        print_predictions();
        verify_and_log_prediction("A3_Green", CTX_GREEN, EVT_2, EVT_2, VP,    "Present");
        verify_and_log_prediction("A3_Green", CTX_GREEN, EVT_2, EVT_4, VF1,   "Future1");
        verify_and_log_prediction("A3_Green", CTX_GREEN, EVT_2, EVT_6, VF2,   "Future2");
        verify_and_log_prediction("A3_Green", CTX_GREEN, EVT_2, EVT_8, 8'h00, "Beyond");
        log_temporal_integration("A3_Green", CTX_GREEN, EVT_2, EVT_2, EVT_4, EVT_6);
        dump_voltage_heatmap("A3_Green");
        dump_ca1_ordering("A3_Green");
        repeat(10) @(posedge clk);
        end_test();

        start_test("A4_Red", "Red ctx 7->9->4, anchor=7", "GroupA");
        run_inference(stored_context_hvs[CTX_RED], stored_event_hvs[EVT_7], "Red/E7");
        print_predictions();
        verify_and_log_prediction("A4_Red", CTX_RED, EVT_7, EVT_7, VP,    "Present");
        verify_and_log_prediction("A4_Red", CTX_RED, EVT_7, EVT_9, VF1,   "Future1");
        verify_and_log_prediction("A4_Red", CTX_RED, EVT_7, EVT_4, VF2,   "Future2");
        log_temporal_integration("A4_Red", CTX_RED, EVT_7, EVT_7, EVT_9, EVT_4);
        dump_voltage_heatmap("A4_Red");
        repeat(10) @(posedge clk);
        end_test();

        start_test("A5_Cyan", "Cyan ctx 10->11->12->13->14, anchor=10", "GroupA");
        run_inference(stored_context_hvs[CTX_CYAN], stored_event_hvs[EVT_10], "Cyan/E10");
        print_predictions();
        verify_and_log_prediction("A5_Cyan", CTX_CYAN, EVT_10, EVT_10, VP,    "Present");
        verify_and_log_prediction("A5_Cyan", CTX_CYAN, EVT_10, EVT_11, VF1,   "Future1");
        verify_and_log_prediction("A5_Cyan", CTX_CYAN, EVT_10, EVT_12, VF2,   "Future2");
        log_temporal_integration("A5_Cyan", CTX_CYAN, EVT_10, EVT_10, EVT_11, EVT_12);
        dump_voltage_heatmap("A5_Cyan");
        repeat(10) @(posedge clk);
        end_test();

        start_test("A6_Yellow", "Yellow ctx 0->1->2->3->4->5, anchor=0", "GroupA");
        run_inference(stored_context_hvs[CTX_YELLOW], stored_event_hvs[EVT_0], "Yellow/E0");
        print_predictions();
        verify_and_log_prediction("A6_Yellow", CTX_YELLOW, EVT_0, EVT_0, VP,    "Present");
        verify_and_log_prediction("A6_Yellow", CTX_YELLOW, EVT_0, EVT_1, VF1,   "Future1");
        verify_and_log_prediction("A6_Yellow", CTX_YELLOW, EVT_0, EVT_2, VF2,   "Future2");
        log_temporal_integration("A6_Yellow", CTX_YELLOW, EVT_0, EVT_0, EVT_1, EVT_2);
        dump_voltage_heatmap("A6_Yellow");
        repeat(10) @(posedge clk);
        end_test();

        start_test("A7_Orange", "Orange ctx 120->121->122->123, anchor=120", "GroupA");
        run_inference(stored_context_hvs[CTX_ORANGE], stored_event_hvs[EVT_120], "Orange/E120");
        print_predictions();
        verify_and_log_prediction("A7_Orange", CTX_ORANGE, EVT_120, EVT_120, VP,    "Present");
        verify_and_log_prediction("A7_Orange", CTX_ORANGE, EVT_120, EVT_121, VF1,   "Future1");
        verify_and_log_prediction("A7_Orange", CTX_ORANGE, EVT_120, EVT_122, VF2,   "Future2");
        log_temporal_integration("A7_Orange", CTX_ORANGE, EVT_120, EVT_120, EVT_121, EVT_122);
        dump_voltage_heatmap("A7_Orange");
        repeat(10) @(posedge clk);
        end_test();

        start_test("A8_Purple", "Purple ctx 124->125->126->127, anchor=124", "GroupA");
        run_inference(stored_context_hvs[CTX_PURPLE], stored_event_hvs[EVT_124], "Purple/E124");
        print_predictions();
        verify_and_log_prediction("A8_Purple", CTX_PURPLE, EVT_124, EVT_124, VP,    "Present");
        verify_and_log_prediction("A8_Purple", CTX_PURPLE, EVT_124, EVT_125, VF1,   "Future1");
        verify_and_log_prediction("A8_Purple", CTX_PURPLE, EVT_124, EVT_126, VF2,   "Future2");
        log_temporal_integration("A8_Purple", CTX_PURPLE, EVT_124, EVT_124, EVT_125, EVT_126);
        dump_voltage_heatmap("A8_Purple");
        repeat(10) @(posedge clk);
        end_test();

        // B: flip 0-13 bits on both cue HVs. Stops at 13 because past that no
        // row passes the threshold, and the old notes say the pipeline then
        // never finishes. TODO: give the DUT a timeout or a no-match path.
        begin
            int noise_levels[10];
            int ni, nflip;
            logic [HV_WIDTH-1:0] noisy_ctx, noisy_evt;
            bit ctx_correct, evt_correct, present_ok, f1_ok, f2_ok;
            int total_correct;

            noise_levels[0]  = 0;   noise_levels[1]  = 1;   noise_levels[2]  = 2;
            noise_levels[3]  = 3;   noise_levels[4]  = 5;   noise_levels[5]  = 7;
            noise_levels[6]  = 8;   noise_levels[7]  = 10;  noise_levels[8]  = 12;
            noise_levels[9]  = 13;  // = MAX_MISMATCH

            for (ni = 0; ni < 10; ni++) begin
                nflip = noise_levels[ni];
                start_test($sformatf("B%0d_Noise%0d", ni+1, nflip),
                           $sformatf("Blue ctx with %0d bit errors (%.1f%% BER)", nflip, real'(nflip)/real'(HV_WIDTH)*100.0),
                           "GroupB");

                noisy_ctx = flip_bits(stored_context_hvs[CTX_BLUE], nflip);
                noisy_evt = flip_bits(stored_event_hvs[EVT_7], nflip);
                run_inference(noisy_ctx, noisy_evt, $sformatf("Blue/E7+%0dbit", nflip));

                ctx_correct = (o_context_idx == CTX_BLUE);
                evt_correct = (o_event_idx == EVT_7);
                present_ok  = (o_event_voltages[EVT_7] == VP);
                f1_ok       = (o_event_voltages[EVT_5] == VF1);
                f2_ok       = (o_event_voltages[EVT_3] == VF2);
                total_correct = ctx_correct + evt_correct + present_ok + f1_ok + f2_ok;

                if (ctx_correct) test_results[current_test_num-1].passed++;
                else test_results[current_test_num-1].failed++;
                if (evt_correct) test_results[current_test_num-1].passed++;
                else test_results[current_test_num-1].failed++;

                $fwrite(fd_noise, "%0d,%0d,%.2f,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
                        current_test_num, nflip, real'(nflip)/real'(HV_WIDTH)*100.0,
                        o_context_idx, ctx_correct, o_event_idx, evt_correct,
                        present_ok, f1_ok, f2_ok, total_correct);

                log_prediction($sformatf("B%0d_Noise%0d", ni+1, nflip), CTX_BLUE, EVT_7, EVT_7, VP, "Present");
                log_prediction($sformatf("B%0d_Noise%0d", ni+1, nflip), CTX_BLUE, EVT_7, EVT_5, VF1, "Future1");
                log_prediction($sformatf("B%0d_Noise%0d", ni+1, nflip), CTX_BLUE, EVT_7, EVT_3, VF2, "Future2");

                dump_voltage_heatmap($sformatf("B%0d_Noise%0d", ni+1, nflip));
                repeat(5) @(posedge clk);
                end_test();
            end
        end

        // C: move the cue along Blue (7, 5, 3), then Green at 4
        start_test("C1_Blue_Slide7", "Blue sliding window anchor=7 (start)", "GroupC");
        run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7], "Blue/E7");
        verify_and_log_prediction("C1_Blue_Slide7", CTX_BLUE, EVT_7, EVT_7, VP,  "Present");
        verify_and_log_prediction("C1_Blue_Slide7", CTX_BLUE, EVT_7, EVT_5, VF1, "Future1");
        verify_and_log_prediction("C1_Blue_Slide7", CTX_BLUE, EVT_7, EVT_3, VF2, "Future2");
        log_temporal_integration("C1_Blue_Slide7", CTX_BLUE, EVT_7, EVT_7, EVT_5, EVT_3);
        dump_voltage_heatmap("C1_Blue_Slide7");
        dump_ca1_ordering("C1_Blue_Slide7");
        repeat(10) @(posedge clk);
        end_test();

        start_test("C2_Blue_Slide5", "Blue sliding window anchor=5 (mid)", "GroupC");
        run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_5], "Blue/E5");
        verify_and_log_prediction("C2_Blue_Slide5", CTX_BLUE, EVT_5, EVT_5, VP,  "Present");
        verify_and_log_prediction("C2_Blue_Slide5", CTX_BLUE, EVT_5, EVT_3, VF1, "Future1");
        verify_and_log_prediction("C2_Blue_Slide5", CTX_BLUE, EVT_5, EVT_2, VF2, "Future2");
        verify_and_log_prediction("C2_Blue_Slide5", CTX_BLUE, EVT_5, EVT_7, 8'h00, "Past");
        log_temporal_integration("C2_Blue_Slide5", CTX_BLUE, EVT_5, EVT_5, EVT_3, EVT_2);
        dump_voltage_heatmap("C2_Blue_Slide5");
        dump_ca1_ordering("C2_Blue_Slide5");
        repeat(10) @(posedge clk);
        end_test();

        start_test("C3_Blue_Slide3", "Blue sliding window anchor=3 (near end)", "GroupC");
        run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_3], "Blue/E3");
        verify_and_log_prediction("C3_Blue_Slide3", CTX_BLUE, EVT_3, EVT_3, VP,  "Present");
        verify_and_log_prediction("C3_Blue_Slide3", CTX_BLUE, EVT_3, EVT_2, VF1, "Future1");
        verify_and_log_prediction("C3_Blue_Slide3", CTX_BLUE, EVT_3, EVT_5, 8'h00, "Past");
        log_temporal_integration("C3_Blue_Slide3", CTX_BLUE, EVT_3, EVT_3, EVT_2, -1);
        dump_voltage_heatmap("C3_Blue_Slide3");
        dump_ca1_ordering("C3_Blue_Slide3");
        repeat(10) @(posedge clk);
        end_test();

        start_test("C4_Green_Slide4", "Green sliding window anchor=4", "GroupC");
        run_inference(stored_context_hvs[CTX_GREEN], stored_event_hvs[EVT_4], "Green/E4");
        verify_and_log_prediction("C4_Green_Slide4", CTX_GREEN, EVT_4, EVT_4, VP,  "Present");
        verify_and_log_prediction("C4_Green_Slide4", CTX_GREEN, EVT_4, EVT_6, VF1, "Future1");
        verify_and_log_prediction("C4_Green_Slide4", CTX_GREEN, EVT_4, EVT_8, VF2, "Future2");
        log_temporal_integration("C4_Green_Slide4", CTX_GREEN, EVT_4, EVT_4, EVT_6, EVT_8);
        dump_voltage_heatmap("C4_Green_Slide4");
        repeat(10) @(posedge clk);
        end_test();

        // D1: cue 7 in Blue, then in Red. The f1/f2 values logged to the
        // isolation CSV are filled in by hand, not read from the DUT.
        start_test("D1_CtxSwitch_BluRed", "Event 7 in Blue vs Red - different futures", "GroupD");
        begin
            int blue_f1, blue_f2, red_f1, red_f2;
            
            $display("   --- Blue Context ---");
            run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7], "Blue/E7");
            verify_and_log_prediction("D1_CtxSwitch_BluRed", CTX_BLUE, EVT_7, EVT_7, VP,    "Present");
            verify_and_log_prediction("D1_CtxSwitch_BluRed", CTX_BLUE, EVT_7, EVT_5, VF1,   "Future1");
            verify_and_log_prediction("D1_CtxSwitch_BluRed", CTX_BLUE, EVT_7, EVT_9, 8'h00, "NotInBlue");
            blue_f1 = EVT_5;
            blue_f2 = EVT_3;
            dump_voltage_heatmap("D1_Blue");
            repeat(5) @(posedge clk);
            
            $display("   --- Red Context ---");
            run_inference(stored_context_hvs[CTX_RED], stored_event_hvs[EVT_7], "Red/E7");
            verify_and_log_prediction("D1_CtxSwitch_BluRed", CTX_RED, EVT_7, EVT_7, VP,    "Present");
            verify_and_log_prediction("D1_CtxSwitch_BluRed", CTX_RED, EVT_7, EVT_9, VF1,   "Future1");
            verify_and_log_prediction("D1_CtxSwitch_BluRed", CTX_RED, EVT_7, EVT_5, 8'h00, "NotInRed");
            red_f1 = EVT_9;
            red_f2 = EVT_4;
            dump_voltage_heatmap("D1_Red");
            
            log_context_isolation(EVT_7, CTX_BLUE, CTX_RED, EVT_7, blue_f1, blue_f2, EVT_7, red_f1, red_f2);
        end
        repeat(10) @(posedge clk);
        end_test();

        // D2: same for Blue vs Magenta. Nothing is checked; it always passes.
        start_test("D2_CtxSwitch_BluMag", "Event 7 in Blue vs Magenta - same start diff end", "GroupD");
        begin
            int blue_f1, blue_f2, mag_f1, mag_f2;
            
            $display("   --- Blue Context (7->5->3->2) ---");
            run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7], "Blue/E7");
            blue_f1 = EVT_5;
            blue_f2 = EVT_3;
            dump_voltage_heatmap("D2_Blue");
            repeat(5) @(posedge clk);
            
            $display("   --- Magenta Context (7->5->3->1) ---");
            run_inference(stored_context_hvs[CTX_MAGENTA], stored_event_hvs[EVT_7], "Magenta/E7");
            mag_f1 = EVT_5;
            mag_f2 = EVT_3;
            dump_voltage_heatmap("D2_Magenta");
            
            // F1/F2 agree here, so this row logs isolation_pass=0
            log_context_isolation(EVT_7, CTX_BLUE, CTX_MAGENTA, EVT_7, blue_f1, blue_f2, EVT_7, mag_f1, mag_f2);
            
            test_results[current_test_num-1].passed++;
        end
        repeat(10) @(posedge clk);
        end_test();

        // D3: Blue, Green, Blue back to back
        start_test("D3_RapidAlt", "Rapid Blue<->Green<->Blue alternation", "GroupD");
        run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7], "Blue/E7");
        verify_and_log_prediction("D3_RapidAlt", CTX_BLUE, EVT_7, EVT_5, VF1, "Blue_F1");
        dump_voltage_heatmap("D3_Blue1");
        repeat(3) @(posedge clk);
        
        run_inference(stored_context_hvs[CTX_GREEN], stored_event_hvs[EVT_2], "Green/E2");
        verify_and_log_prediction("D3_RapidAlt", CTX_GREEN, EVT_2, EVT_4, VF1, "Green_F1");
        dump_voltage_heatmap("D3_Green");
        repeat(3) @(posedge clk);
        
        run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7], "Blue/E7_again");
        verify_and_log_prediction("D3_RapidAlt", CTX_BLUE, EVT_7, EVT_5, VF1, "Blue_F1_again");
        dump_voltage_heatmap("D3_Blue2");
        repeat(10) @(posedge clk);
        end_test();

        // E1: cue on the last Green event
        start_test("E1_SeqEnd", "Green anchor=8 (sequence terminal)", "GroupE");
        run_inference(stored_context_hvs[CTX_GREEN], stored_event_hvs[EVT_8], "Green/E8");
        verify_and_log_prediction("E1_SeqEnd", CTX_GREEN, EVT_8, EVT_8, VP,    "Present_Terminal");
        verify_and_log_prediction("E1_SeqEnd", CTX_GREEN, EVT_8, EVT_6, 8'h00, "Past");
        log_temporal_integration("E1_SeqEnd", CTX_GREEN, EVT_8, EVT_8, -1, -1);
        dump_voltage_heatmap("E1_SeqEnd");
        repeat(10) @(posedge clk);
        end_test();

        // E2: event 15 has no links
        start_test("E2_Isolated", "E15 with no trained associations", "GroupE");
        run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_15], "Blue/E15");
        verify_and_log_prediction("E2_Isolated", CTX_BLUE, EVT_15, EVT_15, VP, "Present_Isolated");
        verify_and_log_prediction("E2_Isolated", CTX_BLUE, EVT_15, EVT_5, 8'h00, "NoChain");
        dump_voltage_heatmap("E2_Isolated");
        repeat(10) @(posedge clk);
        end_test();

        // F: six-event Yellow chain, cue at 0 and then at 3
        start_test("F1_Yellow_Start", "Yellow 6-event chain from anchor=0", "GroupF");
        run_inference(stored_context_hvs[CTX_YELLOW], stored_event_hvs[EVT_0], "Yellow/E0");
        print_predictions();
        verify_and_log_prediction("F1_Yellow_Start", CTX_YELLOW, EVT_0, EVT_0, VP,  "Present");
        verify_and_log_prediction("F1_Yellow_Start", CTX_YELLOW, EVT_0, EVT_1, VF1, "Future1");
        verify_and_log_prediction("F1_Yellow_Start", CTX_YELLOW, EVT_0, EVT_2, VF2, "Future2");
        verify_and_log_prediction("F1_Yellow_Start", CTX_YELLOW, EVT_0, EVT_3, 8'h00, "Beyond");
        log_temporal_integration("F1_Yellow_Start", CTX_YELLOW, EVT_0, EVT_0, EVT_1, EVT_2);
        dump_voltage_heatmap("F1_Yellow_Start");
        dump_ca1_ordering("F1_Yellow_Start");
        repeat(10) @(posedge clk);
        end_test();

        start_test("F2_Yellow_Mid", "Yellow 6-event chain from anchor=3", "GroupF");
        run_inference(stored_context_hvs[CTX_YELLOW], stored_event_hvs[EVT_3], "Yellow/E3");
        print_predictions();
        verify_and_log_prediction("F2_Yellow_Mid", CTX_YELLOW, EVT_3, EVT_3, VP,  "Present");
        verify_and_log_prediction("F2_Yellow_Mid", CTX_YELLOW, EVT_3, EVT_4, VF1, "Future1");
        verify_and_log_prediction("F2_Yellow_Mid", CTX_YELLOW, EVT_3, EVT_5, VF2, "Future2");
        verify_and_log_prediction("F2_Yellow_Mid", CTX_YELLOW, EVT_3, EVT_0, 8'h00, "Past");
        log_temporal_integration("F2_Yellow_Mid", CTX_YELLOW, EVT_3, EVT_3, EVT_4, EVT_5);
        dump_voltage_heatmap("F2_Yellow_Mid");
        repeat(10) @(posedge clk);
        end_test();

        // G: ten recalls in a row, timed. Always passes.

        start_test("G1_Throughput", "10 back-to-back inferences for throughput", "GroupG");
        begin
            int start_cyc, total_cyc, ii;
            start_cyc = cycle_count;
            for (ii = 0; ii < 10; ii++) begin
                run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7], $sformatf("iter%0d", ii));
                repeat(2) @(posedge clk);
            end
            total_cyc = cycle_count - start_cyc;
            $display("   10 inferences in %0d cycles (avg %.1f cyc/inf)", total_cyc, real'(total_cyc)/10.0);
            test_results[current_test_num-1].passed++;
        end
        repeat(10) @(posedge clk);
        end_test();

        begin
            int total_checks;
            real pass_rate;
            total_checks = total_checks_passed + total_checks_failed;
            pass_rate = (total_checks > 0) ? (real'(total_checks_passed) / real'(total_checks)) * 100.0 : 0.0;

            $display("");
            $display("================================================================================");
            $display("  HIPPOCAMPEX IEEE VERIFICATION SUITE - FINAL REPORT");
            $display("================================================================================");
            $display("  Total Tests:      %0d", current_test_num);
            $display("  Tests Passed:     %0d", total_tests_passed);
            $display("  Tests Failed:     %0d", total_tests_failed);
            $display("  Checks Passed:    %0d", total_checks_passed);
            $display("  Checks Failed:    %0d", total_checks_failed);
            $display("  Pass Rate:        %.1f%%", pass_rate);
            $display("  Total Inferences: %0d", inference_count);
            $display("");
            $display("  CSV Files Generated:");
            $display("    - test_summary.csv        (test results & timing)");
            $display("    - prediction_detail.csv   (per-event predictions)");
            $display("    - noise_sweep.csv         (BER vs accuracy)");
            $display("    - voltage_heatmap.csv     (full voltage snapshots)");
            $display("    - ca1_ordering.csv        (temporal ordering)");
            $display("    - context_isolation.csv   (cross-context analysis)");
            $display("    - temporal_integration.csv(past/present/future)");
            $display("    - latency_breakdown.csv   (per-stage timing)");
            $display("");
            if (total_tests_failed == 0) begin
                $display("  *** ALL TESTS PASSED ***");
            end else begin
                $display("  *** %0d TEST(S) FAILED ***", total_tests_failed);
            end
            $display("================================================================================");
        end

        close_csv_files();
        $finish;
    end

    // watchdog, 1 ms
    initial begin
        #(CLK_PERIOD * 100000);
        $display("");
        $display("[ERROR] Testbench timeout after 100000 cycles");
        $display(" Last completed test: %0d / %0d", current_test_num, MAX_TESTS);
        close_csv_files();
        $finish;
    end

endmodule
