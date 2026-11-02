// testbench_hippocampex: full pipeline (DG/EC -> CA3 -> CA1) on
// hippocampex_top directly, no AXI. Used for the presentation runs.
// Cases:
//   1. Blue, primes 7->5->3->2: cue at the start of the chain
//   2. Green, evens 2->4->6->8: noisy cue in the middle of the chain
//   3. Same cue 7 in Blue vs Magenta: different predictions
//   4. Moving the cue along 7 -> 5 -> 3
//   5. Longer chain 7->5->3->2->1
//   6. Chain with a missing link
//   7. Overlapping events in two contexts with different futures
// Prints per-stage cycle counts and checks that CA1 timestamps run
// future -> present -> past.
`timescale 1ns/1ps

module testbench_hippocampex;

    localparam HV_WIDTH = 128;
    localparam NUM_CONTEXTS = 16;
    localparam NUM_EVENTS = 128;
    localparam DIST_WIDTH = 8;
    localparam MAX_MISMATCH = 13;
    localparam MAX_STEPS = 8;
    localparam CNTR_WIDTH = 6;
    localparam VOLTAGE_WIDTH = 8;
    localparam DECAY_STEP = 4;
    localparam CLK_PERIOD = 10; // 100 MHz

    // MCAM tags
    localparam VP  = 8'h40; // present
    localparam VF1 = 8'h80; // future-1
    localparam VF2 = 8'hC0; // future-2

    // Only printed next to the measured averages. These numbers are from the ramp-sorter build; with the bitonic sorter DG/EC and CA1 finish much sooner.
    localparam EXPECTED_DGEC_LATENCY = 260;
    localparam EXPECTED_CA3_LATENCY  = 10;
    localparam EXPECTED_CA1_LATENCY  = 256;
    localparam EXPECTED_TOTAL_LATENCY = 526;
    localparam CLK_FREQ_MHZ = 100;

    localparam CTX_BLUE    = 0;
    localparam CTX_MAGENTA = 1;
    localparam CTX_GREEN   = 2;
    localparam CTX_RED     = 3;  // only used in test 7
    localparam EVT_7 = 7, EVT_5 = 5, EVT_3 = 3, EVT_2 = 2, EVT_1 = 1;
    localparam EVT_4 = 4, EVT_6 = 6, EVT_8 = 8;
    localparam EVT_9 = 9;  // in Red, and appended to Blue in test 5

    // per-test latency and pass/fail counts
    typedef struct {
        string test_name;
        string description;
        int start_cycle;
        int end_cycle;
        int dgec_latency;
        int ca3_latency;
        int ca1_latency;
        int total_latency;
        real latency_us;
        int passed;
        int failed;
    } test_metrics_t;

    test_metrics_t test_results [7];
    int current_test_num = 0;
    int total_tests_passed = 0;
    int total_tests_failed = 0;

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

    logic [NUM_CONTEXTS-1:0] o_context_onehot;
    logic [NUM_EVENTS-1:0]   o_event_onehot;
    logic [$clog2(NUM_CONTEXTS)-1:0] o_context_idx;
    logic [$clog2(NUM_EVENTS)-1:0]   o_event_idx;

    logic o_valid, o_ready;

    int cycle_count = 0;
    int dgec_start_cycle = 0;
    int ca3_start_cycle = 0;
    int ca1_start_cycle = 0;

    // copies of the trained HVs, reused as queries
    logic [HV_WIDTH-1:0] stored_context_hvs [NUM_CONTEXTS];
    logic [HV_WIDTH-1:0] stored_event_hvs [NUM_EVENTS];

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

    // sparse random HV: at most 12 ones (128/10), fewer if a position repeats
    function automatic logic [HV_WIDTH-1:0] gen_random_hv();
        logic [HV_WIDTH-1:0] hv;
        int ones_count;
        hv = '0;
        ones_count = HV_WIDTH / 10;
        for (int i = 0; i < ones_count; i++) begin
            int pos = $urandom_range(0, HV_WIDTH-1);
            hv[pos] = 1'b1;
        end
        return hv;
    endfunction

    // flip n random bits; positions can repeat, so fewer may actually change
    function automatic logic [HV_WIDTH-1:0] flip_bits(logic [HV_WIDTH-1:0] hv, int n);
        logic [HV_WIDTH-1:0] result;
        result = hv;
        for (int i = 0; i < n; i++) begin
            int pos = $urandom_range(0, HV_WIDTH-1);
            result[pos] = ~result[pos];
        end
        return result;
    endfunction

    function automatic real calc_bit_error_percent(int num_flipped_bits);
        return (real'(num_flipped_bits) / real'(HV_WIDTH)) * 100.0;
    endfunction

    task start_test(input string name, input string desc);
        begin
            current_test_num++;
            test_results[current_test_num-1].test_name = name;
            test_results[current_test_num-1].description = desc;
            test_results[current_test_num-1].start_cycle = cycle_count;
            test_results[current_test_num-1].passed = 0;
            test_results[current_test_num-1].failed = 0;
            $display("");
            $display("================================================================================");
            $display(" TEST CASE %0d ", current_test_num);
            $display("================================================================================");
            $display(" Name: %s", name);
            $display(" Description: %s", desc);
            $display(" Start Cycle: %0d", cycle_count);
            $display("================================================================================");
        end
    endtask

    task end_test();
        begin
            test_results[current_test_num-1].end_cycle = cycle_count;
            test_results[current_test_num-1].total_latency =
                test_results[current_test_num-1].end_cycle - test_results[current_test_num-1].start_cycle;
            test_results[current_test_num-1].latency_us =
                real'(test_results[current_test_num-1].total_latency) * (1000.0 / real'(CLK_FREQ_MHZ));
            if (test_results[current_test_num-1].failed == 0) begin
                total_tests_passed++;
                $display("");
                $display("+------------------------------------------------------------------------------+");
                $display("| TEST CASE %0d: PASSED |", current_test_num);
                $display("+------------------------------------------------------------------------------+");
            end else begin
                total_tests_failed++;
                $display("");
                $display("+------------------------------------------------------------------------------+");
                $display("| TEST CASE %0d: FAILED (%0d checks failed) |",
                         current_test_num, test_results[current_test_num-1].failed);
                $display("+------------------------------------------------------------------------------+");
            end
            $display(" Total Cycles: %0d", test_results[current_test_num-1].total_latency);
            $display(" DG/EC Latency: %0d cycles", test_results[current_test_num-1].dgec_latency);
            $display(" CA3 Latency: %0d cycles", test_results[current_test_num-1].ca3_latency);
            $display(" CA1 Latency: %0d cycles", test_results[current_test_num-1].ca1_latency);
            $display(" Latency (us): %.2f us @ %0d MHz",
                     test_results[current_test_num-1].latency_us, CLK_FREQ_MHZ);
            $display(" Checks Passed: %0d", test_results[current_test_num-1].passed);
            $display(" Checks Failed: %0d", test_results[current_test_num-1].failed);
            $display("+------------------------------------------------------------------------------+");
        end
    endtask

    task print_test_summary();
        int i;
        real total_latency_us = 0.0;
        int avg_dgec_latency = 0;
        int avg_ca3_latency = 0;
        int avg_ca1_latency = 0;
        begin
            $display("");
            $display("================================================================================");
            $display(" HIPPOCAMPEX TEST SUMMARY ");
            $display("================================================================================");
            $display("");
            $display(" Total Tests Run: %0d", current_test_num);
            $display(" Tests Passed: %0d", total_tests_passed);
            $display(" Tests Failed: %0d", total_tests_failed);
            $display("");
            $display("+------------------------------------------------------------------------------+");
            $display("| Test | Test Name | Cycles | DGEC | CA3 | CA1 | Pass/Fail |");
            $display("+------------------------------------------------------------------------------+");
            for (i = 0; i < current_test_num; i++) begin
                $display("| %0d | %-33s | %6d | %4d | %3d | %3d | %-9s |",
                        i+1,
                        test_results[i].test_name,
                        test_results[i].total_latency,
                        test_results[i].dgec_latency,
                        test_results[i].ca3_latency,
                        test_results[i].ca1_latency,
                        (test_results[i].failed == 0) ? "PASS" : "FAIL");
                total_latency_us += test_results[i].latency_us;
                avg_dgec_latency += test_results[i].dgec_latency;
                avg_ca3_latency += test_results[i].ca3_latency;
                avg_ca1_latency += test_results[i].ca1_latency;
            end
            $display("+------------------------------------------------------------------------------+");
            $display("");
            if (current_test_num > 0) begin
                avg_dgec_latency = avg_dgec_latency / current_test_num;
                avg_ca3_latency = avg_ca3_latency / current_test_num;
                avg_ca1_latency = avg_ca1_latency / current_test_num;
            end
            $display(" PERFORMANCE METRICS:");
            $display(" -------------------");
            $display(" Avg DG/EC Latency: %0d cycles (Expected: %0d)", avg_dgec_latency, EXPECTED_DGEC_LATENCY);
            $display(" Avg CA3 Latency: %0d cycles (Expected: %0d)", avg_ca3_latency, EXPECTED_CA3_LATENCY);
            $display(" Avg CA1 Latency: %0d cycles (Expected: %0d)", avg_ca1_latency, EXPECTED_CA1_LATENCY);
            $display(" Avg Total Latency: %.2f us @ %0d MHz",
                     total_latency_us / real'(current_test_num), CLK_FREQ_MHZ);
            $display(" Estimated Throughput: %.0f predictions/second",
                     (real'(CLK_FREQ_MHZ) * 1000000.0) / real'(EXPECTED_TOTAL_LATENCY));
            $display("");
            $display(" DESIGN PARAMETERS:");
            $display(" -----------------");
            $display(" Hypervector Width: %0d bits", HV_WIDTH);
            $display(" Number of Contexts: %0d", NUM_CONTEXTS);
            $display(" Number of Events: %0d", NUM_EVENTS);
            $display(" Max Mismatch Threshold: %0d bits (%.1f%%)", MAX_MISMATCH,
                     (real'(MAX_MISMATCH)/real'(HV_WIDTH))*100.0);
            $display(" CA3 Max Recurrent Steps: %0d", MAX_STEPS);
            $display(" Voltage Width: %0d bits", VOLTAGE_WIDTH);
            $display(" Decay Step: %0d", DECAY_STEP);
            $display("");
            $display(" BIOLOGICAL PRINCIPLES VALIDATED:");
            $display(" --------------------------------");
            $display(" * Pattern Separation (DG sparse coding)");
            $display(" * Event-in-Context Coding (CA3 conjunctive neurons)");
            $display(" * Phase Precession (theta-compressed sequences)");
            $display(" * Context-Dependent Retrieval (EC -> CA3 gating)");
            $display(" * Prospective Coding (forward-looking predictions)");
            $display(" * Temporal Integration (CA1 non-simultaneous events)");
            $display("");
            if (total_tests_failed == 0) begin
                $display(" RESULT: ALL TESTS PASSED");
            end else begin
                $display(" RESULT: %0d TEST(S) FAILED", total_tests_failed);
            end
            $display("================================================================================");
        end
    endtask

    task init_all_mem_slots;
        logic [HV_WIDTH-1:0] random_hv;
        begin
            // give every slot a random HV first, so unused slots hold known values
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
            $display(" [Cycle %0d] Initialized %0d context and %0d event slots with sparse random HVs",
                     cycle_count, NUM_CONTEXTS, NUM_EVENTS);
        end
    endtask

    task reset_dut();
        begin
            $display("");
            $display("================================================================================");
            $display(" HIPPOCAMPEX SYSTEM RESET ");
            $display("================================================================================");
            $display(" Clearing all cortical inputs and internal states...");
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
            $display(" [Cycle %0d] Reset complete - hippocampus ready for encoding", cycle_count);
            $display("================================================================================");
        end
    endtask

    task train_context_hv(input int addr, input logic [HV_WIDTH-1:0] hv, string name);
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
            $display(" [Cycle %0d] EC: Stored Context '%s' at address %0d (HV[127:96] = 0x%08h)",
                     cycle_count, name, addr, hv[127:96]);
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
            $display(" [Cycle %0d] EC: Stored Event %0d (HV[127:96] = 0x%08h)",
                     cycle_count, addr, hv[127:96]);
        end
    endtask

    task train_crossbar_assoc(input int from_evt, to_evt);
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
            $display(" [Cycle %0d] CA3: Recurrent synapse Event[%0d] -> Event[%0d] (Hebbian)",
                     cycle_count, from_evt, to_evt);
        end
    endtask

    task train_crossbar_gate(input int evt, ctx);
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
            $display(" [Cycle %0d] CA3: Context gate Event[%0d] <-> Context[%0d]",
                     cycle_count, evt, ctx);
        end
    endtask

    task train_mcam(input int evt, ctx, t_code, string desc);
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
            $display(" [Cycle %0d] CA3: MCAM[Event=%0d][Context=%0d] = T%0d (%s)",
                     cycle_count, evt, ctx, t_code, desc);
        end
    endtask

    // One query through the whole pipeline. Stage times come from the control FSM state. The "ramp" wording in the prints predates the bitonic sorter.
    task run_inference(input logic [HV_WIDTH-1:0] ctx_hv, evt_hv, string test_name);
        int inference_start_cycle;
        begin
            $display("");
            $display("--------------------------------------------------------------------------------");
            $display(" INFERENCE PHASE: %s", test_name);
            $display(" Simulating sensory input -> hippocampal sequence prediction");
            $display("--------------------------------------------------------------------------------");
            i_train_mode = 0;
            @(posedge clk);
            inference_start_cycle = cycle_count;
            $display(" [Cycle %0d] Cortical Input Phase", cycle_count);
            $display(" Flow: Sensory -> Association Cortex -> Entorhinal Cortex (EC)");
            $display(" EC generates compressed hypervectors for DG/EC stage");
            wait(o_ready);
            $display(" [Cycle %0d] FSM ready - submitting query hypervectors", cycle_count);
            i_context_hv = ctx_hv;
            i_event_hv = evt_hv;
            i_valid = 1;
            @(posedge clk);
            i_valid = 0;
            dgec_start_cycle = cycle_count;
            $display("");
            $display(" [Cycle %0d] STAGE 1: DG/EC Pattern Recognition", cycle_count);
            $display(" Biological: Dentate Gyrus pattern separation + EC coding");
            $display(" Hardware: AP-CAM Hamming distance search + ramp-based sorting");
            wait(dut.u_control_fsm.state == dut.u_control_fsm.CA3_INFER);
            ca3_start_cycle = cycle_count;
            test_results[current_test_num-1].dgec_latency = cycle_count - dgec_start_cycle;
            $display("");
            $display(" [Cycle %0d] DG/EC Recognition Complete", cycle_count);
            $display(" Latency: %0d cycles", test_results[current_test_num-1].dgec_latency);
            $display(" Recognized Context: %0d (one-hot[3:0] = %4b)",
                     o_context_idx, o_context_onehot[3:0]);
            $display(" Recognized Event: %0d (one-hot[15:0] = %16b)",
                     o_event_idx, o_event_onehot[15:0]);
            $display("");
            $display(" [Cycle %0d] STAGE 2: CA3 Sequence Prediction", cycle_count);
            $display(" Biological: CA3 recurrent collaterals + phase precession");
            $display(" Hardware: Crossbar chain (%0d steps) + MCAM tagging + tracker decay", MAX_STEPS);
            wait(dut.u_control_fsm.state == dut.u_control_fsm.CA1_INFER);
            ca1_start_cycle = cycle_count;
            test_results[current_test_num-1].ca3_latency = cycle_count - ca3_start_cycle;
            $display("");
            $display(" [Cycle %0d] CA3 Prediction Complete", cycle_count);
            $display(" CA3 Latency: %0d cycles", test_results[current_test_num-1].ca3_latency);
            $display("");
            $display(" [Cycle %0d] STAGE 3: CA1 Temporal Integration", cycle_count);
            $display(" Biological: CA1 memory index + theta-sequence binding (non-simultaneous events)");
            $display(" Hardware: Ramp-comparator sorting + global counter timestamps");
            wait(o_valid);
            test_results[current_test_num-1].ca1_latency = cycle_count - ca1_start_cycle;
            $display("");
            $display(" [Cycle %0d] CA1 Integration Complete", cycle_count);
            $display(" CA1 Latency: %0d cycles", test_results[current_test_num-1].ca1_latency);
            $display(" Total Pipeline Latency: %0d cycles (%.2f us @ %0d MHz)",
                     cycle_count - dgec_start_cycle,
                     (cycle_count - dgec_start_cycle) * (1000.0 / real'(CLK_FREQ_MHZ)),
                     CLK_FREQ_MHZ);
            @(posedge clk);
        end
    endtask

    task print_predictions();
        int num_predictions = 0;
        begin
            $display("");
            $display("+------------------------------------------------------------------------------+");
            $display("| HIPPOCAMPAL OUTPUT - CA3 to CA1 Interface |");
            $display("+------------------------------------------------------------------------------+");
            $display("| Event | MCAM | Past | Merged | Timestamp | Biological_Interpretation |");
            $display("+-------+--------+--------+--------+-----------+----------------------------------------+");
            for (int i = 0; i < NUM_EVENTS; i++) begin
                if (o_event_voltages[i] != 0 || o_past_voltages[i] != 0 || o_timestamps[i] != 0) begin
                    string tag;
                    num_predictions++;
                    case (o_event_voltages[i])
                        VP: tag = "PRESENT (theta trough)";
                        VF1: tag = "FUTURE-1 (early phase)";
                        VF2: tag = "FUTURE-2 (earliest)";
                        default: tag = (o_past_voltages[i] > 0) ? "PAST (late phase)" : "---";
                    endcase
                    $display("| %5d | 0x%02h | 0x%02h | 0x%02h | 0x%02h | %-38s |",
                             i, o_event_voltages[i], o_past_voltages[i], o_merged_voltages[i], o_timestamps[i], tag);
                end
            end
            $display("+------------------------------------------------------------------------------+");
            $display(" Legend: VP=0x%02h (Present), VF1=0x%02h (Future+1), VF2=0x%02h (Future+2)",
                     VP, VF1, VF2);
            $display(" Total Predicted Events: %0d", num_predictions);
            $display(" CA1 Order: Future (high voltage/low timestamp) -> Present -> Past (low voltage/high timestamp)");
            $display("");
        end
    endtask

    task verify_prediction(input int evt, input [7:0] exp_voltage, input string desc);
        begin
            if (o_event_voltages[evt] === exp_voltage) begin
                $display(" [PASS] Event[%0d] voltage = 0x%02h | %s",
                         evt, o_event_voltages[evt], desc);
                test_results[current_test_num-1].passed++;
            end else begin
                $display(" [FAIL] Event[%0d] voltage = 0x%02h (expected 0x%02h) | %s",
                         evt, o_event_voltages[evt], exp_voltage, desc);
                test_results[current_test_num-1].failed++;
            end
        end
    endtask

    // evt_past is any event that should rank after the present one, not
    // necessarily an event that came before the cue
    task verify_order(input int evt_future, evt_present, evt_past, input string desc);
        begin
            if (o_timestamps[evt_future] < o_timestamps[evt_present] &&
                o_timestamps[evt_present] < o_timestamps[evt_past]) begin
                $display(" [PASS] CA1 Order: F[%0d]=0x%02h < P[%0d]=0x%02h < Past[%0d]=0x%02h | %s",
                         evt_future, o_timestamps[evt_future],
                         evt_present, o_timestamps[evt_present],
                         evt_past, o_timestamps[evt_past], desc);
                test_results[current_test_num-1].passed++;
            end else begin
                $display(" [FAIL] CA1 Order Mismatch: F[%0d]=0x%02h, P[%0d]=0x%02h, Past[%0d]=0x%02h | %s",
                         evt_future, o_timestamps[evt_future],
                         evt_present, o_timestamps[evt_present],
                         evt_past, o_timestamps[evt_past], desc);
                test_results[current_test_num-1].failed++;
            end
        end
    endtask

    initial begin
        $display("");
        $display("################################################################################");
        $display("# #");
        $display("# HIPPOCAMPEX: CONTEXT-AWARE SEQUENCE PREDICTION #");
        $display("# Neuromorphic Hardware Hardware Testbench #");
        $display("# #");
        $display("################################################################################");
        $display("");
        $display(" Pipeline: Cortex -> EC -> DG/EC -> CA3 -> CA1 -> PFC");
        $display(" Validating: Pattern separation + event-in-context + theta sequences");
        $display("");
        reset_dut();

        // training
        $display("");
        $display("================================================================================");
        $display(" TRAINING PHASE: Memory Encoding ");
        $display("================================================================================");
        $display(" Simulating offline memory consolidation (non-theta, slow integration)");
        $display("");
        $display("+--- STEP 1: Entorhinal Cortex - Store Context/Event Hypervectors -----------+");
        $display("| Biological: EC receives compressed representations from association cortex |");
        $display("+------------------------------------------------------------------------------+");
        train_context_hv(CTX_BLUE, gen_random_hv(), "Blue Room");
        train_context_hv(CTX_MAGENTA, gen_random_hv(), "Magenta Room");
        train_context_hv(CTX_GREEN, gen_random_hv(), "Green Room");
        train_context_hv(CTX_RED, gen_random_hv(), "Red Room");
        train_event_hv(EVT_7, gen_random_hv());
        train_event_hv(EVT_5, gen_random_hv());
        train_event_hv(EVT_3, gen_random_hv());
        train_event_hv(EVT_2, gen_random_hv());
        train_event_hv(EVT_1, gen_random_hv());
        train_event_hv(EVT_4, gen_random_hv());
        train_event_hv(EVT_6, gen_random_hv());
        train_event_hv(EVT_8, gen_random_hv());
        train_event_hv(EVT_9, gen_random_hv());
        $display("");
        $display("+--- STEP 2: CA3 Blue Context (Primes 7->5->3->2) ---------------------------+");
        $display("| Biological: Hebbian plasticity in CA3 recurrent collaterals |");
        $display("+------------------------------------------------------------------------------+");
        train_crossbar_assoc(EVT_7, EVT_5);
        train_crossbar_assoc(EVT_5, EVT_3);
        train_crossbar_assoc(EVT_3, EVT_2);
        $display("| Context-gating (EC modulation of CA3): |");
        train_crossbar_gate(EVT_7, CTX_BLUE);
        train_crossbar_gate(EVT_5, CTX_BLUE);
        train_crossbar_gate(EVT_3, CTX_BLUE);
        train_crossbar_gate(EVT_2, CTX_BLUE);
        $display("| Temporal codes (theta phase precession encoding): |");
        train_mcam(EVT_7, CTX_BLUE, 0, "first");
        train_mcam(EVT_5, CTX_BLUE, 1, "second");
        train_mcam(EVT_3, CTX_BLUE, 2, "third");
        train_mcam(EVT_2, CTX_BLUE, 3, "fourth");
        $display("");
        $display("+--- STEP 3: CA3 Magenta Context (Odds 7->5->3->1) --------------------------+");
        train_crossbar_assoc(EVT_7, EVT_5);
        train_crossbar_assoc(EVT_5, EVT_3);
        train_crossbar_assoc(EVT_3, EVT_1);
        train_crossbar_gate(EVT_7, CTX_MAGENTA);
        train_crossbar_gate(EVT_5, CTX_MAGENTA);
        train_crossbar_gate(EVT_3, CTX_MAGENTA);
        train_crossbar_gate(EVT_1, CTX_MAGENTA);
        train_mcam(EVT_7, CTX_MAGENTA, 0, "first");
        train_mcam(EVT_5, CTX_MAGENTA, 1, "second");
        train_mcam(EVT_3, CTX_MAGENTA, 2, "third");
        train_mcam(EVT_1, CTX_MAGENTA, 3, "fourth");
        $display("");
        $display("+--- STEP 4: CA3 Green Context (Evens 2->4->6->8) ---------------------------+");
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
        $display("");
        $display("+--- STEP 5: CA3 Red Context (Overlapping with Blue for Isolation Test) -----+");
        train_crossbar_assoc(EVT_7, EVT_9);
        train_crossbar_assoc(EVT_9, EVT_4);
        train_crossbar_gate(EVT_7, CTX_RED);
        train_crossbar_gate(EVT_9, CTX_RED);
        train_crossbar_gate(EVT_4, CTX_RED);
        train_mcam(EVT_7, CTX_RED, 0, "first");
        train_mcam(EVT_9, CTX_RED, 1, "second");
        train_mcam(EVT_4, CTX_RED, 2, "third");
        $display("");
        $display(" [Cycle %0d] Memory encoding complete - switching to inference mode", cycle_count);
        $display("================================================================================");
        repeat(5) @(posedge clk);

        // TEST 1: Blue, exact cue 7 at the start of the chain
        start_test("Blue Context - Sequence Start",
                   "Primes 7->5->3->2, Perfect HV match, Start position");
        run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7],
                     "Blue/Event7");
        print_predictions();
        $display("+--- VERIFICATION --------------------------------------------------------+");
        verify_prediction(EVT_7, VP, "Event 7 = PRESENT (theta trough)");
        verify_prediction(EVT_5, VF1, "Event 5 = FUTURE-1 (next in sequence)");
        verify_prediction(EVT_3, VF2, "Event 3 = FUTURE-2 (lookahead +2)");
        verify_prediction(EVT_2, 8'h00, "Event 2 = None (beyond +2 window)");
        verify_order(EVT_3, EVT_7, EVT_2, "CA1: Future < Present < Past timestamps");
        $display("+--------------------------------------------------------------------------+");
        $display(" Interpretation: CA3 predicts next 2 events via phase precession");
        repeat(10) @(posedge clk);
        end_test();

        // TEST 2: Green, cue 4 with 5 bits flipped
        start_test("Green Context - Noisy Input",
                   "Evens 2->4->6->8, 5-bit errors, Mid-sequence anchor");
        run_inference(stored_context_hvs[CTX_GREEN],
                     flip_bits(stored_event_hvs[EVT_4], 5),
                     "Green/Event4 (Noisy)");
        $display(" [INFO] Input HV: %0d bits flipped (%.1f%% error, within %.1f%% threshold)",
                 5, calc_bit_error_percent(5),
                 (real'(MAX_MISMATCH)/real'(HV_WIDTH))*100.0);
        print_predictions();
        $display("+--- VERIFICATION --------------------------------------------------------+");
        verify_prediction(EVT_4, VP, "Event 4 = PRESENT (noisy match successful)");
        verify_prediction(EVT_6, VF1, "Event 6 = FUTURE-1 (next in Green)");
        verify_prediction(EVT_8, VF2, "Event 8 = FUTURE-2 (sequence end)");
        verify_prediction(EVT_2, 8'h00, "Event 2 = None (in past)");
        verify_order(EVT_8, EVT_4, EVT_2, "CA1: Future < Present < Past timestamps");
        $display("+--------------------------------------------------------------------------+");
        $display(" Interpretation: Pattern completion despite noisy cortical input");
        repeat(10) @(posedge clk);
        end_test();

        // TEST 3: cue 7 in Blue, then in Magenta
        start_test("Context Switch - Same Event",
                   "Event 7 in Blue vs Magenta contexts");
        $display("");
        $display("--- Sub-test 3a: Blue Context ---");
        run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7],
                     "Blue/Event7");
        print_predictions();
        verify_prediction(EVT_7, VP, "Blue: Event 7 = PRESENT");
        verify_prediction(EVT_5, VF1, "Blue: Event 5 = FUTURE-1");
        verify_prediction(EVT_3, VF2, "Blue: Event 3 = FUTURE-2");
        verify_prediction(EVT_2, 8'h00, "Blue: Event 2 outside window");
        verify_order(EVT_3, EVT_7, EVT_2, "CA1: Future < Present < Past timestamps");
        repeat(10) @(posedge clk);
        $display("");
        $display("--- Sub-test 3b: Magenta Context (CONTEXT SWITCH) ---");
        run_inference(stored_context_hvs[CTX_MAGENTA], stored_event_hvs[EVT_7],
                     "Magenta/Event7");
        print_predictions();
        $display("+--- VERIFICATION --------------------------------------------------------+");
        verify_prediction(EVT_7, VP, "Magenta: Event 7 = PRESENT");
        verify_prediction(EVT_5, VF1, "Magenta: Event 5 = FUTURE-1 (shared)");
        verify_prediction(EVT_3, VF2, "Magenta: Event 3 = FUTURE-2 (shared)");
        verify_prediction(EVT_1, 8'h00, "Magenta: Event 1 outside window");
        verify_prediction(EVT_2, 8'h00, "Magenta: Event 2 NOT in sequence");
        verify_order(EVT_3, EVT_7, EVT_1, "CA1: Future < Present < Past timestamps");
        $display("+--------------------------------------------------------------------------+");
        $display(" Interpretation: Context-gated retrieval shows different endings (2 vs 1)");
        repeat(10) @(posedge clk);
        end_test();

        // TEST 4: move the cue along Blue (7, 5, 3)
        start_test("Progressive Sliding Window",
                   "Anchor-relative prediction through Blue sequence");
        $display("");
        $display("--- Sub-test 4a: Event 7 (Sequence Start) ---");
        run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7],
                     "Blue/Event7");
        $display(" Prediction window: [7:present, 5:+1, 3:+2]");
        print_predictions();
        verify_prediction(EVT_7, VP, "Event 7 = PRESENT");
        verify_prediction(EVT_5, VF1, "Event 5 = FUTURE-1");
        verify_prediction(EVT_3, VF2, "Event 3 = FUTURE-2");

        verify_order(EVT_3, EVT_7, EVT_2, "CA1: Future < Present < Past timestamps");
        repeat(10) @(posedge clk);
        $display("");
        $display("--- Sub-test 4b: Event 5 (Window Slides Forward) ---");
        run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_5],
                     "Blue/Event5");
        $display(" Prediction window: [5:present, 3:+1, 2:+2]");
        print_predictions();
        verify_prediction(EVT_5, VP, "Event 5 = PRESENT (anchor shifted)");
        verify_prediction(EVT_3, VF1, "Event 3 = FUTURE-1 (was F2, now F1)");
        verify_prediction(EVT_2, VF2, "Event 2 = FUTURE-2 (was hidden)");
        verify_prediction(EVT_7, 8'h00, "Event 7 = None (now in past)");
        verify_order(EVT_2, EVT_5, EVT_7, "CA1: Future < Present < Past timestamps");
        repeat(10) @(posedge clk);
        $display("");
        $display("--- Sub-test 4c: Event 3 (Near Sequence End) ---");
        run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_3],
                     "Blue/Event3");
        $display(" Prediction window: [3:present, 2:+1]");
        print_predictions();
        verify_prediction(EVT_3, VP, "Event 3 = PRESENT");
        verify_prediction(EVT_2, VF1, "Event 2 = FUTURE-1 (last event)");
        verify_prediction(EVT_5, 8'h00, "Event 5 = None (in past)");
        verify_order(EVT_2, EVT_3, EVT_5, "CA1: Future < Present < Past timestamps");
        $display("");
        $display("+--- VERIFICATION --------------------------------------------------------+");
        $display(" [PASS] Sliding window correctly tracks sequence position");
        $display(" [PASS] Anchor-relative alignment enables flexible queries");
        $display(" [PASS] Past events suppressed (forward predictions only)");
        $display("+--------------------------------------------------------------------------+");
        $display(" Interpretation: MCAM uses relative time, not absolute time");
        repeat(10) @(posedge clk);
        end_test();

        // TEST 5: extend Blue with 2->9->1; only two futures still get tags
        start_test("Longer Sequence",
                   "Primes 7->5->3->2->9->1, Test extended prediction window");
        train_crossbar_assoc(EVT_2, EVT_9);
        train_crossbar_assoc(EVT_9, EVT_1);
        train_crossbar_gate(EVT_9, CTX_BLUE);
        train_crossbar_gate(EVT_1, CTX_BLUE);
        train_mcam(EVT_9, CTX_BLUE, 4, "fifth");
        train_mcam(EVT_1, CTX_BLUE, 5, "sixth");
        run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7],
                     "Blue/Event7 (Longer)");
        print_predictions();
        $display("+--- VERIFICATION --------------------------------------------------------+");
        verify_prediction(EVT_7, VP, "Event 7 = PRESENT");
        verify_prediction(EVT_5, VF1, "Event 5 = FUTURE-1");
        verify_prediction(EVT_3, VF2, "Event 3 = FUTURE-2");
        verify_prediction(EVT_2, 8'h00, "Event 2 = None (beyond +2)");
        verify_prediction(EVT_9, 8'h00, "Event 9 = None (beyond +2)");
        verify_prediction(EVT_1, 8'h00, "Event 1 = None (beyond +2)");
        verify_order(EVT_3, EVT_7, EVT_2, "CA1: Future < Present < Past timestamps");
        $display("+--------------------------------------------------------------------------+");
        $display(" Interpretation: Handles longer sequences, predicts only +2 futures");
        repeat(10) @(posedge clk);
        end_test();

        // TEST 6: meant to break Green at 4->6, but nothing is removed: 4->8 is
        // added next to 4->6 and 8 keeps code 3, so the expected tags match plain Green.
        start_test("Partial Activation",
                   "Green with missing assoc (2->4->8, skip 6), Test broken chain");
        train_crossbar_assoc(EVT_4, EVT_8);
        train_mcam(EVT_8, CTX_GREEN, 3, "fourth (skipped)");
        run_inference(stored_context_hvs[CTX_GREEN], stored_event_hvs[EVT_2],
                     "Green/Event2 (Partial)");
        print_predictions();
        $display("+--- VERIFICATION --------------------------------------------------------+");
        verify_prediction(EVT_2, VP, "Event 2 = PRESENT");
        verify_prediction(EVT_4, VF1, "Event 4 = FUTURE-1");
        verify_prediction(EVT_6, VF2, "Event 6 = FUTURE-2 (4->6 link still active)");
        verify_prediction(EVT_8, 8'h00, "Event 8 = None (4->8 is 3rd hop, beyond +2)");
        // 8 is outside the window (unranked), so it stands in for past
        verify_order(EVT_6, EVT_2, EVT_8, "CA1: Future < Present < Past timestamps");
        $display("+--------------------------------------------------------------------------+");
        $display(" Interpretation: Graceful degradation on partial memories (PDF page 1: non-simultaneous events)");
        repeat(10) @(posedge clk);
        end_test();

        // TEST 7: cue 7 in Blue (7->5->3) and in Red (7->9->4)
        start_test("Context Isolation",
                   "Event 7 in Blue (7->5->3) vs Red (7->9->4), Overlap test");
        $display("");
        $display("--- Sub-test 7a: Blue Context ---");
        run_inference(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7],
                     "Blue/Event7");
        print_predictions();
        verify_prediction(EVT_7, VP, "Blue: Event 7 = PRESENT");
        verify_prediction(EVT_5, VF1, "Blue: Event 5 = FUTURE-1");
        verify_prediction(EVT_3, VF2, "Blue: Event 3 = FUTURE-2");
        verify_order(EVT_3, EVT_7, EVT_2, "CA1: Future < Present < Past timestamps");
        repeat(10) @(posedge clk);
        $display("");
        $display("--- Sub-test 7b: Red Context ---");
        run_inference(stored_context_hvs[CTX_RED], stored_event_hvs[EVT_7],
                     "Red/Event7");
        print_predictions();
        $display("+--- VERIFICATION --------------------------------------------------------+");
        verify_prediction(EVT_7, VP, "Red: Event 7 = PRESENT");
        verify_prediction(EVT_9, VF1, "Red: Event 9 = FUTURE-1");
        verify_prediction(EVT_4, VF2, "Red: Event 4 = FUTURE-2");
        verify_prediction(EVT_5, 8'h00, "Red: Event 5 = None (Blue-specific)");
        verify_order(EVT_4, EVT_7, EVT_5, "CA1: Future < Present < Past timestamps");
        $display("+--------------------------------------------------------------------------+");
        $display(" Interpretation: Same event, isolated contexts (PDF page 1: context-dependent decision-making)");
        repeat(10) @(posedge clk);
        end_test();

        print_test_summary();
        $finish;
    end

    // watchdog
    initial begin
        #(CLK_PERIOD * 50000);
        $display("");
        $display("[ERROR] Testbench timeout after %0d cycles", 50000);
        $display(" This may indicate a deadlock or infinite loop in the design");
        $finish;
    end

endmodule
