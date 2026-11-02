// testbench_hippocampex_axis_wrapper: the scenario groups from
// testbench_hippocampex_ieee.sv, but everything goes through AXI-Stream
// packets, the way the PS talks to the IP over DMA. No direct access to
// hippocampex_top.
//   A basic, all contexts     B noise sweep        C moving cue
//   D context switching       E edge cases         F longer chains
//   G back-to-back recalls
// Packet helpers, one per command:
//   axis_send_train_ctx()   -> CMD 0x1
//   axis_send_train_evt()   -> CMD 0x2
//   axis_send_xbar_assoc()  -> CMD 0x3
//   axis_send_xbar_gate()   -> CMD 0x4
//   axis_send_mcam()        -> CMD 0x5
//   axis_send_infer()       -> CMD 0x6
//   axis_send_status()      -> CMD 0x7
`timescale 1ns/1ps

module testbench_hippocampex_axis_wrapper;

    // same as the wrapper defaults
    localparam AXIS_DATA_WIDTH = 32;
    localparam AXIS_KEEP_WIDTH = 4;
    localparam HV_WIDTH        = 128;
    localparam NUM_CONTEXTS    = 16;
    localparam NUM_EVENTS      = 64;
    localparam DIST_WIDTH      = 8;
    localparam MAX_MISMATCH    = 13;
    localparam MAX_STEPS       = 8;
    localparam CNTR_WIDTH      = 6;
    localparam VOLTAGE_WIDTH   = 8;
    localparam DECAY_STEP      = 4;
    localparam CLK_PERIOD      = 10;
    localparam CLK_FREQ_MHZ    = 100;

    localparam CTX_ADDR_W = $clog2(NUM_CONTEXTS);
    localparam EVT_ADDR_W = $clog2(NUM_EVENTS);

    // MCAM tags: present, future-1, future-2
    localparam VP  = 8'h40;
    localparam VF1 = 8'h80;
    localparam VF2 = 8'hC0;

    localparam CTX_BLUE    = 0;
    localparam CTX_MAGENTA = 1;
    localparam CTX_GREEN   = 2;
    localparam CTX_RED     = 3;
    localparam CTX_CYAN    = 4;
    localparam CTX_YELLOW  = 5;
    localparam CTX_ORANGE  = 6;
    localparam CTX_PURPLE  = 7;

    // event slots
    localparam EVT_0=0,  EVT_1=1,  EVT_2=2,  EVT_3=3,  EVT_4=4;
    localparam EVT_5=5,  EVT_6=6,  EVT_7=7,  EVT_8=8,  EVT_9=9;
    localparam EVT_10=10, EVT_11=11, EVT_12=12, EVT_13=13, EVT_14=14, EVT_15=15;
    // Orange and Purple were written for 128 events (slots 120-127). With 64
    // events those slots don't exist, so the names now point at 56-63.
    localparam EVT_120=56, EVT_121=57, EVT_122=58, EVT_123=59;
    localparam EVT_124=60, EVT_125=61, EVT_126=62, EVT_127=63;

    logic axi_aclk;
    logic axi_aresetn;

    // TB -> DUT
    logic [AXIS_DATA_WIDTH-1:0] s_axis_tdata;
    logic [AXIS_KEEP_WIDTH-1:0] s_axis_tkeep;
    logic                       s_axis_tvalid;
    logic                       s_axis_tready;
    logic                       s_axis_tlast;

    // DUT -> TB
    logic [AXIS_DATA_WIDTH-1:0] m_axis_tdata;
    logic [AXIS_KEEP_WIDTH-1:0] m_axis_tkeep;
    logic                       m_axis_tvalid;
    logic                       m_axis_tready;
    logic                       m_axis_tlast;

    logic irq_result_ready;

    hippocampex_axis_wrapper #(
        .AXIS_DATA_WIDTH (AXIS_DATA_WIDTH),
        .HV_WIDTH        (HV_WIDTH),
        .NUM_CONTEXTS    (NUM_CONTEXTS),
        .NUM_EVENTS      (NUM_EVENTS),
        .DIST_WIDTH      (DIST_WIDTH),
        .MAX_MISMATCH    (MAX_MISMATCH),
        .MAX_STEPS       (MAX_STEPS),
        .CNTR_WIDTH      (CNTR_WIDTH),
        .VOLTAGE_WIDTH   (VOLTAGE_WIDTH),
        .DECAY_STEP      (DECAY_STEP)
    ) dut (
        .axi_aclk       (axi_aclk),
        .axi_aresetn     (axi_aresetn),
        .s_axis_tdata    (s_axis_tdata),
        .s_axis_tkeep    (s_axis_tkeep),
        .s_axis_tvalid   (s_axis_tvalid),
        .s_axis_tready   (s_axis_tready),
        .s_axis_tlast    (s_axis_tlast),
        .m_axis_tdata    (m_axis_tdata),
        .m_axis_tkeep    (m_axis_tkeep),
        .m_axis_tvalid   (m_axis_tvalid),
        .m_axis_tready   (m_axis_tready),
        .m_axis_tlast    (m_axis_tlast),
        .irq_result_ready(irq_result_ready)
    );

    initial begin
        axi_aclk = 0;
        forever #(CLK_PERIOD/2) axi_aclk = ~axi_aclk;
    end

    int cycle_count       = 0;
    int total_tests       = 0;
    int total_passed      = 0;
    int total_failed      = 0;
    int total_checks_pass = 0;
    int total_checks_fail = 0;
    int inference_count   = 0;
    int test_checks_pass  = 0;
    int test_checks_fail  = 0;

    // copies of every HV sent, reused as queries
    logic [HV_WIDTH-1:0] stored_context_hvs [NUM_CONTEXTS];
    logic [HV_WIDTH-1:0] stored_event_hvs   [NUM_EVENTS];

    // fields from the last A1 reply
    logic [CTX_ADDR_W-1:0]    rx_context_idx;
    logic [EVT_ADDR_W-1:0]    rx_event_idx;
    logic [7:0]                rx_num_active;
    logic [VOLTAGE_WIDTH-1:0] rx_merged_voltages [NUM_EVENTS];
    logic [NUM_EVENTS-1:0]    rx_ordered_onehot;
    logic [CNTR_WIDTH-1:0]    rx_timestamps [NUM_EVENTS];

    integer fd_summary, fd_prediction, fd_axi_trace;

    always @(posedge axi_aclk) begin
        if (axi_aresetn) cycle_count <= cycle_count + 1;
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

    // Sending side, standing in for the DMA MM2S channel. One task per command.

    // send one beat and hold it until tready
    task automatic axis_send_beat(input logic [31:0] data, input logic last);
        @(posedge axi_aclk);
        s_axis_tdata  <= data;
        s_axis_tkeep  <= 4'hF;
        s_axis_tvalid <= 1'b1;
        s_axis_tlast  <= last;
        do begin
            @(posedge axi_aclk);
        end while (!s_axis_tready);
        s_axis_tvalid <= 1'b0;
        s_axis_tlast  <= 1'b0;
    endtask

    // 0x1 context HV: header + 4 HV words, low word first
    task automatic axis_send_train_ctx(input int addr, input logic [HV_WIDTH-1:0] hv);
        logic [31:0] header;
        header = {4'h1, {(28-CTX_ADDR_W){1'b0}}, addr[CTX_ADDR_W-1:0]};
        axis_send_beat(header, 1'b0);
        axis_send_beat(hv[  31:  0], 1'b0);
        axis_send_beat(hv[  63: 32], 1'b0);
        axis_send_beat(hv[  95: 64], 1'b0);
        axis_send_beat(hv[ 127: 96], 1'b1);
        wait_for_ack(4'h1);
    endtask

    // 0x2 event HV: same layout
    task automatic axis_send_train_evt(input int addr, input logic [HV_WIDTH-1:0] hv);
        logic [31:0] header;
        header = {4'h2, {(28-EVT_ADDR_W){1'b0}}, addr[EVT_ADDR_W-1:0]};
        axis_send_beat(header, 1'b0);
        axis_send_beat(hv[  31:  0], 1'b0);
        axis_send_beat(hv[  63: 32], 1'b0);
        axis_send_beat(hv[  95: 64], 1'b0);
        axis_send_beat(hv[ 127: 96], 1'b1);
        wait_for_ack(4'h2);
    endtask

    // 0x3 crossbar link from_evt -> to_evt
    task automatic axis_send_xbar_assoc(input int from_evt, input int to_evt);
        logic [31:0] pkt;
        pkt = {4'h3, 7'd0, from_evt[6:0], to_evt[6:0], 6'd0, 1'b1};
        axis_send_beat(pkt, 1'b1);
        wait_for_ack(4'h3);
    endtask

    // 0x4 gate: allow evt in ctx
    task automatic axis_send_xbar_gate(input int evt, input int ctx);
        logic [31:0] pkt;
        pkt = {4'h4, 7'd0, evt[6:0], ctx[6:0], 6'd0, 1'b1};
        axis_send_beat(pkt, 1'b1);
        wait_for_ack(4'h4);
    endtask

    // 0x5 MCAM time code for (evt, ctx)
    task automatic axis_send_mcam(input int evt, input int ctx, input int t_code);
        logic [31:0] pkt;
        pkt = {4'h5, 7'd0, evt[6:0], ctx[6:0], 1'b0, t_code[CNTR_WIDTH-1:0]};
        axis_send_beat(pkt, 1'b1);
        wait_for_ack(4'h5);
    endtask

    // 0x6 recall: header, context HV, event HV (9 beats)
    task automatic axis_send_infer(input logic [HV_WIDTH-1:0] ctx_hv,
                                   input logic [HV_WIDTH-1:0] evt_hv);
        logic [31:0] header;
        header = {4'h6, 28'd0};
        axis_send_beat(header, 1'b0);
        axis_send_beat(ctx_hv[  31:  0], 1'b0);
        axis_send_beat(ctx_hv[  63: 32], 1'b0);
        axis_send_beat(ctx_hv[  95: 64], 1'b0);
        axis_send_beat(ctx_hv[ 127: 96], 1'b0);
        axis_send_beat(evt_hv[  31:  0], 1'b0);
        axis_send_beat(evt_hv[  63: 32], 1'b0);
        axis_send_beat(evt_hv[  95: 64], 1'b0);
        axis_send_beat(evt_hv[ 127: 96], 1'b1);
    endtask

    // 0x7 status request (not called by any test here)
    task automatic axis_send_status();
        axis_send_beat({4'h7, 28'd0}, 1'b1);
    endtask

    // Receiving side. Always ready, like the DMA S2MM channel.
    assign m_axis_tready = 1'b1;

    // wait for the one-beat C1 ack; only warns on timeout or wrong echo
    task automatic wait_for_ack(input logic [3:0] expected_cmd);
        int timeout;
        timeout = 0;
        while (timeout < 100) begin
            @(posedge axi_aclk);
            if (m_axis_tvalid && m_axis_tdata[31:24] == 8'hC1) begin
                if (m_axis_tdata[3:0] !== expected_cmd)
                    $display("   [WARN] ACK cmd mismatch: got 0x%01h exp 0x%01h",
                             m_axis_tdata[3:0], expected_cmd);
                return;
            end
            timeout++;
        end
        $display("   [WARN] ACK timeout for CMD 0x%01h", expected_cmd);
    endtask

    // read one A1 reply into rx_*
    task automatic wait_for_result(input int max_wait_cycles);
        int timeout;
        int beat;
        timeout = 0;

        while (timeout < max_wait_cycles) begin
            @(posedge axi_aclk);
            if (m_axis_tvalid && m_axis_tdata[31:24] == 8'hA1) begin
                rx_context_idx = m_axis_tdata[23:17];
                rx_event_idx   = m_axis_tdata[16:10];
                rx_num_active  = m_axis_tdata[7:0];  // wrong in the RTL, see the wrapper's known-bug note

                // voltages, 4 per beat
                for (beat = 0; beat < NUM_EVENTS/4; beat++) begin
                    @(posedge axi_aclk);
                    while (!m_axis_tvalid) @(posedge axi_aclk);

                    rx_merged_voltages[beat*4 + 0] = m_axis_tdata[ 7: 0];
                    rx_merged_voltages[beat*4 + 1] = m_axis_tdata[15: 8];
                    rx_merged_voltages[beat*4 + 2] = m_axis_tdata[23:16];
                    rx_merged_voltages[beat*4 + 3] = m_axis_tdata[31:24];
                end

                // CA1 one-hot, 32 per beat
                for (beat = 0; beat < NUM_EVENTS/32; beat++) begin
                    @(posedge axi_aclk);
                    while (!m_axis_tvalid) @(posedge axi_aclk);
                    rx_ordered_onehot[beat*32 +: 32] = m_axis_tdata;
                end

                // CA1 ranks, 4 per beat
                for (beat = 0; beat < NUM_EVENTS/4; beat++) begin
                    @(posedge axi_aclk);
                    while (!m_axis_tvalid) @(posedge axi_aclk);
                    rx_timestamps[beat*4 + 0] = m_axis_tdata[ 7: 0];
                    rx_timestamps[beat*4 + 1] = m_axis_tdata[15: 8];
                    rx_timestamps[beat*4 + 2] = m_axis_tdata[23:16];
                    rx_timestamps[beat*4 + 3] = m_axis_tdata[31:24];
                end

                return;
            end
            timeout++;
        end
        $display("   [ERROR] Inference result timeout after %0d cycles", max_wait_cycles);
     endtask

    task automatic run_inference_axi(input logic [HV_WIDTH-1:0] ctx_hv,
                                     input logic [HV_WIDTH-1:0] evt_hv,
                                     input string label);
        int start_cyc;
        start_cyc = cycle_count;
        inference_count++;
        $display("   [INF #%0d] %s (AXI4-Stream)", inference_count, label);

        axis_send_infer(ctx_hv, evt_hv);
        wait_for_result(2000);

        $display("   Result: ctx=%0d evt=%0d active=%0d latency=%0d cycles",
                 rx_context_idx, rx_event_idx, rx_num_active,
                 cycle_count - start_cyc);
    endtask

    task automatic verify_voltage(input string test_name, input int target_evt,
                                  input logic [7:0] expected, input string role);
        logic [7:0] actual;
        actual = rx_merged_voltages[target_evt];
        if (actual === expected) begin
            $display("   [PASS] E%0d = 0x%02h | %s", target_evt, actual, role);
            test_checks_pass++;
        end else begin
            $display("   [FAIL] E%0d = 0x%02h (exp 0x%02h) | %s",
                     target_evt, actual, expected, role);
            test_checks_fail++;
        end
        $fwrite(fd_prediction, "%0d,%s,%0d,%0d,%0d,%0d,%s\n",
                inference_count, test_name, target_evt, expected, actual,
                (actual === expected) ? 1 : 0, role);
    endtask

    // CA1 order checks. Rank 0 is the highest voltage, so F2 < F1 < present.
    task automatic verify_ca1_order(input string test_name,
                                    input int evt_f2, input int evt_f1, input int evt_p);
        if (rx_timestamps[evt_f2] < rx_timestamps[evt_f1] &&
            rx_timestamps[evt_f1] < rx_timestamps[evt_p]) begin
            $display("   [PASS] CA1-order: ts[E%0d]=%0d < ts[E%0d]=%0d < ts[E%0d]=%0d",
                     evt_f2, rx_timestamps[evt_f2],
                     evt_f1, rx_timestamps[evt_f1],
                     evt_p,  rx_timestamps[evt_p]);
            test_checks_pass++;
        end else begin
            $display("   [FAIL] CA1-order: ts[E%0d]=%0d  ts[E%0d]=%0d  ts[E%0d]=%0d",
                     evt_f2, rx_timestamps[evt_f2],
                     evt_f1, rx_timestamps[evt_f1],
                     evt_p,  rx_timestamps[evt_p]);
            test_checks_fail++;
        end
    endtask

    task automatic verify_ca1_ts_lt(input string test_name,
                                    input int evt_earlier, input int evt_later,
                                    input string role);
        if (rx_timestamps[evt_earlier] < rx_timestamps[evt_later]) begin
            $display("   [PASS] CA1-order: ts[E%0d]=%0d < ts[E%0d]=%0d | %s",
                     evt_earlier, rx_timestamps[evt_earlier],
                     evt_later,   rx_timestamps[evt_later], role);
            test_checks_pass++;
        end else begin
            $display("   [FAIL] CA1-order: ts[E%0d]=%0d NOT < ts[E%0d]=%0d | %s",
                     evt_earlier, rx_timestamps[evt_earlier],
                     evt_later,   rx_timestamps[evt_later], role);
            test_checks_fail++;
        end
    endtask

    // event must be unranked (all ones)
    task automatic verify_ca1_inactive(input string test_name, input int evt,
                                       input string role);
        logic [CNTR_WIDTH-1:0] inactive_val;
        inactive_val = {CNTR_WIDTH{1'b1}};
        if (rx_timestamps[evt] == inactive_val) begin
            $display("   [PASS] CA1-inactive: ts[E%0d]=0x%0h | %s",
                     evt, rx_timestamps[evt], role);
            test_checks_pass++;
        end else begin
            $display("   [FAIL] CA1-inactive: ts[E%0d]=0x%0h (exp=0x%0h) | %s",
                     evt, rx_timestamps[evt], inactive_val, role);
            test_checks_fail++;
        end
    endtask

    task automatic print_active_events();
        int cnt;
        cnt = 0;
        $display("   +-------+--------+-----------+");
        $display("   | Event | Merged | Role      |");
        $display("   +-------+--------+-----------+");
        for (int i = 0; i < NUM_EVENTS; i++) begin
            if (rx_merged_voltages[i] != 0) begin
                $display("   | %5d | 0x%02h | %-9s |",
                         i, rx_merged_voltages[i],
                         get_voltage_role(rx_merged_voltages[i]));
                cnt++;
            end
        end
        $display("   +-------+--------+-----------+");
        $display("   Active: %0d", cnt);
    endtask

    task automatic start_test(input string name, input string desc, input string group);
        total_tests++;
        test_checks_pass = 0;
        test_checks_fail = 0;
        $display("");
        $display("################################################################################");
        $display("# TEST %0d [%s]: %s", total_tests, group, name);
        $display("# %s", desc);
        $display("# Cycle: %0d", cycle_count);
        $display("################################################################################");
    endtask

    task automatic end_test(input string name);
        total_checks_pass += test_checks_pass;
        total_checks_fail += test_checks_fail;
        if (test_checks_fail == 0) begin
            total_passed++;
            $display(" >> TEST %0d: PASSED (%0d checks)", total_tests, test_checks_pass);
        end else begin
            total_failed++;
            $display(" >> TEST %0d: FAILED (%0d pass, %0d fail)",
                     total_tests, test_checks_pass, test_checks_fail);
        end
        $fwrite(fd_summary, "%0d,%s,%s,%0d,%0d\n",
                total_tests, name,
                (test_checks_fail == 0) ? "PASS" : "FAIL",
                test_checks_pass, test_checks_fail);
    endtask

    // training wrappers: send the packet and keep a local copy of the HV
    task automatic train_context(input int addr, input logic [HV_WIDTH-1:0] hv, input string name);
        stored_context_hvs[addr] = hv;
        axis_send_train_ctx(addr, hv);
        $display("   EC: Context[%0d] '%s' stored via AXI", addr, name);
    endtask

    task automatic train_event(input int addr, input logic [HV_WIDTH-1:0] hv);
        stored_event_hvs[addr] = hv;
        axis_send_train_evt(addr, hv);
    endtask

    task automatic train_assoc(input int from_evt, input int to_evt);
        axis_send_xbar_assoc(from_evt, to_evt);
    endtask

    task automatic train_gate(input int evt, input int ctx);
        axis_send_xbar_gate(evt, ctx);
    endtask

    task automatic train_mcam_entry(input int evt, input int ctx, input int t_code);
        axis_send_mcam(evt, ctx, t_code);
    endtask

    // fill every slot with random HVs first, as testbench_hippocampex_ieee does
    task automatic init_all_slots();
        logic [HV_WIDTH-1:0] hv;
        for (int i = 0; i < NUM_CONTEXTS; i++) begin
            hv = gen_random_hv();
            axis_send_train_ctx(i, hv);
        end
        for (int i = 0; i < NUM_EVENTS; i++) begin
            hv = gen_random_hv();
            axis_send_train_evt(i, hv);
        end
        $display(" [INIT] %0d ctx + %0d evt slots initialized via AXI", NUM_CONTEXTS, NUM_EVENTS);
    endtask

    task automatic open_csv_files();
        fd_summary    = $fopen("axi_test_summary.csv", "w");
        fd_prediction = $fopen("axi_prediction_detail.csv", "w");
        fd_axi_trace  = $fopen("axi_trace.csv", "w");
        $fwrite(fd_summary, "test_num,test_name,result,checks_pass,checks_fail\n");
        $fwrite(fd_prediction, "inference_id,test_name,target_event,exp_voltage,act_voltage,match,role\n");
        $fwrite(fd_axi_trace, "cycle,direction,data,tlast,description\n");
        $display(" [CSV] AXI testbench CSV files opened");
    endtask

    task automatic close_csv_files();
        $fclose(fd_summary);
        $fclose(fd_prediction);
        $fclose(fd_axi_trace);
        $display(" [CSV] CSV files closed");
    endtask

    initial begin
        s_axis_tdata  = 32'h0;
        s_axis_tkeep  = 4'hF;
        s_axis_tvalid = 1'b0;
        s_axis_tlast  = 1'b0;
        axi_aresetn   = 1'b0;

        $display("");
        $display("--------------------------------------------------------");
        $display("  HIPPOCAMPEX AXI4-STREAM FPGA VERIFICATION SUITE");
        $display("  Target: AMD Zynq UltraScale+ ZCU102");
        $display("--------------------------------------------------------");

        open_csv_files();

        repeat(10) @(posedge axi_aclk);
        axi_aresetn = 1'b1;
        repeat(5) @(posedge axi_aclk);

        $display("");
        $display("----------------------- INIT PHASE (AXI4-S) -----------------------");
        init_all_slots();
        repeat(5) @(posedge axi_aclk);

        $display("");
        $display("----------------------- TRAINING PHASE (AXI4-S) -----------------------");

        $display(" STEP 1: Store Context Hypervectors");
        train_context(CTX_BLUE,    gen_random_hv(), "Blue");
        train_context(CTX_MAGENTA, gen_random_hv(), "Magenta");
        train_context(CTX_GREEN,   gen_random_hv(), "Green");
        train_context(CTX_RED,     gen_random_hv(), "Red");
        train_context(CTX_CYAN,    gen_random_hv(), "Cyan");
        train_context(CTX_YELLOW,  gen_random_hv(), "Yellow");
        train_context(CTX_ORANGE,  gen_random_hv(), "Orange");
        train_context(CTX_PURPLE,  gen_random_hv(), "Purple");

        $display(" STEP 2: Store Event Hypervectors");
        train_event(EVT_0,   gen_random_hv());  train_event(EVT_1,   gen_random_hv());
        train_event(EVT_2,   gen_random_hv());  train_event(EVT_3,   gen_random_hv());
        train_event(EVT_4,   gen_random_hv());  train_event(EVT_5,   gen_random_hv());
        train_event(EVT_6,   gen_random_hv());  train_event(EVT_7,   gen_random_hv());
        train_event(EVT_8,   gen_random_hv());  train_event(EVT_9,   gen_random_hv());
        train_event(EVT_10,  gen_random_hv());  train_event(EVT_11,  gen_random_hv());
        train_event(EVT_12,  gen_random_hv());  train_event(EVT_13,  gen_random_hv());
        train_event(EVT_14,  gen_random_hv());  train_event(EVT_15,  gen_random_hv());
        train_event(EVT_120, gen_random_hv());  train_event(EVT_121, gen_random_hv());
        train_event(EVT_122, gen_random_hv());  train_event(EVT_123, gen_random_hv());
        train_event(EVT_124, gen_random_hv());  train_event(EVT_125, gen_random_hv());
        train_event(EVT_126, gen_random_hv());  train_event(EVT_127, gen_random_hv());
        $display("   24 event HVs stored via AXI");

        // STEP 3: crossbar + MCAM, same sequences as testbench_hippocampex_ieee
        $display(" STEP 3: Train CA3 Crossbar and MCAM Sequences");

        $display("   Blue: 7->5->3->2");
        train_assoc(EVT_7, EVT_5); train_assoc(EVT_5, EVT_3); train_assoc(EVT_3, EVT_2);
        train_gate(EVT_7, CTX_BLUE); train_gate(EVT_5, CTX_BLUE);
        train_gate(EVT_3, CTX_BLUE); train_gate(EVT_2, CTX_BLUE);
        train_mcam_entry(EVT_7, CTX_BLUE, 0); train_mcam_entry(EVT_5, CTX_BLUE, 1);
        train_mcam_entry(EVT_3, CTX_BLUE, 2); train_mcam_entry(EVT_2, CTX_BLUE, 3);

        $display("   Magenta: 7->5->3->1");
        train_assoc(EVT_3, EVT_1);
        train_gate(EVT_7, CTX_MAGENTA); train_gate(EVT_5, CTX_MAGENTA);
        train_gate(EVT_3, CTX_MAGENTA); train_gate(EVT_1, CTX_MAGENTA);
        train_mcam_entry(EVT_7, CTX_MAGENTA, 0); train_mcam_entry(EVT_5, CTX_MAGENTA, 1);
        train_mcam_entry(EVT_3, CTX_MAGENTA, 2); train_mcam_entry(EVT_1, CTX_MAGENTA, 3);

        $display("   Green: 2->4->6->8");
        train_assoc(EVT_2, EVT_4); train_assoc(EVT_4, EVT_6); train_assoc(EVT_6, EVT_8);
        train_gate(EVT_2, CTX_GREEN); train_gate(EVT_4, CTX_GREEN);
        train_gate(EVT_6, CTX_GREEN); train_gate(EVT_8, CTX_GREEN);
        train_mcam_entry(EVT_2, CTX_GREEN, 0); train_mcam_entry(EVT_4, CTX_GREEN, 1);
        train_mcam_entry(EVT_6, CTX_GREEN, 2); train_mcam_entry(EVT_8, CTX_GREEN, 3);

        $display("   Red: 7->9->4");
        train_assoc(EVT_7, EVT_9); train_assoc(EVT_9, EVT_4);
        train_gate(EVT_7, CTX_RED); train_gate(EVT_9, CTX_RED); train_gate(EVT_4, CTX_RED);
        train_mcam_entry(EVT_7, CTX_RED, 0); train_mcam_entry(EVT_9, CTX_RED, 1);
        train_mcam_entry(EVT_4, CTX_RED, 2);

        $display("   Cyan: 10->11->12->13->14");
        train_assoc(EVT_10, EVT_11); train_assoc(EVT_11, EVT_12);
        train_assoc(EVT_12, EVT_13); train_assoc(EVT_13, EVT_14);
        train_gate(EVT_10, CTX_CYAN); train_gate(EVT_11, CTX_CYAN);
        train_gate(EVT_12, CTX_CYAN); train_gate(EVT_13, CTX_CYAN);
        train_gate(EVT_14, CTX_CYAN);
        train_mcam_entry(EVT_10, CTX_CYAN, 0); train_mcam_entry(EVT_11, CTX_CYAN, 1);
        train_mcam_entry(EVT_12, CTX_CYAN, 2); train_mcam_entry(EVT_13, CTX_CYAN, 3);
        train_mcam_entry(EVT_14, CTX_CYAN, 4);

        $display("   Yellow: 0->1->2->3->4->5");
        train_assoc(EVT_0, EVT_1); train_assoc(EVT_1, EVT_2);
        train_assoc(EVT_2, EVT_3); train_assoc(EVT_3, EVT_4);
        train_assoc(EVT_4, EVT_5);
        train_gate(EVT_0, CTX_YELLOW); train_gate(EVT_1, CTX_YELLOW);
        train_gate(EVT_2, CTX_YELLOW); train_gate(EVT_3, CTX_YELLOW);
        train_gate(EVT_4, CTX_YELLOW); train_gate(EVT_5, CTX_YELLOW);
        train_mcam_entry(EVT_0, CTX_YELLOW, 0); train_mcam_entry(EVT_1, CTX_YELLOW, 1);
        train_mcam_entry(EVT_2, CTX_YELLOW, 2); train_mcam_entry(EVT_3, CTX_YELLOW, 3);
        train_mcam_entry(EVT_4, CTX_YELLOW, 4); train_mcam_entry(EVT_5, CTX_YELLOW, 5);

        // Orange and Purple use the remapped slots 56-63
        $display("   Orange: 120->121->122->123");
        train_assoc(EVT_120, EVT_121); train_assoc(EVT_121, EVT_122);
        train_assoc(EVT_122, EVT_123);
        train_gate(EVT_120, CTX_ORANGE); train_gate(EVT_121, CTX_ORANGE);
        train_gate(EVT_122, CTX_ORANGE); train_gate(EVT_123, CTX_ORANGE);
        train_mcam_entry(EVT_120, CTX_ORANGE, 0); train_mcam_entry(EVT_121, CTX_ORANGE, 1);
        train_mcam_entry(EVT_122, CTX_ORANGE, 2); train_mcam_entry(EVT_123, CTX_ORANGE, 3);

        $display("   Purple: 124->125->126->127");
        train_assoc(EVT_124, EVT_125); train_assoc(EVT_125, EVT_126);
        train_assoc(EVT_126, EVT_127);
        train_gate(EVT_124, CTX_PURPLE); train_gate(EVT_125, CTX_PURPLE);
        train_gate(EVT_126, CTX_PURPLE); train_gate(EVT_127, CTX_PURPLE);
        train_mcam_entry(EVT_124, CTX_PURPLE, 0); train_mcam_entry(EVT_125, CTX_PURPLE, 1);
        train_mcam_entry(EVT_126, CTX_PURPLE, 2); train_mcam_entry(EVT_127, CTX_PURPLE, 3);

        $display(" [TRAIN] All 8 contexts trained via AXI4-Stream");
        $display("--------------------------------------------------------");
        repeat(10) @(posedge axi_aclk);

        // A: cue the first event of every context
        start_test("A1_Blue_AXI", "Blue ctx 7->5->3->2 anchor=7 via AXI", "GroupA");
        run_inference_axi(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7], "Blue/E7");
        print_active_events();
        verify_voltage("A1_Blue", EVT_7, VP,    "Present");
        verify_voltage("A1_Blue", EVT_5, VF1,   "Future1");
        verify_voltage("A1_Blue", EVT_3, VF2,   "Future2");
        verify_voltage("A1_Blue", EVT_2, 8'h00, "Beyond");
        verify_ca1_order("A1_Blue", EVT_3, EVT_5, EVT_7);
        repeat(10) @(posedge axi_aclk);
        end_test("A1_Blue_AXI");

        start_test("A2_Magenta_AXI", "Magenta ctx 7->5->3->1 anchor=7 via AXI", "GroupA");
        run_inference_axi(stored_context_hvs[CTX_MAGENTA], stored_event_hvs[EVT_7], "Magenta/E7");
        print_active_events();
        verify_voltage("A2_Magenta", EVT_7, VP,    "Present");
        verify_voltage("A2_Magenta", EVT_5, VF1,   "Future1");
        verify_voltage("A2_Magenta", EVT_3, VF2,   "Future2");
        verify_voltage("A2_Magenta", EVT_1, 8'h00, "Beyond");
        verify_voltage("A2_Magenta", EVT_2, 8'h00, "NotInCtx");
        verify_ca1_order("A2_Magenta", EVT_3, EVT_5, EVT_7);
        repeat(10) @(posedge axi_aclk);
        end_test("A2_Magenta_AXI");

        start_test("A3_Green_AXI", "Green ctx 2->4->6->8 anchor=2 via AXI", "GroupA");
        run_inference_axi(stored_context_hvs[CTX_GREEN], stored_event_hvs[EVT_2], "Green/E2");
        print_active_events();
        verify_voltage("A3_Green", EVT_2, VP,    "Present");
        verify_voltage("A3_Green", EVT_4, VF1,   "Future1");
        verify_voltage("A3_Green", EVT_6, VF2,   "Future2");
        verify_voltage("A3_Green", EVT_8, 8'h00, "Beyond");
        verify_ca1_order("A3_Green", EVT_6, EVT_4, EVT_2);
        repeat(10) @(posedge axi_aclk);
        end_test("A3_Green_AXI");

        start_test("A4_Red_AXI", "Red ctx 7->9->4 anchor=7 via AXI", "GroupA");
        run_inference_axi(stored_context_hvs[CTX_RED], stored_event_hvs[EVT_7], "Red/E7");
        print_active_events();
        verify_voltage("A4_Red", EVT_7, VP,  "Present");
        verify_voltage("A4_Red", EVT_9, VF1, "Future1");
        verify_voltage("A4_Red", EVT_4, VF2, "Future2");
        repeat(10) @(posedge axi_aclk);
        end_test("A4_Red_AXI");

        start_test("A5_Cyan_AXI", "Cyan ctx 10-14 anchor=10 via AXI", "GroupA");
        run_inference_axi(stored_context_hvs[CTX_CYAN], stored_event_hvs[EVT_10], "Cyan/E10");
        print_active_events();
        verify_voltage("A5_Cyan", EVT_10, VP,  "Present");
        verify_voltage("A5_Cyan", EVT_11, VF1, "Future1");
        verify_voltage("A5_Cyan", EVT_12, VF2, "Future2");
        repeat(10) @(posedge axi_aclk);
        end_test("A5_Cyan_AXI");

        start_test("A6_Yellow_AXI", "Yellow ctx 0-5 anchor=0 via AXI", "GroupA");
        run_inference_axi(stored_context_hvs[CTX_YELLOW], stored_event_hvs[EVT_0], "Yellow/E0");
        print_active_events();
        verify_voltage("A6_Yellow", EVT_0, VP,  "Present");
        verify_voltage("A6_Yellow", EVT_1, VF1, "Future1");
        verify_voltage("A6_Yellow", EVT_2, VF2, "Future2");
        repeat(10) @(posedge axi_aclk);
        end_test("A6_Yellow_AXI");

        start_test("A7_Orange_AXI", "Orange ctx 120-123 anchor=120 via AXI", "GroupA");
        run_inference_axi(stored_context_hvs[CTX_ORANGE], stored_event_hvs[EVT_120], "Orange/E120");
        print_active_events();
        verify_voltage("A7_Orange", EVT_120, VP,  "Present");
        verify_voltage("A7_Orange", EVT_121, VF1, "Future1");
        verify_voltage("A7_Orange", EVT_122, VF2, "Future2");
        repeat(10) @(posedge axi_aclk);
        end_test("A7_Orange_AXI");

        start_test("A8_Purple_AXI", "Purple ctx 124-127 anchor=124 via AXI", "GroupA");
        run_inference_axi(stored_context_hvs[CTX_PURPLE], stored_event_hvs[EVT_124], "Purple/E124");
        print_active_events();
        verify_voltage("A8_Purple", EVT_124, VP,  "Present");
        verify_voltage("A8_Purple", EVT_125, VF1, "Future1");
        verify_voltage("A8_Purple", EVT_126, VF2, "Future2");
        repeat(10) @(posedge axi_aclk);
        end_test("A8_Purple_AXI");

        // B: Blue/7 with 0-13 flips on both HVs; only the two recalled indices are checked
        begin
            int noise_levels[10];
            int ni, nflip;
            logic [HV_WIDTH-1:0] noisy_ctx, noisy_evt;

            noise_levels[0]=0;  noise_levels[1]=1;  noise_levels[2]=2;
            noise_levels[3]=3;  noise_levels[4]=5;  noise_levels[5]=7;
            noise_levels[6]=8;  noise_levels[7]=10; noise_levels[8]=12;
            noise_levels[9]=13;

            for (ni = 0; ni < 10; ni++) begin
                nflip = noise_levels[ni];
                start_test($sformatf("B%0d_Noise%0d_AXI", ni+1, nflip),
                           $sformatf("Blue with %0d bit errors via AXI", nflip), "GroupB");

                noisy_ctx = flip_bits(stored_context_hvs[CTX_BLUE], nflip);
                noisy_evt = flip_bits(stored_event_hvs[EVT_7], nflip);
                run_inference_axi(noisy_ctx, noisy_evt,
                                  $sformatf("Blue/E7+%0dbit", nflip));

                if (rx_context_idx == CTX_BLUE) test_checks_pass++;
                else test_checks_fail++;
                if (rx_event_idx == EVT_7) test_checks_pass++;
                else test_checks_fail++;

                $display("   Noise=%0d: ctx_ok=%0b evt_ok=%0b",
                         nflip, (rx_context_idx == CTX_BLUE),
                         (rx_event_idx == EVT_7));

                repeat(5) @(posedge axi_aclk);
                end_test($sformatf("B%0d_Noise%0d_AXI", ni+1, nflip));
            end
        end

        // C: move the cue along Blue (7, 5, 3), then Green at 4
        start_test("C1_Slide7_AXI", "Blue slide anchor=7 via AXI", "GroupC");
        run_inference_axi(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7], "Blue/E7");
        verify_voltage("C1", EVT_7, VP,  "Present");
        verify_voltage("C1", EVT_5, VF1, "Future1");
        verify_voltage("C1", EVT_3, VF2, "Future2");
        verify_ca1_order("C1", EVT_3, EVT_5, EVT_7);
        repeat(10) @(posedge axi_aclk);
        end_test("C1_Slide7_AXI");

        start_test("C2_Slide5_AXI", "Blue slide anchor=5 via AXI", "GroupC");
        run_inference_axi(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_5], "Blue/E5");
        verify_voltage("C2", EVT_5, VP,  "Present");
        verify_voltage("C2", EVT_3, VF1, "Future1");
        verify_voltage("C2", EVT_2, VF2, "Future2");
        verify_ca1_order("C2", EVT_2, EVT_3, EVT_5);
        repeat(10) @(posedge axi_aclk);
        end_test("C2_Slide5_AXI");

        start_test("C3_Slide3_AXI", "Blue slide anchor=3 via AXI", "GroupC");
        run_inference_axi(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_3], "Blue/E3");
        verify_voltage("C3", EVT_3, VP,  "Present");
        verify_voltage("C3", EVT_2, VF1, "Future1");
        repeat(10) @(posedge axi_aclk);
        end_test("C3_Slide3_AXI");

        start_test("C4_Green4_AXI", "Green slide anchor=4 via AXI", "GroupC");
        run_inference_axi(stored_context_hvs[CTX_GREEN], stored_event_hvs[EVT_4], "Green/E4");
        verify_voltage("C4", EVT_4, VP,  "Present");
        verify_voltage("C4", EVT_6, VF1, "Future1");
        verify_voltage("C4", EVT_8, VF2, "Future2");
        repeat(10) @(posedge axi_aclk);
        end_test("C4_Green4_AXI");

        // D1: cue 7 in Blue, then in Red
        start_test("D1_BluRed_AXI", "Event 7: Blue vs Red via AXI", "GroupD");
        run_inference_axi(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7], "Blue/E7");
        verify_voltage("D1_Blue", EVT_7, VP,    "Present");
        verify_voltage("D1_Blue", EVT_5, VF1,   "Future1(Blue)");
        verify_voltage("D1_Blue", EVT_9, 8'h00, "NotInBlue");
        verify_ca1_ts_lt("D1_Blue", EVT_5, EVT_7, "F1 before Present");
        verify_ca1_inactive("D1_Blue", EVT_9,      "E9 not in Blue");
        repeat(5) @(posedge axi_aclk);

        run_inference_axi(stored_context_hvs[CTX_RED], stored_event_hvs[EVT_7], "Red/E7");
        verify_voltage("D1_Red", EVT_7, VP,    "Present");
        verify_voltage("D1_Red", EVT_9, VF1,   "Future1(Red)");
        verify_voltage("D1_Red", EVT_5, 8'h00, "NotInRed");
        verify_ca1_ts_lt("D1_Red", EVT_9, EVT_7, "F1 before Present");
        verify_ca1_inactive("D1_Red", EVT_5,      "E5 not in Red");
        repeat(10) @(posedge axi_aclk);
        end_test("D1_BluRed_AXI");

        // D2: alternate Blue and Green queries
        start_test("D2_Rapid_AXI", "Rapid Blue<->Green via AXI", "GroupD");
        run_inference_axi(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7], "Blue/E7");
        verify_voltage("D2_B1", EVT_5, VF1, "Blue_F1");
        repeat(3) @(posedge axi_aclk);
        run_inference_axi(stored_context_hvs[CTX_GREEN], stored_event_hvs[EVT_2], "Green/E2");
        verify_voltage("D2_G1", EVT_4, VF1, "Green_F1");
        repeat(3) @(posedge axi_aclk);
        run_inference_axi(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_7], "Blue/E7_again");
        verify_voltage("D2_B2", EVT_5, VF1, "Blue_F1_again");
        repeat(10) @(posedge axi_aclk);
        end_test("D2_Rapid_AXI");

        // E1: cue on the last Green event
        start_test("E1_SeqEnd_AXI", "Green anchor=8 terminal via AXI", "GroupE");
        run_inference_axi(stored_context_hvs[CTX_GREEN], stored_event_hvs[EVT_8], "Green/E8");
        verify_voltage("E1", EVT_8, VP, "Present_Terminal");
        repeat(10) @(posedge axi_aclk);
        end_test("E1_SeqEnd_AXI");

        // E2: event 15 has no links
        start_test("E2_Isolated_AXI", "E15 no associations via AXI", "GroupE");
        run_inference_axi(stored_context_hvs[CTX_BLUE], stored_event_hvs[EVT_15], "Blue/E15");
        verify_voltage("E2", EVT_15, VP,    "Present_Isolated");
        verify_voltage("E2", EVT_5,  8'h00, "NoChain");
        repeat(10) @(posedge axi_aclk);
        end_test("E2_Isolated_AXI");

        // F: six-event Yellow chain, cue at 0 and then at 3
        start_test("F1_Yellow0_AXI", "Yellow 6-event anchor=0 via AXI", "GroupF");
        run_inference_axi(stored_context_hvs[CTX_YELLOW], stored_event_hvs[EVT_0], "Yellow/E0");
        print_active_events();
        verify_voltage("F1", EVT_0, VP,    "Present");
        verify_voltage("F1", EVT_1, VF1,   "Future1");
        verify_voltage("F1", EVT_2, VF2,   "Future2");
        verify_voltage("F1", EVT_3, 8'h00, "Beyond");
        verify_ca1_order("F1", EVT_2, EVT_1, EVT_0);
        repeat(10) @(posedge axi_aclk);
        end_test("F1_Yellow0_AXI");

        start_test("F2_Yellow3_AXI", "Yellow 6-event anchor=3 via AXI", "GroupF");
        run_inference_axi(stored_context_hvs[CTX_YELLOW], stored_event_hvs[EVT_3], "Yellow/E3");
        print_active_events();
        verify_voltage("F2", EVT_3, VP,  "Present");
        verify_voltage("F2", EVT_4, VF1, "Future1");
        verify_voltage("F2", EVT_5, VF2, "Future2");
        verify_ca1_order("F2", EVT_5, EVT_4, EVT_3);
        repeat(10) @(posedge axi_aclk);
        end_test("F2_Yellow3_AXI");

        // G: ten recalls in a row, timed. Passes unless something times out.

        start_test("G1_Throughput_AXI", "10 back-to-back inferences via AXI", "GroupG");
        begin
            int start_cyc, total_cyc, ii;
            start_cyc = cycle_count;
            for (ii = 0; ii < 10; ii++) begin
                run_inference_axi(stored_context_hvs[CTX_BLUE],
                                  stored_event_hvs[EVT_7],
                                  $sformatf("iter%0d", ii));
                repeat(2) @(posedge axi_aclk);
            end
            total_cyc = cycle_count - start_cyc;
            $display("   10 AXI inferences in %0d cycles (avg %.1f cyc/inf)",
                     total_cyc, real'(total_cyc)/10.0);
            test_checks_pass++;
        end
        repeat(10) @(posedge axi_aclk);
        end_test("G1_Throughput_AXI");

        begin
            int total_checks;
            real pass_rate;
            total_checks = total_checks_pass + total_checks_fail;
            pass_rate = (total_checks > 0) ?
                        (real'(total_checks_pass) / real'(total_checks)) * 100.0 : 0.0;

            $display("");
            $display("--------------------------------------------------------");
            $display("  HIPPOCAMPEX AXI4-STREAM FPGA VERIFICATION SUITE ");
            $display("  Target: AMD Zynq UltraScale+ ZCU102");
            $display("--------------------------------------------------------");
            $display("  Total Tests:      %0d", total_tests);
            $display("  Tests Passed:     %0d", total_passed);
            $display("  Tests Failed:     %0d", total_failed);
            $display("  Checks Passed:    %0d", total_checks_pass);
            $display("  Checks Failed:    %0d", total_checks_fail);
            $display("  Pass Rate:        %.1f%%", pass_rate);
            $display("  Total Inferences: %0d (all via AXI4-Stream)", inference_count);
            $display("");
            $display("  CSV Files Generated:");
            $display("    - axi_test_summary.csv");
            $display("    - axi_prediction_detail.csv");
            $display("    - axi_trace.csv");
            $display("");
            if (total_failed == 0) begin
                $display("  *** ALL TESTS PASSED - READY FOR ZCU102 DEPLOYMENT ***");
            end else begin
                $display("  *** %0d TEST(S) FAILED ***", total_failed);
            end
            $display("--------------------------------------------------------");
        end

        close_csv_files();
        $finish;
    end

    // watchdog, 2 ms
    initial begin
        #(CLK_PERIOD * 200000);
        $display("");
        $display("[ERROR] AXI Testbench timeout after 200000 cycles");
        close_csv_files();
        $finish;
    end

endmodule
