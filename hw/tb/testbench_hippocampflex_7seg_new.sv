`timescale 1ns/1ps
// testbench_hippocampex_7seg_new: larger 7-segment test through the AXI-Stream
// wrapper, with the digit and colour set from the paper's benchmark figure.
//    10 events  : digits 0-9 (HVs copied from Event_HVs.txt)
//     8 contexts: Black / Red / Yellow / Green / Magenta / Blue / Cyan / White
//                 (HVs copied from Context_HVs.txt)
//
// Learned sequences
//   Blue    : 0->1->2->3->4->5   (ascending)
//   Red     : 9->7->5->3->1      (descending odd)
//   Green   : 0->2->4->6->8      (even ascending)
//   Magenta : 1->3->5->7->9      (odd ascending)
//   White   : 3->5->7->9         (same links as Magenta, different gates)
//   Black   : 0->3->6->9         (multiples of 3)
//   Yellow  : 2->4->6->8         (same links as Green from 2 on)
//   Cyan    : 8->6->4->2->0      (even descending)
//
// Tests:
//   T1   Blue    / D0  -> P=D0  F1=D1  F2=D2
//   T2   Red     / D9  -> P=D9  F1=D7  F2=D5
//   T3   Green   / D0  -> P=D0  F1=D2  F2=D4
//   T4   Magenta / D1  -> P=D1  F1=D3  F2=D5
//   T5   White   / D3  -> P=D3  F1=D5  F2=D7
//   T6   Sliding : Blue/D3  -> P=D3  F1=D4  F2=D5
//   T7   Black   / D0  -> P=D0  F1=D3  F2=D6
//   T8   Yellow  / D2  -> P=D2  F1=D4  F2=D6
//   T9   Cyan    / D8  -> P=D8  F1=D6  F2=D4
//   T10  Isolate : D6 in Green (F1=D8) vs Cyan (F1=D4)
//   T11  Isolate : D3 in Red   (F1=D1) vs Magenta (F1=D5)
//
// DUT is hippocampex_axis_wrapper with NUM_EVT=128 (the board build uses 64).
// Commands 0x1-0x6; a recall reply is 1 + NUM_EVT/4 + NUM_EVT/32 + NUM_EVT/4 beats.

module testbench_hippocampex_7seg_new;

    localparam AXIS_DW   = 32;
    localparam HV_WIDTH  = 128;
    localparam NUM_CTX   = 16;
    localparam NUM_EVT   = 128;
    localparam CNTR_W    = 6;
    localparam VOLT_W    = 8;
    localparam CLK_HALF  = 5;   // 100 MHz

    localparam CTX_AW = $clog2(NUM_CTX);
    localparam EVT_AW = $clog2(NUM_EVT);

    // MCAM tags
    localparam VP  = 8'h40;  // present
    localparam VF1 = 8'h80;  // future-1
    localparam VF2 = 8'hC0;  // future-2

    // HVs copied from Event_HVs.txt / Context_HVs.txt, highest word first
    localparam [HV_WIDTH-1:0] EV_DIGIT_0 = {32'h8ad92b85, 32'hd0f89f4c, 32'ha2346375, 32'h603443c5};
    localparam [HV_WIDTH-1:0] EV_DIGIT_1 = {32'h08cd03a7, 32'h90f03a5c, 32'ha29463fd, 32'h607dd3c5};
    localparam [HV_WIDTH-1:0] EV_DIGIT_2 = {32'h98e17ee7, 32'h58c20f44, 32'h32107a2f, 32'h6630c3c5};
    localparam [HV_WIDTH-1:0] EV_DIGIT_3 = {32'h18e13685, 32'h1ac03e5c, 32'ha2947a2f, 32'h6039c365};
    localparam [HV_WIDTH-1:0] EV_DIGIT_4 = {32'h80dd22a5, 32'h92b03a5c, 32'ha2146375, 32'h667dd17f};
    localparam [HV_WIDTH-1:0] EV_DIGIT_5 = {32'h02f12685, 32'h5a88be44, 32'h82907a27, 32'h6078417f};
    localparam [HV_WIDTH-1:0] EV_DIGIT_6 = {32'h8ad967c5, 32'h52989f44, 32'ha2307b25, 32'h6078416f};
    localparam [HV_WIDTH-1:0] EV_DIGIT_7 = {32'h08e11b87, 32'h9af0ba5c, 32'h8294727d, 32'h603dc3c5};
    localparam [HV_WIDTH-1:0] EV_DIGIT_8 = {32'h98f13e85, 32'h52d09f44, 32'ha2107b25, 32'h6630436d};
    localparam [HV_WIDTH-1:0] EV_DIGIT_9 = {32'h8af12285, 32'h52b81f4c, 32'ha2147325, 32'h607cc167};

    localparam [HV_WIDTH-1:0] CTX_BLACK_HV   = {32'h89e79469, 32'h6e502be1, 32'ha84ace89, 32'h3b530f45};
    localparam [HV_WIDTH-1:0] CTX_RED_HV     = {32'h9da79178, 32'h26988b09, 32'hb44a8a89, 32'h1a512447};
    localparam [HV_WIDTH-1:0] CTX_YELLOW_HV  = {32'h8da7906c, 32'h7e982be4, 32'h8f4a8bb9, 32'h0c516dc7};
    localparam [HV_WIDTH-1:0] CTX_GREEN_HV   = {32'h0d25986a, 32'h6eaa3bf1, 32'hec63808b, 32'h1a152d77};
    localparam [HV_WIDTH-1:0] CTX_MAGENTA_HV = {32'h99e7957d, 32'h36508b0c, 32'h934acfb9, 32'h2d5346c5};
    localparam [HV_WIDTH-1:0] CTX_BLUE_HV    = {32'h19659d7b, 32'h26629b19, 32'hf063c48b, 32'h3b170675};
    localparam [HV_WIDTH-1:0] CTX_CYAN_HV    = {32'h09659c6f, 32'h7e623bf4, 32'hcb63c5bb, 32'h2d174ff5};
    localparam [HV_WIDTH-1:0] CTX_WHITE_HV   = {32'h1d25997e, 32'h36aa9b1c, 32'hd76381bb, 32'h0c1564f7};

    // slots: contexts Black=0 .. White=7, digit N goes to event slot N
    localparam CTX_BLACK   = 4'd0;
    localparam CTX_RED     = 4'd1;
    localparam CTX_YELLOW  = 4'd2;
    localparam CTX_GREEN   = 4'd3;
    localparam CTX_MAGENTA = 4'd4;
    localparam CTX_BLUE    = 4'd5;
    localparam CTX_CYAN    = 4'd6;
    localparam CTX_WHITE   = 4'd7;

    localparam EVT_D0 = 7'd0;
    localparam EVT_D1 = 7'd1;
    localparam EVT_D2 = 7'd2;
    localparam EVT_D3 = 7'd3;
    localparam EVT_D4 = 7'd4;
    localparam EVT_D5 = 7'd5;
    localparam EVT_D6 = 7'd6;
    localparam EVT_D7 = 7'd7;
    localparam EVT_D8 = 7'd8;
    localparam EVT_D9 = 7'd9;

    logic        axi_aclk, axi_aresetn;
    logic [31:0] s_axis_tdata;
    logic [3:0]  s_axis_tkeep;
    logic        s_axis_tvalid, s_axis_tready, s_axis_tlast;
    logic [31:0] m_axis_tdata;
    logic [3:0]  m_axis_tkeep;
    logic        m_axis_tvalid, m_axis_tready, m_axis_tlast;
    logic        irq_result_ready;

    hippocampex_axis_wrapper #(
        .HV_WIDTH    (HV_WIDTH),
        .NUM_CONTEXTS(NUM_CTX),
        .NUM_EVENTS  (NUM_EVT),
        .CNTR_WIDTH  (CNTR_W),
        .VOLTAGE_WIDTH(VOLT_W)
    ) dut (
        .axi_aclk       (axi_aclk),
        .axi_aresetn    (axi_aresetn),
        .s_axis_tdata   (s_axis_tdata),
        .s_axis_tkeep   (s_axis_tkeep),
        .s_axis_tvalid  (s_axis_tvalid),
        .s_axis_tready  (s_axis_tready),
        .s_axis_tlast   (s_axis_tlast),
        .m_axis_tdata   (m_axis_tdata),
        .m_axis_tkeep   (m_axis_tkeep),
        .m_axis_tvalid  (m_axis_tvalid),
        .m_axis_tready  (m_axis_tready),
        .m_axis_tlast   (m_axis_tlast),
        .irq_result_ready(irq_result_ready)
    );

    // clock; the TB always accepts output beats
    initial begin axi_aclk = 0; forever #CLK_HALF axi_aclk = ~axi_aclk; end
    assign m_axis_tready = 1'b1;

    // fields from the last A1 reply
    int pass_cnt = 0, fail_cnt = 0;
    logic [VOLT_W-1:0]  rx_volt [NUM_EVT];
    logic [CTX_AW-1:0]  rx_ctx_idx;
    logic [EVT_AW-1:0]  rx_evt_idx;
    logic [NUM_EVT-1:0] rx_ordered_onehot;
    logic [CNTR_W-1:0]  rx_timestamps [NUM_EVT];  // CA1 rank: 0 = furthest future, 0x3F = inactive

    // send one beat and hold it until tready
    task automatic axis_beat(input logic [31:0] data, input logic last);
        @(posedge axi_aclk);
        s_axis_tdata  <= data;
        s_axis_tkeep  <= 4'hF;
        s_axis_tvalid <= 1'b1;
        s_axis_tlast  <= last;
        do @(posedge axi_aclk); while (!s_axis_tready);
        s_axis_tvalid <= 1'b0;
        s_axis_tlast  <= 1'b0;
    endtask

    // wait for the C1 ack; only warns on timeout
    task automatic wait_ack(input logic [3:0] cmd);
        int t;
        for (t = 0; t < 300; t++) begin
            @(posedge axi_aclk);
            if (m_axis_tvalid && m_axis_tdata[31:24] == 8'hC1) return;
        end
        $display("  [WARN] ACK timeout CMD=0x%h", cmd);
    endtask

    // Training commands, packet layouts as in the wrapper header.

    // 0x1 context HV: header + 4 HV words, low word first
    task automatic train_ctx(input int addr, input logic [HV_WIDTH-1:0] hv, input string name);
        $display("  [CTX] slot=%0d  '%s'", addr, name);
        axis_beat({4'h1, {(28-CTX_AW){1'b0}}, addr[CTX_AW-1:0]}, 1'b0);
        axis_beat(hv[ 31:  0],  1'b0);
        axis_beat(hv[ 63: 32],  1'b0);
        axis_beat(hv[ 95: 64],  1'b0);
        axis_beat(hv[127: 96],  1'b1);
        wait_ack(4'h1);
    endtask

    // 0x2 event HV: same layout
    task automatic train_evt(input int addr, input logic [HV_WIDTH-1:0] hv, input string name);
        $display("  [EVT] slot=%0d  '%s'", addr, name);
        axis_beat({4'h2, {(28-EVT_AW){1'b0}}, addr[EVT_AW-1:0]}, 1'b0);
        axis_beat(hv[ 31:  0],  1'b0);
        axis_beat(hv[ 63: 32],  1'b0);
        axis_beat(hv[ 95: 64],  1'b0);
        axis_beat(hv[127: 96],  1'b1);
        wait_ack(4'h2);
    endtask

    // 0x3 crossbar link from_e -> to_e
    task automatic train_assoc(input int from_e, input int to_e);
        axis_beat({4'h3, 7'd0, from_e[6:0], to_e[6:0], 6'd0, 1'b1}, 1'b1);
        wait_ack(4'h3);
    endtask

    // 0x4 gate: allow evt in ctx
    task automatic train_gate(input int evt, input int ctx);
        axis_beat({4'h4, 7'd0, evt[6:0], ctx[6:0], 6'd0, 1'b1}, 1'b1);
        wait_ack(4'h4);
    endtask

    // 0x5 MCAM time code for (evt, ctx)
    task automatic train_mcam(input int evt, input int ctx, input int t_code);
        axis_beat({4'h5, 7'd0, evt[6:0], ctx[6:0], 1'b0, t_code[CNTR_W-1:0]}, 1'b1);
        wait_ack(4'h5);
    endtask

    // 0x6 recall: 9 beats out (header, context HV, event HV). Reads the A1 reply
    // into rx_*: header, voltages, CA1 one-hot, CA1 ranks (69 beats at NUM_EVT=128).
    task automatic infer(
        input logic [HV_WIDTH-1:0] ctx_hv,
        input logic [HV_WIDTH-1:0] evt_hv,
        input string               label
    );
        int t, b;
        $display("  [INF] %s", label);
        axis_beat({4'h6, 28'd0},    1'b0);
        axis_beat(ctx_hv[ 31:  0],  1'b0);
        axis_beat(ctx_hv[ 63: 32],  1'b0);
        axis_beat(ctx_hv[ 95: 64],  1'b0);
        axis_beat(ctx_hv[127: 96],  1'b0);
        axis_beat(evt_hv[ 31:  0],  1'b0);
        axis_beat(evt_hv[ 63: 32],  1'b0);
        axis_beat(evt_hv[ 95: 64],  1'b0);
        axis_beat(evt_hv[127: 96],  1'b1);

        // look for the A1 header; anything else is skipped
        for (t = 0; t < 3000; t++) begin
            @(posedge axi_aclk);
            if (m_axis_tvalid && m_axis_tdata[31:24] == 8'hA1) begin
                rx_ctx_idx = m_axis_tdata[23:17];
                rx_evt_idx = m_axis_tdata[16:10];
                $display("         -> ctx_idx=%0d  evt_idx=%0d",
                         rx_ctx_idx, rx_evt_idx);

                // voltages, 4 per beat
                for (b = 0; b < NUM_EVT/4; b++) begin
                    @(posedge axi_aclk);
                    while (!m_axis_tvalid) @(posedge axi_aclk);
                    rx_volt[b*4+0] = m_axis_tdata[ 7: 0];
                    rx_volt[b*4+1] = m_axis_tdata[15: 8];
                    rx_volt[b*4+2] = m_axis_tdata[23:16];
                    rx_volt[b*4+3] = m_axis_tdata[31:24];
                end

                // CA1 one-hot, 32 per beat
                for (b = 0; b < NUM_EVT/32; b++) begin
                    @(posedge axi_aclk);
                    while (!m_axis_tvalid) @(posedge axi_aclk);
                    rx_ordered_onehot[b*32 +: 32] = m_axis_tdata;
                end

                // CA1 ranks, 4 per beat (8-bit fields, kept to CNTR_W bits)
                for (b = 0; b < NUM_EVT/4; b++) begin
                    @(posedge axi_aclk);
                    while (!m_axis_tvalid) @(posedge axi_aclk);
                    rx_timestamps[b*4+0] = m_axis_tdata[ 7: 0];
                    rx_timestamps[b*4+1] = m_axis_tdata[15: 8];
                    rx_timestamps[b*4+2] = m_axis_tdata[23:16];
                    rx_timestamps[b*4+3] = m_axis_tdata[31:24];
                end

                return;
            end
        end
        $display("  [ERROR] Inference timeout: %s", label);
    endtask

    task automatic chk(
        input string             tname,
        input int                evt,
        input logic [VOLT_W-1:0] exp,
        input string             role
    );
        if (rx_volt[evt] === exp) begin
            $display("    [PASS] %s  E%0d=0x%02h  %s",
                     tname, evt, rx_volt[evt], role);
            pass_cnt++;
        end else begin
            $display("    [FAIL] %s  E%0d=0x%02h (exp=0x%02h)  %s",
                     tname, evt, rx_volt[evt], exp, role);
            fail_cnt++;
        end
    endtask

    // CA1 order checks. Rank 0 is the highest voltage, so F2 < F1 < present.
    task automatic chk_order(
        input string tname,
        input int    evt_f2,
        input int    evt_f1,
        input int    evt_p
    );
        if (rx_timestamps[evt_f2] < rx_timestamps[evt_f1] &&
            rx_timestamps[evt_f1] < rx_timestamps[evt_p]) begin
            $display("    [PASS] %s CA1-order: ts[E%0d]=%0d < ts[E%0d]=%0d < ts[E%0d]=%0d",
                     tname,
                     evt_f2, rx_timestamps[evt_f2],
                     evt_f1, rx_timestamps[evt_f1],
                     evt_p,  rx_timestamps[evt_p]);
            pass_cnt++;
        end else begin
            $display("    [FAIL] %s CA1-order: ts[E%0d]=%0d  ts[E%0d]=%0d  ts[E%0d]=%0d",
                     tname,
                     evt_f2, rx_timestamps[evt_f2],
                     evt_f1, rx_timestamps[evt_f1],
                     evt_p,  rx_timestamps[evt_p]);
            fail_cnt++;
        end
    endtask

    task automatic chk_ts_lt(
        input string tname,
        input int    evt_earlier,
        input int    evt_later,
        input string role
    );
        if (rx_timestamps[evt_earlier] < rx_timestamps[evt_later]) begin
            $display("    [PASS] %s CA1-order: ts[E%0d]=%0d < ts[E%0d]=%0d  (%s)",
                     tname,
                     evt_earlier, rx_timestamps[evt_earlier],
                     evt_later,   rx_timestamps[evt_later], role);
            pass_cnt++;
        end else begin
            $display("    [FAIL] %s CA1-order: ts[E%0d]=%0d NOT < ts[E%0d]=%0d  (%s)",
                     tname,
                     evt_earlier, rx_timestamps[evt_earlier],
                     evt_later,   rx_timestamps[evt_later], role);
            fail_cnt++;
        end
    endtask

    // event must be unranked (all ones)
    task automatic chk_ts_inactive(
        input string tname,
        input int    evt,
        input string role
    );
        logic [CNTR_W-1:0] inactive_val;
        inactive_val = {CNTR_W{1'b1}};
        if (rx_timestamps[evt] == inactive_val) begin
            $display("    [PASS] %s CA1-inactive: ts[E%0d]=0x%0h  (%s)",
                     tname, evt, rx_timestamps[evt], role);
            pass_cnt++;
        end else begin
            $display("    [FAIL] %s CA1-inactive: ts[E%0d]=0x%0h (exp=0x%0h)  (%s)",
                     tname, evt, rx_timestamps[evt], inactive_val, role);
            fail_cnt++;
        end
    endtask

    // print every nonzero voltage
    task automatic dump_active();
        $display("    +--------+--------+-----------+");
        $display("    | Evt    | Merged | Role      |");
        $display("    +--------+--------+-----------+");
        for (int i = 0; i < NUM_EVT; i++) begin
            if (rx_volt[i] != 8'h00) begin
                string role;
                case (rx_volt[i])
                    VP:  role = "Present";
                    VF1: role = "Future-1";
                    VF2: role = "Future-2";
                    default: role = "Decay";
                endcase
                $display("    | D%0d(E%0d) | 0x%02h   | %-9s |",
                         i, i, rx_volt[i], role);
            end
        end
        $display("    +--------+--------+-----------+");
    endtask

    logic [HV_WIDTH-1:0] ctx_hv [8];
    logic [HV_WIDTH-1:0] evt_hv [10];

    initial begin
        s_axis_tdata  = 32'h0;
        s_axis_tkeep  = 4'hF;
        s_axis_tvalid = 1'b0;
        s_axis_tlast  = 1'b0;
        axi_aresetn   = 1'b0;

        $display("---------------------------------------------------");
        $display("  HippocamPEX Testbench  -  New HV Edition");
        $display("  8 Contexts  |  10 Events (digits 0-9)");
        $display("  Seqs: Blue=0..5  Red=9,7,5,3,1  Green=0,2,4,6,8");
        $display("        Magenta=1,3,5,7,9  White=3,5,7,9");
        $display("        Black=0,3,6,9  Yellow=2,4,6,8  Cyan=8,6,4,2,0");
        $display("---------------------------------------------------");

        repeat(10) @(posedge axi_aclk);
        axi_aresetn = 1'b1;
        repeat(5)   @(posedge axi_aclk);

        ctx_hv[CTX_BLACK]   = CTX_BLACK_HV;
        ctx_hv[CTX_RED]     = CTX_RED_HV;
        ctx_hv[CTX_YELLOW]  = CTX_YELLOW_HV;
        ctx_hv[CTX_GREEN]   = CTX_GREEN_HV;
        ctx_hv[CTX_MAGENTA] = CTX_MAGENTA_HV;
        ctx_hv[CTX_BLUE]    = CTX_BLUE_HV;
        ctx_hv[CTX_CYAN]    = CTX_CYAN_HV;
        ctx_hv[CTX_WHITE]   = CTX_WHITE_HV;
        evt_hv[EVT_D0] = EV_DIGIT_0;
        evt_hv[EVT_D1] = EV_DIGIT_1;
        evt_hv[EVT_D2] = EV_DIGIT_2;
        evt_hv[EVT_D3] = EV_DIGIT_3;
        evt_hv[EVT_D4] = EV_DIGIT_4;
        evt_hv[EVT_D5] = EV_DIGIT_5;
        evt_hv[EVT_D6] = EV_DIGIT_6;
        evt_hv[EVT_D7] = EV_DIGIT_7;
        evt_hv[EVT_D8] = EV_DIGIT_8;
        evt_hv[EVT_D9] = EV_DIGIT_9;

        $display("\n---- Phase 1: Context HVs (8 colours) ----");
        train_ctx(CTX_BLACK,   ctx_hv[CTX_BLACK],   "Black");
        train_ctx(CTX_RED,     ctx_hv[CTX_RED],     "Red");
        train_ctx(CTX_YELLOW,  ctx_hv[CTX_YELLOW],  "Yellow");
        train_ctx(CTX_GREEN,   ctx_hv[CTX_GREEN],   "Green");
        train_ctx(CTX_MAGENTA, ctx_hv[CTX_MAGENTA], "Magenta");
        train_ctx(CTX_BLUE,    ctx_hv[CTX_BLUE],    "Blue");
        train_ctx(CTX_CYAN,    ctx_hv[CTX_CYAN],    "Cyan");
        train_ctx(CTX_WHITE,   ctx_hv[CTX_WHITE],   "White");

        $display("\n---- Phase 2: Event HVs (digits 0-9) ----");
        train_evt(EVT_D0, evt_hv[EVT_D0], "Digit0_abcdefg");
        train_evt(EVT_D1, evt_hv[EVT_D1], "Digit1_bc");
        train_evt(EVT_D2, evt_hv[EVT_D2], "Digit2_abdeg");
        train_evt(EVT_D3, evt_hv[EVT_D3], "Digit3_abcdg");
        train_evt(EVT_D4, evt_hv[EVT_D4], "Digit4_bcfg");
        train_evt(EVT_D5, evt_hv[EVT_D5], "Digit5_acdfg");
        train_evt(EVT_D6, evt_hv[EVT_D6], "Digit6_acdefg");
        train_evt(EVT_D7, evt_hv[EVT_D7], "Digit7_abc");
        train_evt(EVT_D8, evt_hv[EVT_D8], "Digit8_abcdefg");
        train_evt(EVT_D9, evt_hv[EVT_D9], "Digit9_abcdfg");

        // Links are shared by all contexts; the gates decide which context may use them.
        $display("\n---- Phase 3: Crossbar + MCAM ----");

        $display("  [BLUE] 0->1->2->3->4->5");
        train_assoc(EVT_D0, EVT_D1); train_assoc(EVT_D1, EVT_D2);
        train_assoc(EVT_D2, EVT_D3); train_assoc(EVT_D3, EVT_D4);
        train_assoc(EVT_D4, EVT_D5);
        train_gate(EVT_D0, CTX_BLUE); train_gate(EVT_D1, CTX_BLUE);
        train_gate(EVT_D2, CTX_BLUE); train_gate(EVT_D3, CTX_BLUE);
        train_gate(EVT_D4, CTX_BLUE); train_gate(EVT_D5, CTX_BLUE);
        train_mcam(EVT_D0, CTX_BLUE, 1); train_mcam(EVT_D1, CTX_BLUE, 2);
        train_mcam(EVT_D2, CTX_BLUE, 3); train_mcam(EVT_D3, CTX_BLUE, 4);
        train_mcam(EVT_D4, CTX_BLUE, 5); train_mcam(EVT_D5, CTX_BLUE, 6);

        $display("  [RED] 9->7->5->3->1");
        train_assoc(EVT_D9, EVT_D7); train_assoc(EVT_D7, EVT_D5);
        train_assoc(EVT_D5, EVT_D3); train_assoc(EVT_D3, EVT_D1);
        train_gate(EVT_D9, CTX_RED); train_gate(EVT_D7, CTX_RED);
        train_gate(EVT_D5, CTX_RED); train_gate(EVT_D3, CTX_RED);
        train_gate(EVT_D1, CTX_RED);
        train_mcam(EVT_D9, CTX_RED, 1); train_mcam(EVT_D7, CTX_RED, 2);
        train_mcam(EVT_D5, CTX_RED, 3); train_mcam(EVT_D3, CTX_RED, 4);
        train_mcam(EVT_D1, CTX_RED, 5);

        $display("  [GREEN] 0->2->4->6->8");
        train_assoc(EVT_D0, EVT_D2); train_assoc(EVT_D2, EVT_D4);
        train_assoc(EVT_D4, EVT_D6); train_assoc(EVT_D6, EVT_D8);
        train_gate(EVT_D0, CTX_GREEN); train_gate(EVT_D2, CTX_GREEN);
        train_gate(EVT_D4, CTX_GREEN); train_gate(EVT_D6, CTX_GREEN);
        train_gate(EVT_D8, CTX_GREEN);
        train_mcam(EVT_D0, CTX_GREEN, 1); train_mcam(EVT_D2, CTX_GREEN, 2);
        train_mcam(EVT_D4, CTX_GREEN, 3); train_mcam(EVT_D6, CTX_GREEN, 4);
        train_mcam(EVT_D8, CTX_GREEN, 5);

        $display("  [MAGENTA] 1->3->5->7->9");
        train_assoc(EVT_D1, EVT_D3); train_assoc(EVT_D3, EVT_D5);
        train_assoc(EVT_D5, EVT_D7); train_assoc(EVT_D7, EVT_D9);
        train_gate(EVT_D1, CTX_MAGENTA); train_gate(EVT_D3, CTX_MAGENTA);
        train_gate(EVT_D5, CTX_MAGENTA); train_gate(EVT_D7, CTX_MAGENTA);
        train_gate(EVT_D9, CTX_MAGENTA);
        train_mcam(EVT_D1, CTX_MAGENTA, 1); train_mcam(EVT_D3, CTX_MAGENTA, 2);
        train_mcam(EVT_D5, CTX_MAGENTA, 3); train_mcam(EVT_D7, CTX_MAGENTA, 4);
        train_mcam(EVT_D9, CTX_MAGENTA, 5);

        // White reuses Magenta's links; rewriting them is harmless
        $display("  [WHITE] 3->5->7->9 (shared assoc edges, gating isolates)");
        train_assoc(EVT_D3, EVT_D5);
        train_assoc(EVT_D5, EVT_D7);
        train_assoc(EVT_D7, EVT_D9);
        train_gate(EVT_D3, CTX_WHITE); train_gate(EVT_D5, CTX_WHITE);
        train_gate(EVT_D7, CTX_WHITE); train_gate(EVT_D9, CTX_WHITE);
        train_mcam(EVT_D3, CTX_WHITE, 1); train_mcam(EVT_D5, CTX_WHITE, 2);
        train_mcam(EVT_D7, CTX_WHITE, 3); train_mcam(EVT_D9, CTX_WHITE, 4);

        $display("  [BLACK] 0->3->6->9");
        train_assoc(EVT_D0, EVT_D3); train_assoc(EVT_D3, EVT_D6);
        train_assoc(EVT_D6, EVT_D9);
        train_gate(EVT_D0, CTX_BLACK); train_gate(EVT_D3, CTX_BLACK);
        train_gate(EVT_D6, CTX_BLACK); train_gate(EVT_D9, CTX_BLACK);
        train_mcam(EVT_D0, CTX_BLACK, 1); train_mcam(EVT_D3, CTX_BLACK, 2);
        train_mcam(EVT_D6, CTX_BLACK, 3); train_mcam(EVT_D9, CTX_BLACK, 4);

        $display("  [YELLOW] 2->4->6->8");
        train_assoc(EVT_D2, EVT_D4); train_assoc(EVT_D4, EVT_D6);
        train_assoc(EVT_D6, EVT_D8);
        train_gate(EVT_D2, CTX_YELLOW); train_gate(EVT_D4, CTX_YELLOW);
        train_gate(EVT_D6, CTX_YELLOW); train_gate(EVT_D8, CTX_YELLOW);
        train_mcam(EVT_D2, CTX_YELLOW, 1); train_mcam(EVT_D4, CTX_YELLOW, 2);
        train_mcam(EVT_D6, CTX_YELLOW, 3); train_mcam(EVT_D8, CTX_YELLOW, 4);

        $display("  [CYAN] 8->6->4->2->0");
        train_assoc(EVT_D8, EVT_D6); train_assoc(EVT_D6, EVT_D4);
        train_assoc(EVT_D4, EVT_D2); train_assoc(EVT_D2, EVT_D0);
        train_gate(EVT_D8, CTX_CYAN); train_gate(EVT_D6, CTX_CYAN);
        train_gate(EVT_D4, CTX_CYAN); train_gate(EVT_D2, CTX_CYAN);
        train_gate(EVT_D0, CTX_CYAN);
        train_mcam(EVT_D8, CTX_CYAN, 1); train_mcam(EVT_D6, CTX_CYAN, 2);
        train_mcam(EVT_D4, CTX_CYAN, 3); train_mcam(EVT_D2, CTX_CYAN, 4);
        train_mcam(EVT_D0, CTX_CYAN, 5);

        repeat(20) @(posedge axi_aclk);

        $display("\n---= Inference Tests ---");

        $display("\n[T1] Blue context - D0 (sequence start)");
        infer(ctx_hv[CTX_BLUE], evt_hv[EVT_D0], "Blue/D0");
        dump_active();
        chk("T1", EVT_D0, VP,  "Present");
        chk("T1", EVT_D1, VF1, "Future-1 (D1)");
        chk("T1", EVT_D2, VF2, "Future-2 (D2)");
        chk_order("T1", EVT_D2, EVT_D1, EVT_D0);
        repeat(20) @(posedge axi_aclk);

        $display("\n[T2] Red context - D9 (descending odd, start)");
        infer(ctx_hv[CTX_RED], evt_hv[EVT_D9], "Red/D9");
        dump_active();
        chk("T2", EVT_D9, VP,  "Present");
        chk("T2", EVT_D7, VF1, "Future-1 (D7)");
        chk("T2", EVT_D5, VF2, "Future-2 (D5)");
        chk_order("T2", EVT_D5, EVT_D7, EVT_D9);
        repeat(20) @(posedge axi_aclk);

        $display("\n[T3] Green context - D0 (even ascending, start)");
        infer(ctx_hv[CTX_GREEN], evt_hv[EVT_D0], "Green/D0");
        dump_active();
        chk("T3", EVT_D0, VP,  "Present");
        chk("T3", EVT_D2, VF1, "Future-1 (D2)");
        chk("T3", EVT_D4, VF2, "Future-2 (D4)");
        chk_order("T3", EVT_D4, EVT_D2, EVT_D0);
        repeat(20) @(posedge axi_aclk);

        $display("\n[T4] Magenta context - D1 (odd ascending, start)");
        infer(ctx_hv[CTX_MAGENTA], evt_hv[EVT_D1], "Mag/D1");
        dump_active();
        chk("T4", EVT_D1, VP,  "Present");
        chk("T4", EVT_D3, VF1, "Future-1 (D3)");
        chk("T4", EVT_D5, VF2, "Future-2 (D5)");
        chk_order("T4", EVT_D5, EVT_D3, EVT_D1);
        repeat(20) @(posedge axi_aclk);

        $display("\n[T5] White context - D3 (shared-edge isolation)");
        infer(ctx_hv[CTX_WHITE], evt_hv[EVT_D3], "White/D3");
        dump_active();
        chk("T5", EVT_D3, VP,  "Present");
        chk("T5", EVT_D5, VF1, "Future-1 (D5)");
        chk("T5", EVT_D7, VF2, "Future-2 (D7)");
        chk_order("T5", EVT_D7, EVT_D5, EVT_D3);
        repeat(20) @(posedge axi_aclk);

        // T6: cue in the middle of Blue; D2 is behind the cue and must stay off
        $display("\n[T6] Sliding window - Blue/D3 (mid-sequence)");
        infer(ctx_hv[CTX_BLUE], evt_hv[EVT_D3], "Blue/D3");
        dump_active();
        chk("T6", EVT_D3, VP,     "Present");
        chk("T6", EVT_D4, VF1,    "Future-1 (D4)");
        chk("T6", EVT_D5, VF2,    "Future-2 (D5)");
        chk("T6", EVT_D2, 8'h00,  "D2 not future in Blue/D3");
        chk_order("T6", EVT_D5, EVT_D4, EVT_D3);
        chk_ts_inactive("T6", EVT_D2, "D2 not placed by CA1 in Blue/D3");
        repeat(20) @(posedge axi_aclk);

        $display("\n[T7] Black context - D0 (multiples-of-3 start)");
        infer(ctx_hv[CTX_BLACK], evt_hv[EVT_D0], "Black/D0");
        dump_active();
        chk("T7", EVT_D0, VP,  "Present");
        chk("T7", EVT_D3, VF1, "Future-1 (D3)");
        chk("T7", EVT_D6, VF2, "Future-2 (D6)");
        chk_order("T7", EVT_D6, EVT_D3, EVT_D0);
        repeat(20) @(posedge axi_aclk);

        $display("\n[T8] Yellow context - D2 (even ascending subset)");
        infer(ctx_hv[CTX_YELLOW], evt_hv[EVT_D2], "Yellow/D2");
        dump_active();
        chk("T8", EVT_D2, VP,  "Present");
        chk("T8", EVT_D4, VF1, "Future-1 (D4)");
        chk("T8", EVT_D6, VF2, "Future-2 (D6)");
        chk_order("T8", EVT_D6, EVT_D4, EVT_D2);
        repeat(20) @(posedge axi_aclk);

        $display("\n[T9] Cyan context - D8 (even descending, start)");
        infer(ctx_hv[CTX_CYAN], evt_hv[EVT_D8], "Cyan/D8");
        dump_active();
        chk("T9", EVT_D8, VP,  "Present");
        chk("T9", EVT_D6, VF1, "Future-1 (D6)");
        chk("T9", EVT_D4, VF2, "Future-2 (D4)");
        chk_order("T9", EVT_D4, EVT_D6, EVT_D8);
        repeat(20) @(posedge axi_aclk);

        // T10: D6 is followed by D8 in Green but by D4 in Cyan
        $display("\n[T10a] Context isolation - Green/D6");
        infer(ctx_hv[CTX_GREEN], evt_hv[EVT_D6], "Isolate:Green/D6");
        dump_active();
        chk("T10a", EVT_D6, VP,     "Present");
        chk("T10a", EVT_D8, VF1,    "Future-1 Green=D8");
        chk("T10a", EVT_D4, 8'h00,  "D4 not future in Green");
        chk_ts_lt("T10a", EVT_D8, EVT_D6, "F1 before Present in CA1");
        chk_ts_inactive("T10a", EVT_D4,   "D4 not placed by CA1 in Green");
        repeat(10) @(posedge axi_aclk);

        $display("\n[T10b] Context isolation - Cyan/D6");
        infer(ctx_hv[CTX_CYAN], evt_hv[EVT_D6], "Isolate:Cyan/D6");
        dump_active();
        chk("T10b", EVT_D6, VP,     "Present");
        chk("T10b", EVT_D4, VF1,    "Future-1 Cyan=D4");
        chk("T10b", EVT_D8, 8'h00,  "D8 not future in Cyan");
        chk_ts_lt("T10b", EVT_D4, EVT_D6, "F1 before Present in CA1");
        chk_ts_inactive("T10b", EVT_D8,   "D8 not placed by CA1 in Cyan");
        repeat(20) @(posedge axi_aclk);

        // T11: D3 is followed by D1 in Red but by D5 in Magenta
        $display("\n[T11a] Context isolation - Red/D3");
        infer(ctx_hv[CTX_RED], evt_hv[EVT_D3], "Isolate:Red/D3");
        dump_active();
        chk("T11a", EVT_D3, VP,     "Present");
        chk("T11a", EVT_D1, VF1,    "Future-1 Red=D1");
        chk("T11a", EVT_D5, 8'h00,  "D5 not future in Red");
        chk_ts_lt("T11a", EVT_D1, EVT_D3, "F1 before Present in CA1");
        chk_ts_inactive("T11a", EVT_D5,   "D5 not placed by CA1 in Red");
        repeat(10) @(posedge axi_aclk);

        $display("\n[T11b] Context isolation - Magenta/D3");
        infer(ctx_hv[CTX_MAGENTA], evt_hv[EVT_D3], "Isolate:Mag/D3");
        dump_active();
        chk("T11b", EVT_D3, VP,     "Present");
        chk("T11b", EVT_D5, VF1,    "Future-1 Magenta=D5");
        chk("T11b", EVT_D1, 8'h00,  "D1 not future in Magenta");
        chk_ts_lt("T11b", EVT_D5, EVT_D3, "F1 before Present in CA1");
        chk_ts_inactive("T11b", EVT_D1,   "D1 not placed by CA1 in Magenta");
        repeat(20) @(posedge axi_aclk);

        $display("\n---------------------------------------------------");
        $display("  RESULTS:  %0d PASS   |   %0d FAIL", pass_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display("  *** ALL CHECKS PASSED ***");
        else begin
            $display("  *** FAILURES DETECTED - review voltage dumps ***");
            $display("  Tip: non-zero decay voltages (not VP/VF1/VF2)");
            $display("       may indicate temporal context leaking.");
        end
        $display("---------------------------------------------------");
        $finish;
    end

    // watchdog, 5 ms
    initial begin
        #5_000_000;
        $display("[FATAL] Simulation watchdog timeout");
        $finish;
    end

endmodule : testbench_hippocampex_7seg_new
