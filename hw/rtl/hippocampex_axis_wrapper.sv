// hippocampex_axis_wrapper: AXI4-Stream front end for hippocampex_top.
// The PS sends command packets through AXI DMA MM2S into s_axis; responses go
// back on m_axis to DMA S2MM. 32-bit beats, TKEEP all ones, TLAST on the last
// beat of each packet. Hypervectors are sent low word first (hv[31:0] first).
//
// Commands, opcode in beat 0 [31:28]:
//   0x1 train context HV  5 beats  slot in the low bits, then 4 HV beats
//   0x2 train event HV    5 beats  slot in the low bits, then 4 HV beats
//   0x3 crossbar link     1 beat   [20:14] from event, [13:7] to event, [0] bit
//   0x4 context gate      1 beat   [20:14] event, [13:7] context, [0] bit
//   0x5 MCAM time code    1 beat   [20:14] event, [13:7] context, [5:0] code
//   0x6 recall            9 beats  header, 4 beats context HV, 4 beats event HV
//   0x7 status            1 beat
//
// Responses- [31:24]:
//   0xA1 recall result, 1 + NUM_EVENTS/4 + NUM_EVENTS/32 + NUM_EVENTS/4 beats
//        (35 at 64 events):
//          header: [23:17] context idx, [16:10] event idx, [7:0] num_active
//          merged voltages, 4 per beat (event 4k in the low byte)
//          CA1 ordered one-hot, 32 bits per beat
//          CA1 timestamps, 4 per beat, zero-extended to 8 bits
//   0xB1 status, 1 beat, [0] core ready
//   0xC1 training ACK, 1 beat, [3:0] echoed opcode

`timescale 1ns/1ps

module hippocampex_axis_wrapper #(
    parameter AXIS_DATA_WIDTH = 32,
    parameter AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 8,

    // core
    parameter HV_WIDTH       = 128,
    parameter NUM_CONTEXTS   = 16,
    parameter NUM_EVENTS     = 64,
    parameter DIST_WIDTH     = 8,
    parameter MAX_MISMATCH   = 13,
    parameter MAX_STEPS      = 8,
    parameter CNTR_WIDTH     = 6,
    parameter VOLTAGE_WIDTH  = 8,
    parameter DECAY_STEP     = 4,

    // derived, do not override
    parameter CTX_ADDR_W = $clog2(NUM_CONTEXTS),
    parameter EVT_ADDR_W = $clog2(NUM_EVENTS),
    parameter HV_BEATS   = HV_WIDTH / AXIS_DATA_WIDTH,   // 4
    parameter RESP_BEATS = 1 + (NUM_EVENTS/4) + (NUM_EVENTS/32) + (NUM_EVENTS/4)
                                                    // 35 at 64 events; not used in the logic
)(
    input  logic axi_aclk,
    input  logic axi_aresetn,

    input  logic sw_rst_n,          // GPIO bit 0, active-low, resets the core only

    // commands in (from DMA MM2S)
    input  logic [AXIS_DATA_WIDTH-1:0] s_axis_tdata,
    input  logic [AXIS_KEEP_WIDTH-1:0] s_axis_tkeep,
    input  logic                       s_axis_tvalid,
    output logic                       s_axis_tready,
    input  logic                       s_axis_tlast,

    // responses out (to DMA S2MM)
    output logic [AXIS_DATA_WIDTH-1:0] m_axis_tdata,
    output logic [AXIS_KEEP_WIDTH-1:0] m_axis_tkeep,
    output logic                       m_axis_tvalid,
    input  logic                       m_axis_tready,
    output logic                       m_axis_tlast,

    // high while a captured recall result waits to be sent
    output logic irq_result_ready

);

    localparam [3:0] CMD_NOP             = 4'h0;
    localparam [3:0] CMD_TRAIN_CTX       = 4'h1;
    localparam [3:0] CMD_TRAIN_EVT       = 4'h2;
    localparam [3:0] CMD_TRAIN_XBAR_ASSOC= 4'h3;
    localparam [3:0] CMD_TRAIN_XBAR_GATE = 4'h4;
    localparam [3:0] CMD_TRAIN_MCAM      = 4'h5;
    localparam [3:0] CMD_INFER           = 4'h6;
    localparam [3:0] CMD_STATUS          = 4'h7;

    logic clk;
    logic rst_n;
    assign clk   = axi_aclk;
    assign rst_n = axi_aresetn & sw_rst_n;

    // core recall input
    logic                    core_i_valid;
    logic [HV_WIDTH-1:0]    core_i_context_hv;
    logic [HV_WIDTH-1:0]    core_i_event_hv;

    // core training input
    logic                    core_train_mode;
    logic                    core_train_wr_context;
    logic [CTX_ADDR_W-1:0]  core_train_context_addr;
    logic [HV_WIDTH-1:0]    core_train_context_data;
    logic                    core_train_wr_event;
    logic [EVT_ADDR_W-1:0]  core_train_event_addr;
    logic [HV_WIDTH-1:0]    core_train_event_data;
    logic                    core_crossbar_we;
    logic                    core_crossbar_wtype;
    logic [EVT_ADDR_W-1:0]  core_crossbar_waddr1;
    logic [EVT_ADDR_W-1:0]  core_crossbar_waddr2_assoc;
    logic [CTX_ADDR_W-1:0]  core_crossbar_waddr2_gate;
    logic                    core_crossbar_wdata;
    logic                    core_mcam_we;
    logic [EVT_ADDR_W-1:0]  core_mcam_event_addr;
    logic [CTX_ADDR_W-1:0]  core_mcam_context_addr;
    logic [CNTR_WIDTH-1:0]  core_mcam_data;

    // core outputs
    logic [VOLTAGE_WIDTH-1:0] core_event_voltages  [NUM_EVENTS];
    logic [VOLTAGE_WIDTH-1:0] core_past_voltages   [NUM_EVENTS];
    logic [VOLTAGE_WIDTH-1:0] core_merged_voltages [NUM_EVENTS];
    logic [NUM_EVENTS-1:0]    core_ordered_onehot;
    logic [CNTR_WIDTH-1:0]    core_timestamps      [NUM_EVENTS];
    logic [NUM_CONTEXTS-1:0]  core_context_onehot;
    logic [NUM_EVENTS-1:0]    core_event_onehot;
    logic [CTX_ADDR_W-1:0]   core_context_idx;
    logic [EVT_ADDR_W-1:0]   core_event_idx;
    logic                     core_o_valid;
    logic                     core_o_ready;

    hippocampex_top #(
        .HV_WIDTH      (HV_WIDTH),
        .NUM_CONTEXTS  (NUM_CONTEXTS),
        .NUM_EVENTS    (NUM_EVENTS),
        .DIST_WIDTH    (DIST_WIDTH),
        .MAX_MISMATCH  (MAX_MISMATCH),
        .MAX_STEPS     (MAX_STEPS),
        .CNTR_WIDTH    (CNTR_WIDTH),
        .VOLTAGE_WIDTH (VOLTAGE_WIDTH),
        .DECAY_STEP    (DECAY_STEP)
    ) u_hippocampex_core (
        .clk                    (clk),
        .rst_n                  (rst_n),

        .i_valid                (core_i_valid),
        .i_context_hv           (core_i_context_hv),
        .i_event_hv             (core_i_event_hv),

        .i_train_mode           (core_train_mode),
        .i_train_wr_context     (core_train_wr_context),
        .i_train_context_addr   (core_train_context_addr),
        .i_train_context_data   (core_train_context_data),
        .i_train_wr_event       (core_train_wr_event),
        .i_train_event_addr     (core_train_event_addr),
        .i_train_event_data     (core_train_event_data),

        .i_crossbar_we          (core_crossbar_we),
        .i_crossbar_wtype       (core_crossbar_wtype),
        .i_crossbar_waddr1      (core_crossbar_waddr1),
        .i_crossbar_waddr2_assoc(core_crossbar_waddr2_assoc),
        .i_crossbar_waddr2_gate (core_crossbar_waddr2_gate),
        .i_crossbar_wdata       (core_crossbar_wdata),

        .i_mcam_we              (core_mcam_we),
        .i_mcam_event_addr      (core_mcam_event_addr),
        .i_mcam_context_addr    (core_mcam_context_addr),
        .i_mcam_data            (core_mcam_data),

        .o_event_voltages       (core_event_voltages),
        .o_past_voltages        (core_past_voltages),
        .o_merged_voltages      (core_merged_voltages),
        .o_ordered_onehot       (core_ordered_onehot),
        .o_timestamps           (core_timestamps),
        .o_context_onehot       (core_context_onehot),
        .o_event_onehot         (core_event_onehot),
        .o_context_idx          (core_context_idx),
        .o_event_idx            (core_event_idx),
        .o_valid                (core_o_valid),
        .o_ready                (core_o_ready)
    );

    // receive side: decode commands
    typedef enum logic [3:0] {
        RX_IDLE,
        RX_HEADER,           // not used
        RX_HV_DATA,          // 4 HV beats for a training write
        RX_INFER_CTX_HV,     // 4 context HV beats for a recall
        RX_INFER_EVT_HV,     // 4 event HV beats for a recall
        RX_EXEC_TRAIN,       // one-cycle training write
        RX_EXEC_INFER,       // one-cycle recall start
        RX_SEND_ACK,         // not used
        RX_WAIT_CORE_READY   // never entered
    } rx_state_t;

    rx_state_t rx_state, rx_next;

    logic [3:0]            cmd_opcode;
    logic [31:0]           cmd_header;
    logic [HV_WIDTH-1:0]  hv_shift_reg;
    logic [2:0]            beat_cnt;
    logic                  send_ack_pending;
    logic [3:0]            ack_cmd_echo;
    logic                  send_status_pending;
    logic                  infer_launched;

    // high from recall launch until the core reports a result
    logic                  inference_busy;

    // one-cycle pulses into the core
    logic core_i_valid_r;
    logic core_train_wr_context_r;
    logic core_train_wr_event_r;
    logic core_crossbar_we_r;
    logic core_mcam_we_r;
    logic core_train_mode_r;

    // transmit side: serialise responses
    typedef enum logic [2:0] {
        TX_IDLE,
        TX_RESULT_HEADER,
        TX_RESULT_DATA,  // merged voltages, NUM_EVENTS/4 beats
        TX_CA1_ONEHOT,   // ordered one-hot, NUM_EVENTS/32 beats
        TX_CA1_TS,       // timestamps, NUM_EVENTS/4 beats
        TX_STATUS,
        TX_ACK
    } tx_state_t;

    tx_state_t tx_state, tx_next;

    // tready: training and status commands are always taken in IDLE; a recall
    // command only when the core is idle and no recall is in flight
    logic can_accept_inference;
    assign can_accept_inference = core_o_ready && !inference_busy;

    logic [3:0] peek_cmd;
    assign peek_cmd = s_axis_tdata[31:28];

    // tready is registered so tdata -> opcode decode -> tready is not one long
    // combinational path
    logic s_axis_tready_next;

    always_comb begin
        s_axis_tready_next = 1'b0;
        case (rx_state)
            RX_IDLE: begin
                if (!s_axis_tvalid) begin
                    s_axis_tready_next = 1'b1;
                end else if (peek_cmd == CMD_INFER) begin
                    s_axis_tready_next = can_accept_inference;
                end else begin
                    s_axis_tready_next = 1'b1;
                end
            end
            RX_HV_DATA,
            RX_INFER_CTX_HV,
            RX_INFER_EVT_HV: begin
                s_axis_tready_next = 1'b1;
            end
            RX_WAIT_CORE_READY: begin
                s_axis_tready_next = 1'b0;
            end
            default: begin
                s_axis_tready_next = 1'b0;
            end
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) s_axis_tready <= 1'b0;
        else        s_axis_tready <= s_axis_tready_next;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            inference_busy <= 1'b0;
        end else begin
            if (core_i_valid_r) begin
                inference_busy <= 1'b1;
            end else if (core_o_valid) begin
                inference_busy <= 1'b0;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rx_state              <= RX_IDLE;
            cmd_opcode            <= 4'h0;
            cmd_header            <= 32'h0;
            hv_shift_reg          <= '0;
            beat_cnt              <= 3'd0;
            send_ack_pending      <= 1'b0;
            ack_cmd_echo          <= 4'h0;
            send_status_pending   <= 1'b0;
            infer_launched        <= 1'b0;

            core_i_valid_r            <= 1'b0;
            core_i_context_hv         <= '0;
            core_i_event_hv           <= '0;
            core_train_mode_r         <= 1'b0;
            core_train_wr_context_r   <= 1'b0;
            core_train_context_addr   <= '0;
            core_train_context_data   <= '0;
            core_train_wr_event_r     <= 1'b0;
            core_train_event_addr     <= '0;
            core_train_event_data     <= '0;
            core_crossbar_we_r        <= 1'b0;
            core_crossbar_wtype       <= 1'b0;
            core_crossbar_waddr1      <= '0;
            core_crossbar_waddr2_assoc<= '0;
            core_crossbar_waddr2_gate <= '0;
            core_crossbar_wdata       <= 1'b0;
            core_mcam_we_r            <= 1'b0;
            core_mcam_event_addr      <= '0;
            core_mcam_context_addr    <= '0;
            core_mcam_data            <= '0;
        end else begin
            // pulses default low
            core_i_valid_r          <= 1'b0;
            core_train_wr_context_r <= 1'b0;
            core_train_wr_event_r   <= 1'b0;
            core_crossbar_we_r      <= 1'b0;
            core_mcam_we_r          <= 1'b0;
            send_ack_pending        <= 1'b0;
            send_status_pending     <= 1'b0;

            case (rx_state)
                RX_IDLE: begin
                    core_train_mode_r <= 1'b0;
                    infer_launched    <= 1'b0;
                    if (s_axis_tvalid && s_axis_tready) begin
                        cmd_header <= s_axis_tdata;
                        cmd_opcode <= s_axis_tdata[31:28];

                        case (s_axis_tdata[31:28])
                            CMD_TRAIN_CTX: begin
                                core_train_context_addr <= s_axis_tdata[CTX_ADDR_W-1:0];
                                beat_cnt    <= 3'd0;
                                rx_state    <= RX_HV_DATA;
                            end

                            CMD_TRAIN_EVT: begin
                                core_train_event_addr <= s_axis_tdata[EVT_ADDR_W-1:0];
                                beat_cnt    <= 3'd0;
                                rx_state    <= RX_HV_DATA;
                            end

                            CMD_TRAIN_XBAR_ASSOC: begin
                                core_crossbar_wtype       <= 1'b0;
                                core_crossbar_waddr1      <= s_axis_tdata[20:14];
                                core_crossbar_waddr2_assoc<= s_axis_tdata[13:7];
                                core_crossbar_wdata       <= s_axis_tdata[0];
                                rx_state                  <= RX_EXEC_TRAIN;
                            end

                            CMD_TRAIN_XBAR_GATE: begin
                                core_crossbar_wtype       <= 1'b1;
                                core_crossbar_waddr1      <= s_axis_tdata[20:14];
                                core_crossbar_waddr2_gate <= s_axis_tdata[13:7];
                                core_crossbar_wdata       <= s_axis_tdata[0];
                                rx_state                  <= RX_EXEC_TRAIN;
                            end

                            CMD_TRAIN_MCAM: begin
                                core_mcam_event_addr   <= s_axis_tdata[20:14];
                                core_mcam_context_addr <= s_axis_tdata[13:7];
                                core_mcam_data         <= s_axis_tdata[CNTR_WIDTH-1:0];
                                rx_state               <= RX_EXEC_TRAIN;
                            end

                            CMD_INFER: begin
                                // only reached when tready allowed a recall
                                beat_cnt <= 3'd0;
                                rx_state <= RX_INFER_CTX_HV;
                            end

                            CMD_STATUS: begin
                                send_status_pending <= 1'b1;
                                rx_state            <= RX_IDLE;
                            end

                            default: begin
                                rx_state <= RX_IDLE; // unknown opcode: drop the beat
                            end
                        endcase
                    end
                end

                // training HV, low word first
                RX_HV_DATA: begin
                    if (s_axis_tvalid && s_axis_tready) begin
                        hv_shift_reg[beat_cnt*32 +: 32] <= s_axis_tdata;
                        beat_cnt <= beat_cnt + 1;

                        if (beat_cnt == 3'd3) begin
                            rx_state <= RX_EXEC_TRAIN;
                        end
                    end
                end

                RX_INFER_CTX_HV: begin
                    if (s_axis_tvalid && s_axis_tready) begin
                        core_i_context_hv[beat_cnt*32 +: 32] <= s_axis_tdata;
                        beat_cnt <= beat_cnt + 1;

                        if (beat_cnt == 3'd3) begin
                            beat_cnt <= 3'd0;
                            rx_state <= RX_INFER_EVT_HV;
                        end
                    end
                end

                RX_INFER_EVT_HV: begin
                    if (s_axis_tvalid && s_axis_tready) begin
                        core_i_event_hv[beat_cnt*32 +: 32] <= s_axis_tdata;
                        beat_cnt <= beat_cnt + 1;

                        if (beat_cnt == 3'd3) begin
                            rx_state <= RX_EXEC_INFER;
                        end
                    end
                end

                // one-cycle write into the core, then queue the ACK
                RX_EXEC_TRAIN: begin
                    core_train_mode_r <= 1'b1;

                    case (cmd_opcode)
                        CMD_TRAIN_CTX: begin
                            core_train_wr_context_r <= 1'b1;
                            core_train_context_data <= hv_shift_reg;
                        end
                        CMD_TRAIN_EVT: begin
                            core_train_wr_event_r <= 1'b1;
                            core_train_event_data <= hv_shift_reg;
                        end
                        CMD_TRAIN_XBAR_ASSOC,
                        CMD_TRAIN_XBAR_GATE: begin
                            core_crossbar_we_r <= 1'b1;
                        end
                        CMD_TRAIN_MCAM: begin
                            core_mcam_we_r <= 1'b1;
                        end
                    endcase

                    send_ack_pending <= 1'b1;
                    ack_cmd_echo     <= cmd_opcode;
                    rx_state         <= RX_IDLE;
                end

                RX_EXEC_INFER: begin
                    core_i_valid_r <= 1'b1;
                    infer_launched <= 1'b1;
                    rx_state       <= RX_IDLE;
                end

                RX_WAIT_CORE_READY: begin
                    if (core_o_ready) begin
                        rx_state <= RX_IDLE;
                    end
                end

                default: rx_state <= RX_IDLE;
            endcase
        end
    end

    assign core_i_valid          = core_i_valid_r;
    assign core_train_mode       = core_train_mode_r;
    assign core_train_wr_context = core_train_wr_context_r;
    assign core_train_wr_event   = core_train_wr_event_r;
    assign core_crossbar_we      = core_crossbar_we_r;
    assign core_mcam_we          = core_mcam_we_r;

    // snapshot the result when the core finishes
    logic                     result_captured;
    logic [CTX_ADDR_W-1:0]   cap_context_idx;
    logic [EVT_ADDR_W-1:0]   cap_event_idx;
    logic [VOLTAGE_WIDTH-1:0] cap_merged_voltages [NUM_EVENTS];
    logic [7:0]               cap_num_active;
    logic [NUM_EVENTS-1:0]   cap_ordered_onehot;
    logic [CNTR_WIDTH-1:0]   cap_timestamps [NUM_EVENTS];
    logic                    tx_result_pending;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            result_captured <= 1'b0;
            cap_context_idx <= '0;
            cap_event_idx   <= '0;
            cap_num_active  <= 8'd0;
        end else if (core_o_valid) begin
            result_captured <= 1'b1;
            cap_context_idx <= core_context_idx;
            cap_event_idx   <= core_event_idx;

            // Known bug: every nonblocking update below reads the old value,
            // so this adds 1 per result instead of counting active events.
            cap_num_active <= 8'd0;
            for (int i = 0; i < NUM_EVENTS; i++) begin
                cap_merged_voltages[i] <= core_merged_voltages[i];
                if (core_merged_voltages[i] != 0)
                    cap_num_active <= cap_num_active + 1;
            end

            cap_ordered_onehot <= core_ordered_onehot;
            for (int i = 0; i < NUM_EVENTS; i++)
                cap_timestamps[i] <= core_timestamps[i];
        end else if (tx_state == TX_IDLE && !tx_result_pending) begin
            result_captured <= 1'b0;
        end
    end

    logic [5:0] tx_beat_cnt;        // beat within the current section
    logic       tx_status_pending_q;
    logic       tx_ack_pending_q;
    logic [3:0] tx_ack_echo_q;

    assign irq_result_ready = result_captured & (tx_state == TX_IDLE);

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            tx_state           <= TX_IDLE;
            tx_beat_cnt        <= 6'd0;
            tx_result_pending  <= 1'b0;
            tx_status_pending_q<= 1'b0;
            tx_ack_pending_q   <= 1'b0;
            tx_ack_echo_q      <= 4'h0;
        end else begin
            if (result_captured && tx_state == TX_IDLE)
                tx_result_pending <= 1'b1;
            if (send_status_pending)
                tx_status_pending_q <= 1'b1;
            if (send_ack_pending) begin
                tx_ack_pending_q <= 1'b1;
                tx_ack_echo_q    <= ack_cmd_echo;
            end

            case (tx_state)
                TX_IDLE: begin
                    // priority: result, then ACK, then status
                    if (tx_result_pending) begin
                        tx_state    <= TX_RESULT_HEADER;
                        tx_beat_cnt <= 6'd0;
                        tx_result_pending <= 1'b0;
                    end else if (tx_ack_pending_q) begin
                        tx_state         <= TX_ACK;
                        tx_ack_pending_q <= 1'b0;
                    end else if (tx_status_pending_q) begin
                        tx_state            <= TX_STATUS;
                        tx_status_pending_q <= 1'b0;
                    end
                end

                TX_RESULT_HEADER: begin
                    if (m_axis_tready) begin
                        tx_state    <= TX_RESULT_DATA;
                        tx_beat_cnt <= 6'd0;
                    end
                end

                TX_RESULT_DATA: begin
                    if (m_axis_tready) begin
                        if (tx_beat_cnt == (NUM_EVENTS/4 - 1)) begin
                            tx_state    <= TX_CA1_ONEHOT;
                            tx_beat_cnt <= 6'd0;
                        end else begin
                            tx_beat_cnt <= tx_beat_cnt + 1;
                        end
                    end
                end

                TX_CA1_ONEHOT: begin
                    if (m_axis_tready) begin
                        if (tx_beat_cnt == (NUM_EVENTS/32 - 1)) begin
                            tx_state    <= TX_CA1_TS;
                            tx_beat_cnt <= 6'd0;
                        end else begin
                            tx_beat_cnt <= tx_beat_cnt + 1;
                        end
                    end
                end

                TX_CA1_TS: begin
                    if (m_axis_tready) begin
                        if (tx_beat_cnt == (NUM_EVENTS/4 - 1)) begin
                            tx_state <= TX_IDLE;
                        end else begin
                            tx_beat_cnt <= tx_beat_cnt + 1;
                        end
                    end
                end

                TX_STATUS: begin
                    if (m_axis_tready)
                        tx_state <= TX_IDLE;
                end

                TX_ACK: begin
                    if (m_axis_tready)
                        tx_state <= TX_IDLE;
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    // beat contents, then one register stage on every m_axis output
    // (adds a cycle, keeps the outputs straight from flops)
    logic [AXIS_DATA_WIDTH-1:0] m_axis_tdata_pre;
    logic [AXIS_KEEP_WIDTH-1:0] m_axis_tkeep_pre;
    logic                       m_axis_tvalid_pre;
    logic                       m_axis_tlast_pre;

    always_comb begin
        m_axis_tdata_pre  = 32'h0;
        m_axis_tvalid_pre = 1'b0;
        m_axis_tlast_pre  = 1'b0;
        m_axis_tkeep_pre  = {AXIS_KEEP_WIDTH{1'b1}};

        case (tx_state)
            TX_RESULT_HEADER: begin
                m_axis_tvalid_pre = 1'b1;
                // {A1, ctx idx (7b), evt idx (7b), 2'b00, num_active}
                m_axis_tdata_pre  = {8'hA1,
                                 {{(7-CTX_ADDR_W){1'b0}}, cap_context_idx},  // [23:17]
                                 {{(7-EVT_ADDR_W){1'b0}}, cap_event_idx},    // [16:10]
                                 2'b00,                                        // [9:8]
                                 cap_num_active};                              // [7:0]
                m_axis_tlast_pre  = 1'b0;
            end

            TX_RESULT_DATA: begin
                m_axis_tvalid_pre = 1'b1;
                m_axis_tdata_pre  = {cap_merged_voltages[tx_beat_cnt*4 + 3],
                                 cap_merged_voltages[tx_beat_cnt*4 + 2],
                                 cap_merged_voltages[tx_beat_cnt*4 + 1],
                                 cap_merged_voltages[tx_beat_cnt*4 + 0]};
                m_axis_tlast_pre  = 1'b0;
            end

            TX_CA1_ONEHOT: begin
                m_axis_tvalid_pre = 1'b1;
                m_axis_tdata_pre  = cap_ordered_onehot[tx_beat_cnt*32 +: 32];
                m_axis_tlast_pre  = 1'b0;
            end

            TX_CA1_TS: begin
                m_axis_tvalid_pre = 1'b1;
                m_axis_tdata_pre  = {
                    {{(8-CNTR_WIDTH){1'b0}}, cap_timestamps[tx_beat_cnt*4 + 3]},
                    {{(8-CNTR_WIDTH){1'b0}}, cap_timestamps[tx_beat_cnt*4 + 2]},
                    {{(8-CNTR_WIDTH){1'b0}}, cap_timestamps[tx_beat_cnt*4 + 1]},
                    {{(8-CNTR_WIDTH){1'b0}}, cap_timestamps[tx_beat_cnt*4 + 0]}};
                m_axis_tlast_pre  = (tx_beat_cnt == (NUM_EVENTS/4 - 1));
            end

            TX_STATUS: begin
                m_axis_tvalid_pre = 1'b1;
                m_axis_tdata_pre  = {8'hB1, 23'd0, core_o_ready};
                m_axis_tlast_pre  = 1'b1;
            end

            TX_ACK: begin
                m_axis_tvalid_pre = 1'b1;
                m_axis_tdata_pre  = {8'hC1, 20'd0, tx_ack_echo_q};
                m_axis_tlast_pre  = 1'b1;
            end

            default: begin
                m_axis_tvalid_pre = 1'b0;
            end
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            m_axis_tdata  <= 32'h0;
            m_axis_tkeep  <= {AXIS_KEEP_WIDTH{1'b1}};
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
        end else begin
            m_axis_tdata  <= m_axis_tdata_pre;
            m_axis_tkeep  <= m_axis_tkeep_pre;
            m_axis_tvalid <= m_axis_tvalid_pre;
            m_axis_tlast  <= m_axis_tlast_pre;
        end
    end

    // simulation-only messages
    // synthesis translate_off
    always @(posedge clk) begin
        if (s_axis_tvalid && s_axis_tready && rx_state == RX_IDLE)
            $display("[AXI4S-Wrapper %0t] RX CMD=0x%01h data=0x%08h",
                     $time, s_axis_tdata[31:28], s_axis_tdata);
        if (core_o_valid)
            $display("[AXI4S-Wrapper %0t] Inference complete: ctx=%0d evt=%0d",
                     $time, core_context_idx, core_event_idx);
        if (m_axis_tvalid && m_axis_tready && m_axis_tlast)
            $display("[AXI4S-Wrapper %0t] TX packet complete", $time);
        if (s_axis_tvalid && !s_axis_tready && rx_state == RX_IDLE)
            $display("[AXI4S-Wrapper %0t] BACK-PRESSURE: core not ready, inference_busy=%b",
                     $time, inference_busy);
    end
    // synthesis translate_on

endmodule : hippocampex_axis_wrapper
