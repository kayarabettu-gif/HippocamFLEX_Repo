// hippocampex_top: HippocamFLEX core.
// control_fsm plus the three stages:
//   dg_ec_stage - recognise the context and the event (top-1 each)
//   ca3_stage   - predict: context-gated expansion + present/next/next tags
//   ca1_stage   - order: rank every event into a timestamp
// Training writes go straight to each stage's memories. A recall runs the
// stages one after another.

`timescale 1ns/1ps

module hippocampex_top #(
    parameter HV_WIDTH = 128,
    parameter NUM_CONTEXTS = 16,
    parameter NUM_EVENTS = 64,
    parameter DIST_WIDTH = 8,
    parameter MAX_MISMATCH = 13,

    parameter MAX_STEPS = 8,
    parameter CNTR_WIDTH = 6,
    parameter VOLTAGE_WIDTH = 8,
    parameter DECAY_STEP = 4
)(
    input logic clk,
    input logic rst_n,

    // recall query
    input logic                  i_valid,
    input logic [HV_WIDTH-1:0]   i_context_hv,
    input logic [HV_WIDTH-1:0]   i_event_hv,

    // training
    input logic i_train_mode,

    // DG/EC CAM writes
    input logic                             i_train_wr_context,
    input logic [$clog2(NUM_CONTEXTS)-1:0]  i_train_context_addr,
    input logic [HV_WIDTH-1:0]              i_train_context_data,
    input logic                             i_train_wr_event,
    input logic [$clog2(NUM_EVENTS)-1:0]    i_train_event_addr,
    input logic [HV_WIDTH-1:0]              i_train_event_data,

    // CA3 crossbar writes (links and context gates)
    input logic                             i_crossbar_we,
    input logic                             i_crossbar_wtype,
    input logic [$clog2(NUM_EVENTS)-1:0]    i_crossbar_waddr1,
    input logic [$clog2(NUM_EVENTS)-1:0]    i_crossbar_waddr2_assoc,
    input logic [$clog2(NUM_CONTEXTS)-1:0]  i_crossbar_waddr2_gate,
    input logic                             i_crossbar_wdata,

    // CA3 MCAM writes (time codes)
    input logic                             i_mcam_we,
    input logic [$clog2(NUM_EVENTS)-1:0]    i_mcam_event_addr,
    input logic [$clog2(NUM_CONTEXTS)-1:0]  i_mcam_context_addr,
    input logic [CNTR_WIDTH-1:0]            i_mcam_data,

    // CA3 outputs
    output logic [VOLTAGE_WIDTH-1:0] o_event_voltages [NUM_EVENTS],
    output logic [VOLTAGE_WIDTH-1:0] o_past_voltages [NUM_EVENTS],
    output logic [VOLTAGE_WIDTH-1:0] o_merged_voltages [NUM_EVENTS],

    // CA1 outputs
    output logic [NUM_EVENTS-1:0]         o_ordered_onehot,
    output logic [CNTR_WIDTH-1:0]         o_timestamps [NUM_EVENTS],  // 0 = furthest future

    // DG/EC result
    output logic [NUM_CONTEXTS-1:0]         o_context_onehot,
    output logic [NUM_EVENTS-1:0]           o_event_onehot,
    output logic [$clog2(NUM_CONTEXTS)-1:0] o_context_idx,
    output logic [$clog2(NUM_EVENTS)-1:0]   o_event_idx,

    output logic o_valid,
    output logic o_ready
);

    // control
    logic fsm_dgec_start;
    logic fsm_ca3_start;
    logic fsm_ca1_start;
    logic fsm_ready;
    logic fsm_valid;

    // DG/EC
    logic [NUM_CONTEXTS-1:0] dgec_context_onehot;
    logic [NUM_EVENTS-1:0] dgec_event_onehot;
    logic [$clog2(NUM_CONTEXTS)-1:0] dgec_context_idx;
    logic [$clog2(NUM_EVENTS)-1:0] dgec_event_idx;
    logic dgec_valid;
    logic dgec_ready;

    // CA3
    logic [VOLTAGE_WIDTH-1:0] ca3_event_voltages [NUM_EVENTS];
    logic [VOLTAGE_WIDTH-1:0] ca3_past_voltages [NUM_EVENTS];
    logic [VOLTAGE_WIDTH-1:0] ca3_merged_voltages [NUM_EVENTS];
    logic ca3_valid;
    logic ca3_ready;

    // CA1
    logic ca1_valid;
    logic ca1_ready;
    logic [NUM_EVENTS-1:0] ca1_ordered_onehot;
    logic [CNTR_WIDTH-1:0] ca1_timestamps [NUM_EVENTS];

    control_fsm u_control_fsm (
        .clk(clk),
        .rst_n(rst_n),

        .i_train_mode(i_train_mode),
        .i_infer_start(i_valid),

        .i_dgec_ready(dgec_ready),
        .i_dgec_done(dgec_valid),
        .i_ca3_ready(ca3_ready),
        .i_ca3_done(ca3_valid),
        .i_ca1_ready(ca1_ready),
        .i_ca1_done(ca1_valid),

        .o_dgec_start(fsm_dgec_start),
        .o_ca3_start(fsm_ca3_start),
        .o_ca1_start(fsm_ca1_start),

        .o_ready(fsm_ready),
        .o_valid(fsm_valid)
    );


    // DG/EC: what am I seeing, and where?
    dg_ec_stage #(
        .HV_WIDTH(HV_WIDTH),
        .NUM_CONTEXTS(NUM_CONTEXTS),
        .NUM_EVENTS(NUM_EVENTS),
        .DIST_WIDTH(DIST_WIDTH),
        .MAX_MISMATCH(MAX_MISMATCH)
    ) u_dg_ec_stage (
        .clk(clk),
        .rst_n(rst_n),

        .i_valid(fsm_dgec_start),
        .i_context_hv(i_context_hv),
        .i_event_hv(i_event_hv),

        .i_train_mode(i_train_mode),
        .i_train_wr_context(i_train_wr_context),
        .i_train_context_addr(i_train_context_addr),
        .i_train_context_data(i_train_context_data),
        .i_train_wr_event(i_train_wr_event),
        .i_train_event_addr(i_train_event_addr),
        .i_train_event_data(i_train_event_data),

        .o_context_onehot(dgec_context_onehot),
        .o_event_onehot(dgec_event_onehot),
        .o_context_idx(dgec_context_idx),
        .o_event_idx(dgec_event_idx),
        .o_valid(dgec_valid),
        .o_ready(dgec_ready)
    );

    // CA3: what comes next?
    ca3_stage #(
        .EVENT_DEPTH(NUM_EVENTS),
        .CONTEXT_DEPTH(NUM_CONTEXTS),
        .HV_WIDTH(HV_WIDTH),
        .MAX_STEPS(MAX_STEPS),
        .CNTR_WIDTH(CNTR_WIDTH),
        .VOLTAGE_WIDTH(VOLTAGE_WIDTH),
        .DECAY_STEP(DECAY_STEP)
    ) u_ca3_stage (
        .clk(clk),
        .rst_n(rst_n),

        .i_event_onehot(dgec_event_onehot),
        .i_context_onehot(dgec_context_onehot),
        .i_valid(fsm_ca3_start),

        .i_train_mode(i_train_mode),
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

        .o_event_voltages(ca3_event_voltages),
        .o_past_voltages(ca3_past_voltages),
        .o_merged_voltages(ca3_merged_voltages),
        .o_valid(ca3_valid),
        .o_ready(ca3_ready)
    );

    // CA1: when did it happen?
    ca1_stage #(
        .EVENT_DEPTH(NUM_EVENTS),
        .CONTEXT_DEPTH(NUM_CONTEXTS),
        .VOLTAGE_WIDTH(VOLTAGE_WIDTH),
        .CNTR_WIDTH(CNTR_WIDTH)
    ) u_ca1_stage (
        .clk(clk),
        .rst_n(rst_n),

        .i_merged_voltages(ca3_merged_voltages),
        .i_valid(fsm_ca1_start),

        // DG/EC result goes straight to CA1 as well
        .i_ec_event_onehot(dgec_event_onehot),
        .i_ec_context_onehot(dgec_context_onehot),

        .o_ordered_onehot(ca1_ordered_onehot),
        .o_timestamps(ca1_timestamps),
        .o_mismatch_detected(),  // not used
        .o_valid(ca1_valid),
        .o_ready(ca1_ready)
    );

    assign o_event_voltages = ca3_event_voltages;
    assign o_past_voltages = ca3_past_voltages;
    assign o_merged_voltages = ca3_merged_voltages;

    assign o_ordered_onehot = ca1_ordered_onehot;
    assign o_timestamps = ca1_timestamps;

    assign o_context_onehot = dgec_context_onehot;
    assign o_event_onehot = dgec_event_onehot;
    assign o_context_idx = dgec_context_idx;
    assign o_event_idx = dgec_event_idx;

    assign o_valid = fsm_valid;
    assign o_ready = fsm_ready;

endmodule : hippocampex_top
