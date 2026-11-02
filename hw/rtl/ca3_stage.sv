// ca3_stage: sequence prediction (stage 2).
//   crossbar  - expands the cue to every event reachable in this context
//   mcam      - tags the cue VP (present) and its next two events VF1/VF2
//   tracker   - decaying trace of past cues; only the cue charges it, so
//               predicted events never show up as past
// o_merged_voltages = MCAM tag where there is one, otherwise the past trace.
// o_valid is MCAM's one-cycle done.

module ca3_stage #(
    parameter EVENT_DEPTH = 128,
    parameter CONTEXT_DEPTH = 16,
    parameter HV_WIDTH = 128,
    parameter MAX_STEPS = 8,
    parameter CNTR_WIDTH = 6,
    parameter VOLTAGE_WIDTH = 8,
    parameter DECAY_STEP = 4  // past-trace drop per cycle
) (
    input                                 clk,
    input                                 rst_n,

    // from DG/EC
    input  logic [EVENT_DEPTH-1:0]        i_event_onehot,
    input  logic [CONTEXT_DEPTH-1:0]      i_context_onehot,
    input  logic                          i_valid,

    // training writes
    input  logic                             i_train_mode,
    input  logic                             i_crossbar_we,
    input  logic                             i_crossbar_wtype,      // 0: link, 1: gate
    input  logic [$clog2(EVENT_DEPTH)-1:0]   i_crossbar_waddr1,
    input  logic [$clog2(EVENT_DEPTH)-1:0]   i_crossbar_waddr2_assoc,
    input  logic [$clog2(CONTEXT_DEPTH)-1:0] i_crossbar_waddr2_gate,
    input  logic                             i_crossbar_wdata,
    input  logic                             i_mcam_we,
    input  logic [$clog2(EVENT_DEPTH)-1:0]   i_mcam_event_addr,
    input  logic [$clog2(CONTEXT_DEPTH)-1:0] i_mcam_context_addr,
    input  logic [CNTR_WIDTH-1:0]            i_mcam_data,

    output logic [VOLTAGE_WIDTH-1:0]      o_event_voltages [EVENT_DEPTH],  // MCAM tags only
    output logic [VOLTAGE_WIDTH-1:0]      o_past_voltages [EVENT_DEPTH],   // past trace only
    output logic [VOLTAGE_WIDTH-1:0]      o_merged_voltages [EVENT_DEPTH], // goes to CA1
    output logic                          o_valid,
    output logic                          o_ready
);

    logic crossbar_start;
    logic mcam_start;
    logic crossbar_done;
    logic [EVENT_DEPTH-1:0] crossbar_activated_events;

    logic mcam_done;
    logic mcam_ready;
    logic [VOLTAGE_WIDTH-1:0] mcam_voltages [EVENT_DEPTH];
    logic [VOLTAGE_WIDTH-1:0] past_voltages [EVENT_DEPTH];

    // not used
    logic inference_active;

    assign inference_active = crossbar_start || !crossbar_done || !mcam_ready;

    // tracker only sees the cue, for one cycle at start
    logic [EVENT_DEPTH-1:0] anchor_for_tracker;
    assign anchor_for_tracker = crossbar_start ? i_event_onehot : '0;

    // MCAM clears its outputs when it goes back to IDLE, so keep a copy
    // from its done until the next recall starts
    logic [VOLTAGE_WIDTH-1:0] held_mcam_voltages [EVENT_DEPTH];
    logic mcam_done_pulse;
    logic mcam_done_d;

    always_ff @(posedge clk ) begin
        if (!rst_n)
            mcam_done_d <= 1'b0;
        else
            mcam_done_d <= mcam_done;
    end
    assign mcam_done_pulse = mcam_done && !mcam_done_d;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < EVENT_DEPTH; i++) begin
                held_mcam_voltages[i] <= 8'h00;
            end
        end else if (mcam_done_pulse) begin
            for (int i = 0; i < EVENT_DEPTH; i++) begin
                held_mcam_voltages[i] <= mcam_voltages[i];
            end
        end else if (crossbar_start) begin
            for (int i = 0; i < EVENT_DEPTH; i++) begin
                held_mcam_voltages[i] <= 8'h00;
            end
        end
    end

    // crossbar starts on the DG/EC result; MCAM starts once the crossbar is done
    assign crossbar_start = i_valid && o_ready && !i_train_mode;
    assign mcam_start     = crossbar_done && !mcam_done && mcam_ready && !i_train_mode;

    recurrent_heteroassociative_crossbar #(
        .EVENT_DEPTH(EVENT_DEPTH),
        .CONTEXT_DEPTH(CONTEXT_DEPTH),
        .MAX_STEPS(MAX_STEPS)
    ) u_crossbar (
        .i_clk(clk),
        .i_rst_n(rst_n),
        .i_start(crossbar_start),
        .i_event_input_one_hot(i_event_onehot),
        .i_context_input_one_hot(i_context_onehot),
        .i_we(i_crossbar_we),
        .i_wtype(i_crossbar_wtype),
        .i_waddr1(i_crossbar_waddr1),
        .i_waddr2_assoc(i_crossbar_waddr2_assoc),
        .i_waddr2_gate(i_crossbar_waddr2_gate),
        .i_wdata(i_crossbar_wdata),
        .o_activated_events(crossbar_activated_events),
        .o_done(crossbar_done)
    );

    mcam_sequence_predictor #(
        .EVENT_DEPTH(EVENT_DEPTH),
        .CONTEXT_DEPTH(CONTEXT_DEPTH),
        .CNTR_WIDTH(CNTR_WIDTH),
        .VOLTAGE_WIDTH(VOLTAGE_WIDTH),
        .CNTR2_DIV(2),       // VF1 counter: every 2nd cycle
        .CNTR3_DIV(4)        // VF2 counter: every 4th cycle
    ) u_mcam_predictor (
        .i_clk(clk),
        .i_rst_n(rst_n),
        .i_activated_events(crossbar_activated_events),
        .i_anchor_event_onehot(i_event_onehot),      // the cue
        .i_context_onehot(i_context_onehot),
        .i_start(mcam_start),
        .i_crossbar_done(crossbar_done),
        .i_train_mode(i_train_mode),
        .i_mcam_we(i_mcam_we),
        .i_mcam_event_addr(i_mcam_event_addr),
        .i_mcam_context_addr(i_mcam_context_addr),
        .i_mcam_data(i_mcam_data),
        .o_event_voltages(mcam_voltages),
        .o_done(mcam_done),
        .o_ready(mcam_ready)
    );

    // decays on every non-training cycle, not per recall
    temporal_context_tracker #(
        .EVENT_DEPTH(EVENT_DEPTH),
        .VOLTAGE_WIDTH(VOLTAGE_WIDTH),
        .DECAY_STEP(DECAY_STEP),
        .MAX_VOLTAGE(8'hFF)
    ) u_tracker (
        .i_clk(clk),
        .i_rst_n(rst_n),
        .i_activated_events(anchor_for_tracker),
        .i_enable(!i_train_mode),
        .o_past_voltages(past_voltages)
    );

    // merge: held MCAM tag, else live MCAM tag (its COMPLETE cycle), else past
    logic [VOLTAGE_WIDTH-1:0] merged_voltages [EVENT_DEPTH];

    always_comb begin
        for (int i = 0; i < EVENT_DEPTH; i++) begin
            if (held_mcam_voltages[i] != 8'h00) begin
                merged_voltages[i] = held_mcam_voltages[i];
            end else if (mcam_voltages[i] != 8'h00) begin
                merged_voltages[i] = mcam_voltages[i];
            end else begin
                merged_voltages[i] = past_voltages[i];
            end
        end
    end

    assign o_event_voltages = held_mcam_voltages;
    assign o_past_voltages  = past_voltages;
    assign o_merged_voltages = merged_voltages;

    assign o_valid = mcam_done;
    assign o_ready = mcam_ready && !i_train_mode;

endmodule
