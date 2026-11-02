// recurrent_heteroassociative_crossbar (CA3)
// assoc_mem[i][j] = 1: event j can follow event i (shared by all contexts)
// gate_mem[j][k]  = 1: event j is allowed in context k
// On i_start the cue becomes the active set and the gate column for the
// current context is registered. Each following cycle adds every event that
// is linked from the active set and allowed in this context.
// MAX_STEPS=8 gives 7 expansion steps; o_done pulses 9 cycles after i_start.

module recurrent_heteroassociative_crossbar #(
    parameter EVENT_DEPTH = 128,
    parameter CONTEXT_DEPTH = 16,
    parameter EVENT_ADDR_WIDTH = $clog2(EVENT_DEPTH),
    parameter CONTEXT_ADDR_WIDTH = $clog2(CONTEXT_DEPTH),
    parameter MAX_STEPS = 8
) (
    input i_clk,
    input i_rst_n,
    input i_start,                                     // one-cycle pulse
    input [EVENT_DEPTH-1:0] i_event_input_one_hot,     // cue from DG/EC
    input [CONTEXT_DEPTH-1:0] i_context_input_one_hot, // context from DG/EC, held for the whole recall

    // training writes, one bit at a time
    input i_we,
    input i_wtype,                    // 0: assoc_mem, 1: gate_mem
    input [EVENT_ADDR_WIDTH-1:0] i_waddr1,  // assoc: from-event, gate: event
    input [EVENT_ADDR_WIDTH-1:0] i_waddr2_assoc,  // assoc: to-event
    input [CONTEXT_ADDR_WIDTH-1:0] i_waddr2_gate, // gate: context
    input i_wdata,

    output logic [EVENT_DEPTH-1:0] o_activated_events,
    output logic o_done
);

// distributed RAM is requested, but the reset clear means Vivado builds these from flip-flops
(* ram_style = "distributed" *) logic [EVENT_DEPTH-1:0]   assoc_mem [EVENT_DEPTH];
(* ram_style = "distributed" *) logic [CONTEXT_DEPTH-1:0] gate_mem  [EVENT_DEPTH];

always @(posedge i_clk) begin
    if (!i_rst_n) begin
        for (int i = 0; i < EVENT_DEPTH; i++) begin
            assoc_mem[i] <= '0;
            gate_mem[i]  <= '0;
        end
    end else if (i_we) begin
        if (i_wtype == 1'b0) begin
            assoc_mem[i_waddr1][i_waddr2_assoc] <= i_wdata;
        end else begin
            gate_mem[i_waddr1][i_waddr2_gate] <= i_wdata;
        end
    end
end

logic [$clog2(MAX_STEPS+1)-1:0] step;  // 0 = idle
logic [EVENT_DEPTH-1:0] active_events;

// gate lookup for the current context. Registered once at i_start so the
// gate_mem read stays out of the expansion feedback loop (timing).
logic gate_comb [EVENT_DEPTH];
always_comb begin
    for (int j = 0; j < EVENT_DEPTH; j++) begin
        gate_comb[j] = 1'b0;
        for (int k = 0; k < CONTEXT_DEPTH; k++) begin
            gate_comb[j] |= i_context_input_one_hot[k] & gate_mem[j][k];
        end
    end
end

logic gate_results [EVENT_DEPTH];
always_ff @(posedge i_clk) begin
    if (!i_rst_n) begin
        for (int j = 0; j < EVENT_DEPTH; j++)
            gate_results[j] <= 1'b0;
    end else if (i_start) begin
        gate_results <= gate_comb;
    end
end

// next events: linked from any active event AND allowed in this context
logic [EVENT_DEPTH-1:0] new_active;
always_comb begin
    logic pre_column_j;
    for (int j = 0; j < EVENT_DEPTH; j++) begin
        pre_column_j = 1'b0;
        for (int i = 0; i < EVENT_DEPTH; i++) begin
            pre_column_j |= active_events[i] & assoc_mem[i][j];
        end
        new_active[j] = gate_results[j] & pre_column_j;
    end
end

// step 1..MAX_STEPS-1: expand; step MAX_STEPS: pulse o_done and go idle
always @(posedge i_clk) begin
    if (!i_rst_n) begin
        step <= 0;
        active_events <= '0;
        o_done <= 1'b0;
    end else begin
        o_done <= 1'b0;
        if (i_start) begin
            step <= 1;
            active_events <= i_event_input_one_hot;
        end else if (step > 0 && step < MAX_STEPS) begin
            active_events <= active_events | new_active;
            step <= step + 1;
        end else if (step == MAX_STEPS) begin
            step <= 0;
            o_done <= 1'b1;
        end
    end
end

assign o_activated_events = active_events;

endmodule
