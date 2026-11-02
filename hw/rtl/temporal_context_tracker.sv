// temporal_context_tracker: decaying trace of past cues.
// One 8-bit value per event. An active input charges it to MAX_VOLTAGE;
// otherwise it drops by DECAY_STEP per enabled cycle and stops at 0.

module temporal_context_tracker #(
    parameter EVENT_DEPTH = 128,
    parameter VOLTAGE_WIDTH = 8,
    parameter DECAY_STEP = 1,     // ca3_stage passes 4
    parameter MAX_VOLTAGE = 8'hFF
)(
    input  logic                     i_clk,
    input  logic                     i_rst_n,

    // events to charge this cycle (ca3_stage sends only the cue)
    input  logic [EVENT_DEPTH-1:0]   i_activated_events,

    input  logic                     i_enable,

    output logic [VOLTAGE_WIDTH-1:0] o_past_voltages [EVENT_DEPTH]
);

    logic [VOLTAGE_WIDTH-1:0] voltage_regs [EVENT_DEPTH];

    integer i;

    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            for (i = 0; i < EVENT_DEPTH; i++) begin
                voltage_regs[i] <= {VOLTAGE_WIDTH{1'b0}};
            end
        end else if (i_enable) begin
            for (i = 0; i < EVENT_DEPTH; i++) begin
                if (i_activated_events[i]) begin
                    voltage_regs[i] <= MAX_VOLTAGE;
                end else begin
                    if (voltage_regs[i] >= DECAY_STEP) begin
                        voltage_regs[i] <= voltage_regs[i] - DECAY_STEP;
                    end else begin
                        voltage_regs[i] <= {VOLTAGE_WIDTH{1'b0}};
                    end
                end
            end
        end
    end

    assign o_past_voltages = voltage_regs;

endmodule
