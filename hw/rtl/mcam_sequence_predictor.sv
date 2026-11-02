// mcam_sequence_predictor (CA3 timing tags)
// mcam_mem holds a time code (0..63) for each (event, context).
// CNTR1 counts up from 0 until it equals the cue's code; that value is t_ref
// and the cue is tagged VP. CNTR2 and CNTR3 then start at t_ref and step every
// 2nd and every 4th cycle. An active event whose code is t_ref+1 is tagged
// VF1; t_ref+2 is tagged VF2. The sweep stops when both are found or 25 counts
// past t_ref, so latency depends on t_ref.

module mcam_sequence_predictor #(
    parameter EVENT_DEPTH = 128,
    parameter CONTEXT_DEPTH = 16,
    parameter CNTR_WIDTH = 6,
    parameter VOLTAGE_WIDTH = 8,
    parameter CNTR2_DIV = 2,
    parameter CNTR3_DIV = 4
) (
    input  logic                          i_clk,
    input  logic                          i_rst_n,

    // recall
    input  logic [EVENT_DEPTH-1:0]        i_activated_events,     // from the crossbar
    input  logic [CONTEXT_DEPTH-1:0]      i_context_onehot,
    input  logic [EVENT_DEPTH-1:0]        i_anchor_event_onehot,  // the cue
    input  logic                          i_start,
    input  logic                          i_crossbar_done,        // not used

    // training
    input  logic                          i_train_mode,
    input  logic                          i_mcam_we,
    input  logic [$clog2(EVENT_DEPTH)-1:0]   i_mcam_event_addr,
    input  logic [$clog2(CONTEXT_DEPTH)-1:0] i_mcam_context_addr,
    input  logic [CNTR_WIDTH-1:0]            i_mcam_data,

    output logic [VOLTAGE_WIDTH-1:0] o_event_voltages [EVENT_DEPTH],
    output logic                          o_done,
    output logic                          o_ready
);

    // tag values: CA1 sorts these, so VF2 > VF1 > VP puts the future first
    localparam logic [VOLTAGE_WIDTH-1:0] VP  = 8'h40;  // present
    localparam logic [VOLTAGE_WIDTH-1:0] VF1 = 8'h80;  // next
    localparam logic [VOLTAGE_WIDTH-1:0] VF2 = 8'hC0;  // one after next

    logic [CNTR_WIDTH-1:0] mcam_mem [EVENT_DEPTH][CONTEXT_DEPTH];

    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            for (int e = 0; e < EVENT_DEPTH; e++) begin
                for (int c = 0; c < CONTEXT_DEPTH; c++) begin
                    mcam_mem[e][c] <= '0;
                end
            end
        end else if (i_train_mode && i_mcam_we) begin
            mcam_mem[i_mcam_event_addr][i_mcam_context_addr] <= i_mcam_data;
        end
    end

    typedef enum logic [2:0] {
        IDLE,
        INIT_CNTR1,
        SWEEP_CNTR1,   // find the cue's code -> t_ref
        INIT_CNTR23,   // load CNTR2/3 with t_ref
        SWEEP_CNTR23,  // look for t_ref+1 and t_ref+2
        COMPLETE
    } state_t;

    state_t state, next_state;

    logic [CNTR_WIDTH-1:0] cntr1;      // every cycle
    logic [CNTR_WIDTH-1:0] cntr2;      // every CNTR2_DIV cycles
    logic [CNTR_WIDTH-1:0] cntr3;      // every CNTR3_DIV cycles

    logic [$clog2(CNTR2_DIV)-1:0] div2;
    logic [$clog2(CNTR3_DIV)-1:0] div3;

    logic cntr1_enable, cntr2_enable, cntr3_enable;
    logic cntr1_found, cntr2_found, cntr3_found;

    logic [CNTR_WIDTH-1:0] t_ref;
    logic [$clog2(EVENT_DEPTH)-1:0]   anchor_idx;
    logic [$clog2(CONTEXT_DEPTH)-1:0] context_idx;
    logic anchor_valid, context_valid;

    // one-hot -> index (lowest set bit)
    always_comb begin
        anchor_idx = '0;
        anchor_valid = 1'b0;
        for (int i = 0; i < EVENT_DEPTH; i++) begin
            if (i_anchor_event_onehot[i] && !anchor_valid) begin
                anchor_idx = i[$clog2(EVENT_DEPTH)-1:0];
                anchor_valid = 1'b1;
            end
        end
    end

    always_comb begin
        context_idx = '0;
        context_valid = 1'b0;
        for (int i = 0; i < CONTEXT_DEPTH; i++) begin
            if (i_context_onehot[i] && !context_valid) begin
                context_idx = i[$clog2(CONTEXT_DEPTH)-1:0];
                context_valid = 1'b1;
            end
        end
    end

    // counters
    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            cntr1 <= '0;
            cntr2 <= '0;
            cntr3 <= '0;
            div2 <= '0;
            div3 <= '0;
        end else begin
            if (state == INIT_CNTR1) begin
                cntr1 <= '0;
            end else if (cntr1_enable && !cntr1_found) begin
                cntr1 <= cntr1 + 1'b1;
            end

            if (state == INIT_CNTR23) begin
                cntr2 <= t_ref;
                div2 <= '0;
            end else if (cntr2_enable && !cntr2_found) begin
                if (div2 == (CNTR2_DIV-1)) begin
                    cntr2 <= cntr2 + 1'b1;
                    div2 <= '0;
                end else begin
                    div2 <= div2 + 1'b1;
                end
            end

            if (state == INIT_CNTR23) begin
                cntr3 <= t_ref;
                div3 <= '0;
            end else if (cntr3_enable && !cntr3_found) begin
                if (div3 == (CNTR3_DIV-1)) begin
                    cntr3 <= cntr3 + 1'b1;
                    div3 <= '0;
                end else begin
                    div3 <= div3 + 1'b1;
                end
            end
        end
    end

    // matching and tagging
    logic [VOLTAGE_WIDTH-1:0] event_voltages_reg [EVENT_DEPTH];

    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            for (int i = 0; i < EVENT_DEPTH; i++) begin
                event_voltages_reg[i] <= 8'h00;
            end
            cntr1_found <= 1'b0;
            cntr2_found <= 1'b0;
            cntr3_found <= 1'b0;
            t_ref <= '0;
        end else begin
            case (state)
                IDLE: begin
                    for (int i = 0; i < EVENT_DEPTH; i++) begin
                        event_voltages_reg[i] <= 8'h00;
                    end
                    cntr1_found <= 1'b0;
                    cntr2_found <= 1'b0;
                    cntr3_found <= 1'b0;
                    t_ref <= '0;
                end

                INIT_CNTR1: begin
                    for (int i = 0; i < EVENT_DEPTH; i++) begin
                        event_voltages_reg[i] <= 8'h00;
                    end
                end

                SWEEP_CNTR1: begin
                    if (anchor_valid && context_valid && !cntr1_found) begin
                        if (cntr1 == mcam_mem[anchor_idx][context_idx]) begin
                            t_ref <= cntr1;
                            cntr1_found <= 1'b1;

                            if (i_activated_events[anchor_idx]) begin
                                event_voltages_reg[anchor_idx] <= VP;
                            end
                        end
                    end
                end

                SWEEP_CNTR23: begin
                    // VF1: active event with code t_ref+1
                    if (!cntr2_found) begin
                        for (int i = 0; i < EVENT_DEPTH; i++) begin
                            if (i_activated_events[i] && context_valid) begin
                                if (cntr2 == mcam_mem[i][context_idx] &&
                                    cntr2 == (t_ref + 1)) begin
                                    event_voltages_reg[i] <= VF1;
                                    cntr2_found <= 1'b1;
                                end
                            end
                        end
                    end

                    // VF2: active event with code t_ref+2
                    if (!cntr3_found) begin
                        for (int i = 0; i < EVENT_DEPTH; i++) begin
                            if (i_activated_events[i] && context_valid) begin
                                if (cntr3 == mcam_mem[i][context_idx] &&
                                    cntr3 == (t_ref + 2)) begin
                                    event_voltages_reg[i] <= VF2;
                                    cntr3_found <= 1'b1;
                                end
                            end
                        end
                    end
                end
            endcase
        end
    end

    always_ff @(posedge i_clk) begin
        if (!i_rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    always_comb begin
        next_state = state;
        cntr1_enable = 1'b0;
        cntr2_enable = 1'b0;
        cntr3_enable = 1'b0;

        case (state)
            IDLE: begin
                if (i_start && !i_train_mode)
                    next_state = INIT_CNTR1;
            end

            INIT_CNTR1: begin
                next_state = SWEEP_CNTR1;
            end

            SWEEP_CNTR1: begin
                cntr1_enable = 1'b1;
                if (cntr1_found || cntr1 == (2**CNTR_WIDTH - 1))
                    next_state = INIT_CNTR23;
            end

            INIT_CNTR23: begin
                next_state = SWEEP_CNTR23;
            end

            SWEEP_CNTR23: begin
                cntr2_enable = 1'b1;
                cntr3_enable = 1'b1;
                // stop when both found, 25 counts past t_ref (about 50 cycles
                // via CNTR2), or close to counter wrap. 7 bits so t_ref + 25 can't wrap.
                if ((cntr2_found && cntr3_found) ||
                    ({1'b0, cntr2} >= {1'b0, t_ref} + 7'd25) ||
                    ({1'b0, cntr3} >= {1'b0, t_ref} + 7'd25) ||
                    (cntr2 >= 6'd62) ||
                    (cntr3 >= 6'd62)) begin
                    next_state = COMPLETE;
                end
            end

            COMPLETE: begin
                next_state = IDLE;
            end

            default: next_state = IDLE;
        endcase
    end

    assign o_event_voltages = event_voltages_reg;
    assign o_done = (state == COMPLETE);
    assign o_ready = (state == IDLE);

endmodule
