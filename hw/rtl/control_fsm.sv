// control_fsm: runs one recall through DG/EC -> CA3 -> CA1.
// Each stage gets a one-cycle start when the previous stage reports done.
// TRAIN keeps the pipeline idle while the wrapper writes the memories.
// o_valid pulses for one cycle in DONE; o_ready is high only in IDLE.

`timescale 1ns/1ps

module control_fsm (
    input logic clk,
    input logic rst_n,

    input logic i_train_mode,
    input logic i_infer_start,

    // stage status
    input logic i_dgec_ready,
    input logic i_dgec_done,
    input logic i_ca3_ready,
    input logic i_ca3_done,
    input logic i_ca1_ready,
    input logic i_ca1_done,

    // stage starts
    output logic o_dgec_start,
    output logic o_ca3_start,
    output logic o_ca1_start,
    output logic o_valid,
    output logic o_ready
);

    typedef enum logic [2:0] {
        IDLE,
        TRAIN,
        DG_EC_INFER,
        CA3_INFER,
        CA1_INFER,
        DONE
    } state_t;

    state_t state, next_state;

    always_ff @(posedge clk ) begin
        if (!rst_n) state <= IDLE;
        else        state <= next_state;
    end

    always_comb begin
        next_state = state;
        case (state)
            IDLE: begin
                if (i_train_mode)           next_state = TRAIN;
                else if (i_infer_start)     next_state = DG_EC_INFER;
            end
            TRAIN: begin
                if (!i_train_mode)          next_state = IDLE;
            end
            DG_EC_INFER: begin
                if (i_dgec_done)            next_state = CA3_INFER;
            end
            CA3_INFER: begin
                if (i_ca3_done)             next_state = CA1_INFER;
            end
            CA1_INFER: begin
                if (i_ca1_done)             next_state = DONE;
            end
            DONE: begin
                next_state = IDLE;
            end
            default: next_state = IDLE;
        endcase
    end

    always_comb begin
        o_dgec_start = 1'b0;
        o_ca3_start  = 1'b0;
        o_ca1_start  = 1'b0;
        o_valid      = 1'b0;
        o_ready      = (state == IDLE);

        case (state)
            IDLE: begin
                if (i_infer_start && !i_train_mode && i_dgec_ready) begin
                    o_dgec_start = 1'b1;
                end
            end

            DG_EC_INFER: begin
                if (i_dgec_done && i_ca3_ready) begin
                    o_ca3_start = 1'b1;
                end
            end

            CA3_INFER: begin
                if (i_ca3_done && i_ca1_ready) begin
                    o_ca1_start = 1'b1;
                end
            end

            DONE: begin
                o_valid = 1'b1;
            end
        endcase
    end

    // simulation-only state trace
    // synthesis translate_off
    always @(posedge clk) begin
        if (state != next_state) begin
            case (next_state)
                IDLE:        $display("[DUT Control FSM %0t] -> IDLE", $time);
                TRAIN:       $display("[DUT Control FSM %0t] -> TRAIN", $time);
                DG_EC_INFER: $display("[DUT Control FSM %0t] -> DG_EC_INFER", $time);
                CA3_INFER:   $display("[DUT Control FSM %0t] -> CA3_INFER", $time);
                CA1_INFER:   $display("[DUT Control FSM %0t] -> CA1_INFER", $time);
                DONE:        $display("[DUT Control FSM %0t] -> DONE", $time);
            endcase
        end
    end
    // synthesis translate_on

endmodule
