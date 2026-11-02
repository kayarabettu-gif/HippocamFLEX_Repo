// ca1_stage: temporal ordering (stage 3).
// Input is CA3's merged voltage per event. If CA3 tagged no event as present
// (VP = 0x40), the DG/EC event is injected at VP instead.
// A full bitonic sort (descending) ranks every event and the rank becomes its
// timestamp: 0 = furthest future (VF2), then VF1, VP, then fading past.
// Zero-voltage events get all ones (0x3F with CNTR_WIDTH=6) = inactive.
// Timestamps update on the edge where o_valid is high; control_fsm's DONE

module ca1_stage #(
    parameter EVENT_DEPTH = 128,
    parameter CONTEXT_DEPTH = 16,
    parameter VOLTAGE_WIDTH = 8,
    parameter CNTR_WIDTH = 8,          // timestamp width; hippocampex_top passes 6
    parameter MAX_VOLTAGE = 8'hFF
) (
    input  logic                          clk,
    input  logic                          rst_n,

    // from CA3 (poly-synaptic path)
    input  logic [VOLTAGE_WIDTH-1:0]      i_merged_voltages [EVENT_DEPTH],
    input  logic                          i_valid,

    // from DG/EC (mono-synaptic path): the event actually seen
    input  logic [EVENT_DEPTH-1:0]     i_ec_event_onehot,
    input  logic [CONTEXT_DEPTH-1:0]   i_ec_context_onehot,

    output logic [EVENT_DEPTH-1:0]        o_ordered_onehot,   // events that got a rank
    output logic [CNTR_WIDTH-1:0]         o_timestamps [EVENT_DEPTH],
    output logic                          o_mismatch_detected, // not wired out in hippocampex_top
    output logic                          o_valid,
    output logic                          o_ready
);

    // which event did CA3 tag as present (exactly 0x40)?
    logic [EVENT_DEPTH-1:0] ca3_vp_onehot;
    logic                   ca3_has_vp;

    always_comb begin
        ca3_has_vp = 1'b0;
        for (int i = 0; i < EVENT_DEPTH; i++) begin
            ca3_vp_onehot[i] = (i_merged_voltages[i] == 8'h40);
            if (ca3_vp_onehot[i]) ca3_has_vp = 1'b1;
        end
    end
    // flag only when both paths name a present event and they differ
    assign o_mismatch_detected = (|i_ec_event_onehot)
                               && ca3_has_vp
                               && (i_ec_event_onehot != ca3_vp_onehot);

    // no VP from CA3: put the DG/EC event at VP so the ordering
    // VF2 (0xC0) > VF1 (0x80) > VP (0x40) > past still holds
    logic [VOLTAGE_WIDTH-1:0] ca1_input_voltages [EVENT_DEPTH];

    always_comb begin
        for (int i = 0; i < EVENT_DEPTH; i++) begin
            if (!ca3_has_vp && i_ec_event_onehot[i])
                ca1_input_voltages[i] = 8'h40;
            else
                ca1_input_voltages[i] = i_merged_voltages[i];
        end
    end

    // full sort, descending
    logic sort_start, sort_done;
    logic [$clog2(EVENT_DEPTH)-1:0] sorted_addrs [EVENT_DEPTH];
    logic [VOLTAGE_WIDTH-1:0]       sorted_dists [EVENT_DEPTH];

    sort_comparator #(
        .DEPTH(EVENT_DEPTH),
        .DIST_WIDTH(VOLTAGE_WIDTH),
        .ADDR_WIDTH($clog2(EVENT_DEPTH)),
        .TOP_K(EVENT_DEPTH)
    ) u_sort (
        .i_clk(clk),
        .i_rst_n(rst_n),
        .i_start(sort_start),
        .i_dist_(ca1_input_voltages),
        .o_sorted_addrs(sorted_addrs),
        .o_sorted_dists(sorted_dists),
        .o_sorted_one_hot(),           // not needed, built from the ranks below
        .o_done(sort_done),
        .o_ready(o_ready)
    );

    // rank i -> timestamp of the event at position i.
    // sorted_addrs is a permutation, so every event is written each recall.
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            o_timestamps     <= '{default:'0};
            o_ordered_onehot <= '0;
        end else if (sort_done) begin
            for (int i = 0; i < EVENT_DEPTH; i++) begin
                if (sorted_dists[i] != '0) begin
                    o_timestamps[sorted_addrs[i]]     <= CNTR_WIDTH'(i);
                    o_ordered_onehot[sorted_addrs[i]] <= 1'b1;
                end else begin
                    o_timestamps[sorted_addrs[i]]     <= {CNTR_WIDTH{1'b1}};
                    o_ordered_onehot[sorted_addrs[i]] <= 1'b0;
                end
            end
        end
    end

    assign sort_start = i_valid && o_ready;
    assign o_valid    = sort_done;

endmodule
