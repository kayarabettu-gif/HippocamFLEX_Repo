// sort_comparator (bitonic version; file name differs from module name)
// Pipelined bitonic sorting network, descending: position 0 holds the largest
// value. One compare-exchange stage per cycle, so latency is
// log2(DEPTH)*(log2(DEPTH)+1)/2 + 2 cycles: 12 for 16, 23 for 64, 30 for 128.
// DEPTH must be a power of two, at most 256. Same ports as the older ramp
// sorter (ramp_sort_comparator), so the stages use it unchanged.
// Unlike the rest of the design this module uses an asynchronous reset.

module sort_comparator #(
    parameter DEPTH      = 16,
    parameter DIST_WIDTH = 8,
    parameter ADDR_WIDTH = $clog2(DEPTH),
    parameter WIDTH      = 128,            // not used
    parameter TOP_K      = 1               // how many winners to mark in o_sorted_one_hot
)(
    input  logic                      i_clk,
    input  logic                      i_rst_n,
    input  logic                      i_start,
    input  logic [DIST_WIDTH-1:0]     i_dist_ [DEPTH],

    output logic [ADDR_WIDTH-1:0]     o_sorted_addrs [DEPTH],
    output logic [DIST_WIDTH-1:0]     o_sorted_dists [DEPTH],
    output logic [DEPTH-1:0]          o_sorted_one_hot,
    output logic                      o_done,
    output logic                      o_ready
);

    // Stage s compares element e with e ^ J_ROM[s]; K_ROM[s] is the block size
    // that sets the direction. Tables cover DEPTH up to 256 (36 stages).
    localparam int LOG_D = $clog2(DEPTH);
    localparam int NUM_STAGES = (LOG_D * (LOG_D + 1)) / 2;

    localparam int K_ROM [36] = '{
        2,
        4, 4,
        8, 8, 8,
        16, 16, 16, 16,
        32, 32, 32, 32, 32,
        64, 64, 64, 64, 64, 64,
        128, 128, 128, 128, 128, 128, 128,
        256, 256, 256, 256, 256, 256, 256, 256
    };

    localparam int J_ROM [36] = '{
        1,
        2, 1,
        4, 2, 1,
        8, 4, 2, 1,
        16, 8, 4, 2, 1,
        32, 16, 8, 4, 2, 1,
        64, 32, 16, 8, 4, 2, 1,
        128, 64, 32, 16, 8, 4, 2, 1
    };

    // value / index / valid at every stage boundary
    logic [DIST_WIDTH-1:0] val [NUM_STAGES+1][DEPTH];
    logic [ADDR_WIDTH-1:0] idx [NUM_STAGES+1][DEPTH];
    logic                  vld [NUM_STAGES+1];

    // stage 0: load inputs with their original index
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            vld[0] <= 1'b0;
            for (int i = 0; i < DEPTH; i++) begin
                val[0][i] <= '0;
                idx[0][i] <= '0;
            end
        end else if (i_start) begin
            vld[0] <= 1'b1;
            for (int i = 0; i < DEPTH; i++) begin
                val[0][i] <= i_dist_[i];
                idx[0][i] <= ADDR_WIDTH'(i);
            end
        end else begin
            vld[0] <= 1'b0;
            // data is don't-care until the next start
        end
    end

    // compare-exchange stages
    genvar s, e;
    generate
        for (s = 0; s < NUM_STAGES; s++) begin : g_stage
            localparam int K = K_ROM[s];
            localparam int J = J_ROM[s];

            always_ff @(posedge i_clk or negedge i_rst_n) begin
                if (!i_rst_n)
                    vld[s+1] <= 1'b0;
                else
                    vld[s+1] <= vld[s];
            end

            for (e = 0; e < DEPTH; e++) begin : g_elem
                localparam int PARTNER = e ^ J;

                localparam bit HAS_PARTNER = (PARTNER < DEPTH);
                localparam bit IS_LO = HAS_PARTNER && (PARTNER > e);
                localparam bit IS_HI = HAS_PARTNER && (PARTNER < e);

                // DIR = 1: this block sorts descending (larger value to the lower index)
                localparam bit DIR = ((e & K) == 0);

                localparam bit PDIR = ((PARTNER & K) == 0);

                if (IS_LO) begin : lo_elem
                    // lower index of the pair
                    always_ff @(posedge i_clk or negedge i_rst_n) begin
                        if (!i_rst_n) begin
                            val[s+1][e] <= '0;
                            idx[s+1][e] <= '0;
                        end else if (vld[s]) begin
                            if (DIR) begin
                                // descending: keep the larger
                                if (val[s][e] >= val[s][PARTNER]) begin
                                    val[s+1][e] <= val[s][e];
                                    idx[s+1][e] <= idx[s][e];
                                end else begin
                                    val[s+1][e] <= val[s][PARTNER];
                                    idx[s+1][e] <= idx[s][PARTNER];
                                end
                            end else begin
                                // ascending: keep the smaller
                                if (val[s][e] <= val[s][PARTNER]) begin
                                    val[s+1][e] <= val[s][e];
                                    idx[s+1][e] <= idx[s][e];
                                end else begin
                                    val[s+1][e] <= val[s][PARTNER];
                                    idx[s+1][e] <= idx[s][PARTNER];
                                end
                            end
                        end else begin
                            val[s+1][e] <= val[s][e];
                            idx[s+1][e] <= idx[s][e];
                        end
                    end
                end
                else if (IS_HI) begin : hi_elem
                    // upper index: takes whatever the lower one did not keep
                    always_ff @(posedge i_clk or negedge i_rst_n) begin
                        if (!i_rst_n) begin
                            val[s+1][e] <= '0;
                            idx[s+1][e] <= '0;
                        end else if (vld[s]) begin
                            if (PDIR) begin
                                if (val[s][e] <= val[s][PARTNER]) begin
                                    val[s+1][e] <= val[s][e];
                                    idx[s+1][e] <= idx[s][e];
                                end else begin
                                    val[s+1][e] <= val[s][PARTNER];
                                    idx[s+1][e] <= idx[s][PARTNER];
                                end
                            end else begin
                                if (val[s][e] >= val[s][PARTNER]) begin
                                    val[s+1][e] <= val[s][e];
                                    idx[s+1][e] <= idx[s][e];
                                end else begin
                                    val[s+1][e] <= val[s][PARTNER];
                                    idx[s+1][e] <= idx[s][PARTNER];
                                end
                            end
                        end else begin
                            val[s+1][e] <= val[s][e];
                            idx[s+1][e] <= idx[s][e];
                        end
                    end
                end
                else begin : no_pair
                    // only reachable if DEPTH is not a power of two
                    always_ff @(posedge i_clk or negedge i_rst_n) begin
                        if (!i_rst_n) begin
                            val[s+1][e] <= '0;
                            idx[s+1][e] <= '0;
                        end else begin
                            val[s+1][e] <= val[s][e];
                            idx[s+1][e] <= idx[s][e];
                        end
                    end
                end
            end
        end
    endgenerate

    // output registers; the one-hot marks the first TOP_K positions
    logic [ADDR_WIDTH-1:0] r_addrs [DEPTH];
    logic [DIST_WIDTH-1:0] r_dists [DEPTH];
    logic [DEPTH-1:0]      r_onehot;
    logic                  r_done;

    logic [DEPTH-1:0] c_onehot;
    always_comb begin
        c_onehot = '0;
        for (int i = 0; i < TOP_K && i < DEPTH; i++) begin
            c_onehot[idx[NUM_STAGES][i]] = 1'b1;
        end
    end

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            r_done   <= 1'b0;
            r_onehot <= '0;
            for (int i = 0; i < DEPTH; i++) begin
                r_addrs[i] <= '0;
                r_dists[i] <= '0;
            end
        end else begin
            r_done <= vld[NUM_STAGES];
            if (vld[NUM_STAGES]) begin
                r_onehot <= c_onehot;
                for (int i = 0; i < DEPTH; i++) begin
                    r_addrs[i] <= idx[NUM_STAGES][i];
                    r_dists[i] <= val[NUM_STAGES][i];
                end
            end
        end
    end

    assign o_sorted_addrs   = r_addrs;
    assign o_sorted_dists   = r_dists;
    assign o_sorted_one_hot = r_onehot;
    assign o_done           = r_done;

    // ready when no sort is in flight
    localparam int PIPE_DEPTH = NUM_STAGES + 2;
    logic [$clog2(PIPE_DEPTH+1)-1:0] inflight;

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            inflight <= '0;
        end else begin
            case ({i_start, r_done})
                2'b10:   inflight <= inflight + 1'd1;
                2'b01:   inflight <= inflight - 1'd1;
                default: inflight <= inflight;
            endcase
        end
    end

    assign o_ready = (inflight == 0);

endmodule
