// ap_cam: proximity CAM.
// Stores DEPTH hypervectors. On i_search_en it registers the Hamming distance
// from the query to every row in parallel (1 cycle). Lower = closer.
// The match threshold is applied by dg_ec_stage; MAX_MISMATCH is unused here.

module ap_cam #(
    parameter WIDTH = 128,
    parameter DEPTH = 16,
    parameter MAX_MISMATCH = 13,
    parameter ADDR_WIDTH = $clog2(DEPTH),
    parameter DIST_WIDTH = 8                // must hold 0..WIDTH
) (
    input i_clk,
    input i_rst_n,                          // active-low, synchronous
    input logic i_we,
    input logic [ADDR_WIDTH-1:0] i_waddr,
    input logic [WIDTH-1:0] i_wdata,

    input logic i_search_en,
    input logic [WIDTH-1:0] i_search_data,

    output logic [DIST_WIDTH-1:0] o_dist [DEPTH]
);

// stored hypervectors
logic [WIDTH-1:0] mem [DEPTH-1:0];

always @(posedge i_clk ) begin
    if (!i_rst_n) begin
        for (int i = 0; i < DEPTH; i++) begin
            mem[i] <= '0;
        end
    end else if (i_we) begin
        mem[i_waddr] <= i_wdata;
    end
end

// distance = popcount(query ^ row), all rows at once
always_ff @(posedge i_clk ) begin
    if(!i_rst_n) begin
        for (int i = 0; i < DEPTH; i++) begin
            o_dist[i] <= {DIST_WIDTH{1'b1}};  // all ones = no match
        end
    end
    else if (i_search_en) begin
        for (int i = 0; i < DEPTH; i++) begin
            o_dist[i] <= $countones(i_search_data ^ mem[i]);
        end
    end  // otherwise hold; the sorter reads these after the search
end

endmodule
