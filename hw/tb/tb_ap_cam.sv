// tb_ap_cam: checks ap_cam's registered distances against $countones for
// exact, partial, inverted, all-zero and random queries.
`timescale 1ns/1ps

module tb_ap_cam;

    parameter WIDTH = 128;
    parameter DEPTH = 16;
    parameter ADDR_WIDTH = $clog2(DEPTH);
    parameter DIST_WIDTH = 8;
    parameter MAX_MISMATCH = 13;

    logic clk;
    logic rst_n;
    logic we;
    logic [ADDR_WIDTH-1:0] waddr;
    logic [WIDTH-1:0] wdata;
    logic search_en;
    logic [WIDTH-1:0] search_data;
    logic [DIST_WIDTH-1:0] dist_ [DEPTH];

    ap_cam #(
        .WIDTH(WIDTH),
        .DEPTH(DEPTH),
        .MAX_MISMATCH(MAX_MISMATCH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .DIST_WIDTH(DIST_WIDTH)
    ) dut (
        .i_clk(clk),
        .i_rst_n(rst_n),
        .i_we(we),
        .i_waddr(waddr),
        .i_wdata(wdata),
        .i_search_en(search_en),
        .i_search_data(search_data),
        .o_dist(dist_)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 100 MHz
    end

    task reset_dut;
        begin
            rst_n = 0;
            #20;
            rst_n = 1;
            #10;
        end
    endtask

    task write_mem(input [ADDR_WIDTH-1:0] addr, input [WIDTH-1:0] data);
        begin
            we = 1;
            waddr = addr;
            wdata = data;
            #10;
            we = 0;
        end
    endtask

    // search once, then compare every row's distance with exp_dist
    task search_and_check(input [WIDTH-1:0] query, input [DIST_WIDTH-1:0] exp_dist [DEPTH]);
        begin
            search_data = query;
            search_en = 1;
            #10;  // one cycle; the result is registered
            search_en = 0;
            for (int i = 0; i < DEPTH; i++) begin
                assert (dist_[i] == exp_dist[i]) else
                    $fatal(1, "Mismatch at row %d: expected %d, got %d", i, exp_dist[i], dist_[i]);
            end
            $display("Search passed for query: %h", query);
        end
    endtask

    task check_hold;
        begin
            #20;  // only waits; nothing is checked
        end
    endtask

        logic [WIDTH-1:0] mem_data [DEPTH];
        logic [DIST_WIDTH-1:0] exp_dist [DEPTH];
        logic [WIDTH-1:0] partial_query ;
        logic [WIDTH-1:0] full_mismatch ;
        logic [WIDTH-1:0] zero_query ;
        logic [WIDTH-1:0] rand_query;
    initial begin
        // Test 1: after reset every distance reads all ones
        reset_dut;
        for (int i = 0; i < DEPTH; i++) begin
            assert (dist_[i] == {DIST_WIDTH{1'b1}}) else $fatal(1, "Reset dist_ fail at %d", i);
        end
        $display("Reset test passed");

        // Test 2: fill all rows, each a different pattern
        for (int i = 0; i < DEPTH; i++) begin
            mem_data[i] = (1 << i) | (i % 2 ? 'hAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA : 'h55555555555555555555555555555555);
            write_mem(i, mem_data[i]);
        end
        $display("Writes completed");

        // Test 3: search with each stored row
        for (int i = 0; i < DEPTH; i++) begin
            for (int j = 0; j < DEPTH; j++) exp_dist[j] = (i == j) ? 0 : $countones(mem_data[i] ^ mem_data[j]);
            search_and_check(mem_data[i], exp_dist);
        end

        // Test 4: row 0 with its low 13 bits flipped
        partial_query = mem_data[0] ^ (1 << MAX_MISMATCH) - 1;
        for (int j = 0; j < DEPTH; j++) exp_dist[j] = $countones(partial_query ^ mem_data[j]);
        search_and_check(partial_query, exp_dist);

        // Test 5: row 0 inverted
        full_mismatch = ~mem_data[0];
        for (int j = 0; j < DEPTH; j++) exp_dist[j] = $countones(full_mismatch ^ mem_data[j]);
        search_and_check(full_mismatch, exp_dist);

        // Test 6: all-zero query
        zero_query = '0;
        for (int j = 0; j < DEPTH; j++) exp_dist[j] = $countones(zero_query ^ mem_data[j]);
        search_and_check(zero_query, exp_dist);

        check_hold;  // check in the waveform

        // Test 8: 1000 random queries
        repeat (1000) begin
            rand_query = $urandom;  // $urandom is 32 bits, so only the low 32 bits vary
            for (int j = 0; j < DEPTH; j++) exp_dist[j] = $countones(rand_query ^ mem_data[j]);
            search_and_check(rand_query, exp_dist);
        end

        $display("All tests passed...");
        $finish;
    end

endmodule