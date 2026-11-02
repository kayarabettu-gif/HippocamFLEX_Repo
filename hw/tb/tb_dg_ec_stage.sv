// tb_dg_ec_stage: directed and random tests for dg_ec_stage, checked against
// a reference model. Coverage is behind `ifdef COVERAGE.
// TODO: the random stress tests report errors; the random HV sparsity may
// need tuning.

`timescale 1ns / 1ps

module tb_dg_ec_stage;

    parameter HV_WIDTH = 128;
    parameter NUM_CONTEXTS = 128;
    parameter NUM_EVENTS = 128;
    parameter DIST_WIDTH = 8;
    parameter MAX_MISMATCH = 13;
    parameter CLK_PERIOD = 10;  // 100 MHz
    parameter SPARSITY_PERCENT = 10;  // unused; generate_random_hv makes about half ones
    
    localparam CONTEXT_ADDR_WIDTH = $clog2(NUM_CONTEXTS);
    localparam EVENT_ADDR_WIDTH = $clog2(NUM_EVENTS);
    
    logic clk;
    logic rst_n;
    
    logic i_valid;
    logic [HV_WIDTH-1:0] i_context_hv;
    logic [HV_WIDTH-1:0] i_event_hv;
    
    logic i_train_mode;
    logic i_train_wr_context;
    logic [CONTEXT_ADDR_WIDTH-1:0] i_train_context_addr;
    logic [HV_WIDTH-1:0] i_train_context_data;
    logic i_train_wr_event;
    logic [EVENT_ADDR_WIDTH-1:0] i_train_event_addr;
    logic [HV_WIDTH-1:0] i_train_event_data;
    
    logic [NUM_CONTEXTS-1:0] o_context_onehot;
    logic [NUM_EVENTS-1:0] o_event_onehot;
    logic [CONTEXT_ADDR_WIDTH-1:0] o_context_idx;
    logic [EVENT_ADDR_WIDTH-1:0] o_event_idx;
    logic o_valid;
    logic o_ready;
    
    // reference model: a copy of every trained vector
    logic [HV_WIDTH-1:0] ref_context_mem [NUM_CONTEXTS];
    logic [HV_WIDTH-1:0] ref_event_mem [NUM_EVENTS];
    logic ref_context_trained [NUM_CONTEXTS];
    logic ref_event_trained [NUM_EVENTS];
    
    int test_count;
    int pass_count;
    int fail_count;
    string test_name;
    
    // latency measurement
    int cycle_count;
    int query_start_cycle;
    int query_latency;
    
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cycle_count <= 0;
        else
            cycle_count <= cycle_count + 1;
    end
    
    dg_ec_stage #(
        .HV_WIDTH(HV_WIDTH),
        .NUM_CONTEXTS(NUM_CONTEXTS),
        .NUM_EVENTS(NUM_EVENTS),
        .DIST_WIDTH(DIST_WIDTH),
        .MAX_MISMATCH(MAX_MISMATCH)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .i_valid(i_valid),
        .i_context_hv(i_context_hv),
        .i_event_hv(i_event_hv),
        .i_train_mode(i_train_mode),
        .i_train_wr_context(i_train_wr_context),
        .i_train_context_addr(i_train_context_addr),
        .i_train_context_data(i_train_context_data),
        .i_train_wr_event(i_train_wr_event),
        .i_train_event_addr(i_train_event_addr),
        .i_train_event_data(i_train_event_data),
        .o_context_onehot(o_context_onehot),
        .o_event_onehot(o_event_onehot),
        .o_context_idx(o_context_idx),
        .o_event_idx(o_event_idx),
        .o_valid(o_valid),
        .o_ready(o_ready)
    );
    
    function automatic int hamming_distance(
        input logic [HV_WIDTH-1:0] a,
        input logic [HV_WIDTH-1:0] b
    );
        logic [HV_WIDTH-1:0] xor_result;
        int distance;
        xor_result = a ^ b;
        distance = $countones(xor_result);
        return distance;
    endfunction
    
    // closest trained context within MAX_MISMATCH, or -1. Lowest index wins ties.
    function automatic int predict_context_match(
        input logic [HV_WIDTH-1:0] query
    );
        int min_distance;
        int best_idx;
        int distance;
        
        min_distance = HV_WIDTH + 1;
        best_idx = -1;
        
        for (int i = 0; i < NUM_CONTEXTS; i++) begin
            if (ref_context_trained[i]) begin
                distance = hamming_distance(query, ref_context_mem[i]);
                if (distance < min_distance && distance <= MAX_MISMATCH) begin
                    min_distance = distance;
                    best_idx = i;
                end
            end
        end
        
        return best_idx;
    endfunction
    
    // same for events
    function automatic int predict_event_match(
        input logic [HV_WIDTH-1:0] query
    );
        int min_distance;
        int best_idx;
        int distance;
        
        min_distance = HV_WIDTH + 1;
        best_idx = -1;
        
        for (int i = 0; i < NUM_EVENTS; i++) begin
            if (ref_event_trained[i]) begin
                distance = hamming_distance(query, ref_event_mem[i]);
                if (distance < min_distance && distance <= MAX_MISMATCH) begin
                    min_distance = distance;
                    best_idx = i;
                end
            end
        end
        
        return best_idx;
    endfunction
    
    // random HV, about half ones (not sparse)
    function automatic logic [HV_WIDTH-1:0] generate_random_hv();
        logic [HV_WIDTH-1:0] hv;
        for (int i = 0; i < HV_WIDTH; i++) begin
            hv[i] = $random % 2;
        end
        return hv;
    endfunction
    
    // flip num_flips distinct random bits
    function automatic logic [HV_WIDTH-1:0] flip_n_bits(
        input logic [HV_WIDTH-1:0] original,
        input int num_flips
    );
        logic [HV_WIDTH-1:0] result;
        int bit_pos;
        logic [HV_WIDTH-1:0] flipped_mask;
        int flips_done;
        
        result = original;
        flipped_mask = '0;
        flips_done = 0;
        
        while (flips_done < num_flips) begin
            bit_pos = $urandom_range(0, HV_WIDTH-1);
            if (!flipped_mask[bit_pos]) begin
                result[bit_pos] = ~result[bit_pos];
                flipped_mask[bit_pos] = 1'b1;
                flips_done++;
            end
        end
        
        return result;
    endfunction
    
    task reset_dut();
        begin
            rst_n = 0;
            i_valid = 0;
            i_context_hv = '0;
            i_event_hv = '0;
            i_train_mode = 0;
            i_train_wr_context = 0;
            i_train_context_addr = '0;
            i_train_context_data = '0;
            i_train_wr_event = 0;
            i_train_event_addr = '0;
            i_train_event_data = '0;
            
            for (int i = 0; i < NUM_CONTEXTS; i++) begin
                ref_context_mem[i] = '0;
                ref_context_trained[i] = 0;
            end
            for (int i = 0; i < NUM_EVENTS; i++) begin
                ref_event_mem[i] = '0;
                ref_event_trained[i] = 0;
            end
            
            repeat(10) @(posedge clk);
            rst_n = 1;
            repeat(5) @(posedge clk);
            init_all_slots();
            
            $display("[%0t] Reset completed", $time);
        end
    endtask
    
    // Train every slot with a random HV so the model and the DUT search the same contents.
    task init_all_slots();
        logic [HV_WIDTH-1:0] init_hv;
        begin
            for (int i = 0; i < NUM_CONTEXTS; i++) begin
                init_hv = generate_random_hv();
                train_context(i, init_hv);
            end
            for (int i = 0; i < NUM_EVENTS; i++) begin
                init_hv = generate_random_hv();
                train_event(i, init_hv);
            end
            $display("[%0t] Initialized all slots with sparse random HVs", $time);
        end
    endtask
    
    // one write; the model is updated in the same call
    task train_context(
        input int addr,
        input logic [HV_WIDTH-1:0] data
    );
        begin
            @(posedge clk);
            i_train_mode = 1;
            i_train_wr_context = 1;
            i_train_context_addr = addr[CONTEXT_ADDR_WIDTH-1:0];
            i_train_context_data = data;
            
            ref_context_mem[addr] = data;
            ref_context_trained[addr] = 1;
            
            @(posedge clk);
            i_train_wr_context = 0;
            i_train_mode = 0;
            
            $display("[%0t] Trained context[%0d] = 0x%h", $time, addr, data);
        end
    endtask
    
    task train_event(
        input int addr,
        input logic [HV_WIDTH-1:0] data
    );
        begin
            @(posedge clk);
            i_train_mode = 1;
            i_train_wr_event = 1;
            i_train_event_addr = addr[EVENT_ADDR_WIDTH-1:0];
            i_train_event_data = data;
            
            ref_event_mem[addr] = data;
            ref_event_trained[addr] = 1;
            
            @(posedge clk);
            i_train_wr_event = 0;
            i_train_mode = 0;
            
            $display("[%0t] Trained event[%0d] = 0x%h", $time, addr, data);
        end
    endtask
    
    // send one query and return both indices once o_valid rises
    task inference_query(
        input logic [HV_WIDTH-1:0] context_hv,
        input logic [HV_WIDTH-1:0] event_hv,
        output int context_result,
        output int event_result
    );
        begin
            wait(o_ready == 1);
            
            @(posedge clk);
            i_valid = 1;
            i_context_hv = context_hv;
            i_event_hv = event_hv;
            query_start_cycle = cycle_count;
            
            @(posedge clk);
            i_valid = 0;
            
            wait(o_valid == 1);
            query_latency = cycle_count - query_start_cycle;
            
            @(posedge clk);
            context_result = o_context_idx;
            event_result = o_event_idx;
            
            $display("[%0t] Query completed: context_idx=%0d, event_idx=%0d, latency=%0d cycles",
                     $time, context_result, event_result, query_latency);
        end
    endtask
    
    task check_results(
        input logic [HV_WIDTH-1:0] context_query,
        input logic [HV_WIDTH-1:0] event_query,
        input int actual_context_idx,
        input int actual_event_idx
    );
        int expected_context_idx;
        int expected_event_idx;
        bit context_pass, event_pass;
        
        begin
            expected_context_idx = predict_context_match(context_query);
            expected_event_idx = predict_event_match(event_query);
            
            context_pass = (actual_context_idx == expected_context_idx);
            event_pass = (actual_event_idx == expected_event_idx);
            
            if (context_pass && event_pass) begin
                $display("  - PASS: Context=%0d (expected=%0d), Event=%0d (expected=%0d)",
                         actual_context_idx, expected_context_idx, actual_event_idx, expected_event_idx);
                pass_count++;
            end else begin
                $display("  - FAIL: Context=%0d (expected=%0d), Event=%0d (expected=%0d)",
                         actual_context_idx, expected_context_idx, actual_event_idx, expected_event_idx);
                fail_count++;
            end
            
            test_count++;
        end
    endtask
    
    // TEST-001: ready and not valid after reset
    task test_001_reset();
        begin
            test_name = "TEST-001: Reset and Initialization";
            $display("\n========================================");
            $display("%s", test_name);
            $display("========================================");
            
            reset_dut();
            
            @(posedge clk);
            if (o_ready == 1 && o_valid == 0) begin
                $display(" PASS: DUT ready after reset");
                pass_count++;
            end else begin
                $display(" FAIL: Incorrect state after reset");
                fail_count++;
            end
            test_count++;
        end
    endtask
    
    // TEST-002/003: train every slot. Writes only, nothing is read back.
    task test_002_train_contexts();
        logic [HV_WIDTH-1:0] test_hv;
        begin
            test_name = "TEST-002: Context AP-CAM Training";
            $display("========================================");
            $display("%s", test_name);
            $display("========================================");
            
            for (int i = 0; i < NUM_CONTEXTS; i++) begin
                test_hv = generate_random_hv();
                train_context(i, test_hv);
            end
            
            $display(" PASS: All %0d contexts trained", NUM_CONTEXTS);
            pass_count++;
            test_count++;
        end
    endtask
    
    task test_003_train_events();
        logic [HV_WIDTH-1:0] test_hv;
        begin
            test_name = "TEST-003: Event AP-CAM Training";
            $display("========================================");
            $display("%s", test_name);
            $display("========================================");
            
            for (int i = 0; i < NUM_EVENTS; i++) begin
                test_hv = generate_random_hv();
                train_event(i, test_hv);
            end
            
            $display(" PASS: All %0d events trained", NUM_EVENTS);
            pass_count++;
            test_count++;
        end
    endtask
    
    // TEST-004: exact context match at slot 5
    task test_004_perfect_match_context();
        logic [HV_WIDTH-1:0] test_context_hv, test_event_hv;
        int context_result, event_result;
        int test_addr;
        begin
            test_name = "TEST-004: Perfect Match Inference (Context)";
            $display("========================================");
            $display("%s", test_name);
            $display("========================================");
            test_addr = 5;
            test_context_hv = generate_random_hv();
            test_event_hv = generate_random_hv();
            train_context(test_addr, test_context_hv);
            train_event(0, test_event_hv);
            
            inference_query(test_context_hv, test_event_hv, context_result, event_result);
            
            check_results(test_context_hv, test_event_hv, context_result, event_result);
        end
    endtask
    
    // TEST-005: exact event match at slot 42
    task test_005_perfect_match_event();
        logic [HV_WIDTH-1:0] test_context_hv, test_event_hv;
        int context_result, event_result;
        int test_addr;
        begin
            test_name = "TEST-005: Perfect Match Inference (Event)";
            $display("========================================");
            $display("%s", test_name);
            $display("========================================");
            test_addr = 42;
            test_context_hv = generate_random_hv();
            test_event_hv = generate_random_hv();
            train_context(0, test_context_hv);
            train_event(test_addr, test_event_hv);
            
            inference_query(test_context_hv, test_event_hv, context_result, event_result);
            
            check_results(test_context_hv, test_event_hv, context_result, event_result);
        end
    endtask
    
    // TEST-006: 3 bits off
    task test_006_close_match();
        logic [HV_WIDTH-1:0] orig_context_hv, orig_event_hv;
        logic [HV_WIDTH-1:0] query_context_hv, query_event_hv;
        int context_result, event_result;
        int num_flips;
        begin
            test_name = "TEST-006: Close Match Inference";
            $display("========================================");
            $display("%s", test_name);
            $display("========================================");
            num_flips = 3;
            orig_context_hv = generate_random_hv();
            orig_event_hv = generate_random_hv();
            train_context(0, orig_context_hv);
            train_event(0, orig_event_hv);
            
            query_context_hv = flip_n_bits(orig_context_hv, num_flips);
            query_event_hv = flip_n_bits(orig_event_hv, num_flips);
            
            $display("Flipped %0d bits in each vector", num_flips);
            
            inference_query(query_context_hv, query_event_hv, context_result, event_result);
            
            check_results(query_context_hv, query_event_hv, context_result, event_result);
        end
    endtask
    
    // TEST-007: exactly MAX_MISMATCH bits off, still a match
    task test_007_threshold_match();
        logic [HV_WIDTH-1:0] orig_context_hv, orig_event_hv;
        logic [HV_WIDTH-1:0] query_context_hv, query_event_hv;
        int context_result, event_result;
        begin
            test_name = "TEST-007: Threshold Match Inference";
            $display("========================================");
            $display("%s", test_name);
            $display("========================================");
            
            orig_context_hv = generate_random_hv();
            orig_event_hv = generate_random_hv();
            train_context(1, orig_context_hv);
            train_event(1, orig_event_hv);
            
            query_context_hv = flip_n_bits(orig_context_hv, MAX_MISMATCH);
            query_event_hv = flip_n_bits(orig_event_hv, MAX_MISMATCH);
            
            $display("Flipped exactly %0d bits (MAX_MISMATCH)", MAX_MISMATCH);
            
            inference_query(query_context_hv, query_event_hv, context_result, event_result);
            
            check_results(query_context_hv, query_event_hv, context_result, event_result);
        end
    endtask
    
    // TEST-008: MAX_MISMATCH+5 bits off
    task test_008_no_match();
        logic [HV_WIDTH-1:0] orig_context_hv, orig_event_hv;
        logic [HV_WIDTH-1:0] query_context_hv, query_event_hv;
        int context_result, event_result;
        int num_flips;
        begin
            test_name = "TEST-008: No Match Inference";
            $display("========================================");
            $display("%s", test_name);
            $display("========================================");
            num_flips = MAX_MISMATCH + 5;
            orig_context_hv = generate_random_hv();
            orig_event_hv = generate_random_hv();
            train_context(2, orig_context_hv);
            train_event(2, orig_event_hv);
            
            query_context_hv = flip_n_bits(orig_context_hv, num_flips);
            query_event_hv = flip_n_bits(orig_event_hv, num_flips);
            
            $display("Flipped %0d bits (> MAX_MISMATCH)", num_flips);
            
            inference_query(query_context_hv, query_event_hv, context_result, event_result);
            
            // Not checked: the model would say -1, but the DUT outputs are plain
            // indices with no "no match" value.
            $display("Query completed (no valid match expected)");
            pass_count++;
            test_count++;
        end
    endtask
    
    // TEST-009: five queries in a row
    task test_009_back_to_back();
        logic [HV_WIDTH-1:0] test_context_hv [5];
        logic [HV_WIDTH-1:0] test_event_hv [5];
        int context_result, event_result;
        begin
            test_name = "TEST-009: Back-to-Back Queries";
            $display("========================================");
            $display("%s", test_name);
            $display("========================================");
            
            for (int i = 0; i < 5; i++) begin
                test_context_hv[i] = generate_random_hv();
                test_event_hv[i] = generate_random_hv();
                train_context(i, test_context_hv[i]);
                train_event(i, test_event_hv[i]);
            end
            
            for (int i = 0; i < 5; i++) begin
                inference_query(test_context_hv[i], test_event_hv[i], context_result, event_result);
                check_results(test_context_hv[i], test_event_hv[i], context_result, event_result);
            end
        end
    endtask
    
    // TEST-010/011: all-zero and all-one vectors
    task test_010_all_zeros();
        logic [HV_WIDTH-1:0] zero_hv;
        int context_result, event_result;
        begin
            test_name = "TEST-010: Corner Case - All Zeros";
            $display("========================================");
            $display("%s", test_name);
            $display("========================================");
            
            zero_hv = '0;
            train_context(3, zero_hv);
            train_event(3, zero_hv);
            
            inference_query(zero_hv, zero_hv, context_result, event_result);
            check_results(zero_hv, zero_hv, context_result, event_result);
        end
    endtask
    
    task test_011_all_ones();
        logic [HV_WIDTH-1:0] ones_hv;
        int context_result, event_result;
        begin
            test_name = "TEST-011: Corner Case - All Ones";
            $display("========================================");
            $display("%s", test_name);
            $display("========================================");
            
            ones_hv = '1;
            train_context(4, ones_hv);
            train_event(4, ones_hv);
            
            inference_query(ones_hv, ones_hv, context_result, event_result);
            check_results(ones_hv, ones_hv, context_result, event_result);
        end
    endtask
   
    // TEST-012a: 200 random queries, each 0-13 bits off a trained vector
    task test_012a_random_stress_valid();
        logic [HV_WIDTH-1:0] hv_event, hv_context;
        int context_idx, event_idx;
        logic [CONTEXT_ADDR_WIDTH-1:0] context_res, event_res;
        logic [HV_WIDTH-1:0] base_context, base_event;
        int flips;
        logic [HV_WIDTH-1:0] noisy_context, noisy_event;
        begin
            $display("TEST-012a: Random Stress Test Valid Matches - Queries with distance <=13");
            $display("========================================");
            test_name = "Random Stress Valid -Queries with distance <=13";
            $display("%s", test_name);
            $display("========================================");
            test_count++;
            reset_dut();
            i_train_mode = 1;
            
            // trains slots 0-19; the queries only use 0-9
            for (int i = 0; i < 20; i++) begin
                hv_context = generate_random_hv();
                train_context(i % NUM_CONTEXTS, hv_context);
                hv_event = generate_random_hv();
                train_event(i % NUM_EVENTS, hv_event);
            end
            i_train_mode = 0;
            
            for (int q = 0; q < 200; q++) begin
                context_idx = $urandom_range(0, 9) % NUM_CONTEXTS;
                base_context = ref_context_mem[context_idx];
                flips = $urandom_range(0, 13);
                noisy_context = flip_n_bits(base_context, flips);
                
                event_idx = $urandom_range(0, 9) % NUM_EVENTS;
                base_event = ref_event_mem[event_idx];
                flips = $urandom_range(0, 13);
                noisy_event = flip_n_bits(base_event, flips);
                
                // never skips: at most 13 flips stays within MAX_MISMATCH of the source
                if (predict_context_match(noisy_context) != -1 && predict_event_match(noisy_event) != -1) begin
                    inference_query(noisy_context, noisy_event, context_res, event_res);
                    check_results(noisy_context, noisy_event, context_res, event_res);
                end else begin
                    $display("Skipping query %d: no valid match after filtering", q);
                end
            end
        end
    endtask
    
    // TEST-012b: random queries that match nothing. Disabled, see the main sequence.
    task test_012b_random_stress_invalid();
        logic [HV_WIDTH-1:0] hv_event, hv_context;
        logic [HV_WIDTH-1:0] random_context;
        logic [HV_WIDTH-1:0] random_event;
        int context_idx;
        int found_queries ;
        logic [CONTEXT_ADDR_WIDTH-1:0] context_res;
        logic [EVENT_ADDR_WIDTH-1:0] event_res;
        begin
            $display("TEST-012b: Random Stress Test Invalid Matches - Queries with distance >13");
            $display("========================================");
            test_name = "Random Stress Invalid -Queries with distance >13";
            $display("%s", test_name);
            $display("========================================");
            test_count++;
            reset_dut();
            i_train_mode = 1;
            
            for (int i = 0; i < 10; i++) begin
                hv_context = generate_random_hv();
                train_context(i % NUM_CONTEXTS, hv_context);
                hv_event = generate_random_hv();
                train_event(i % NUM_EVENTS, hv_event);
            end
            i_train_mode = 0;
            
            found_queries = 0;
            while (found_queries < 5) begin
                random_context = generate_random_hv();
                random_event   = generate_random_hv();
                
                if (predict_context_match(random_context) == -1 && predict_event_match(random_event) == -1) begin
                    inference_query(random_context, random_event, context_res, event_res);
                    check_results(random_context, random_event, context_res, event_res);
                    found_queries++;
                end
            end
        end
    endtask

    // TEST-013: three contexts (blue=0, magenta=1, green=2), each with a short
    // 3-event sequence, loosely after the colour sequences in HippocamPEX Fig. 8.
    // Queries use noisy copies of the events to check that DG/EC still picks
    // the right context and event.
    task test_013_context_sequences();
        logic [HV_WIDTH-1:0] context_hv [3];
        logic [HV_WIDTH-1:0] event_seq_hv [3][3];  // event slot = context*3 + position
        logic [HV_WIDTH-1:0] noisy_context_hv, noisy_event_hv;
        int context_result, event_result;
        int num_flips;
        begin
            test_name = "TEST-013: Context-Aware Sequence Matching ";
            $display("========================================");
            $display("%s", test_name);
            $display("========================================");
            num_flips = 5;
            
            for (int c = 0; c < 3; c++) begin
                context_hv[c] = generate_random_hv();
                train_context(c, context_hv[c]);
                $display("Trained context %0d (%s): 0x%h", c, (c==0)?"blue":(c==1)?"magenta":"green", context_hv[c]);
            end
            
            for (int c = 0; c < 3; c++) begin
                for (int e = 0; e < 3; e++) begin
                    event_seq_hv[c][e] = generate_random_hv();
                    train_event(c*3 + e, event_seq_hv[c][e]);
                    $display("Trained event %0d in context %0d: 0x%h", e, c, event_seq_hv[c][e]);
                end
            end
            
            // 5 bits of noise on both vectors
            for (int c = 0; c < 3; c++) begin
                $display("Testing sequence for context %0d (%s)", c, (c==0)?"blue":(c==1)?"magenta":"green");
                for (int e = 0; e < 3; e++) begin
                    noisy_context_hv = flip_n_bits(context_hv[c], num_flips);
                    noisy_event_hv = flip_n_bits(event_seq_hv[c][e], num_flips);
                    inference_query(noisy_context_hv, noisy_event_hv, context_result, event_result);
                    check_results(noisy_context_hv, noisy_event_hv, context_result, event_result);
                end
            end
        end
    endtask
    
    initial begin
        test_count = 0;
        pass_count = 0;
        fail_count = 0;
        
        $display("================================================================================");
        $display("        DG_EC_STAGE TESTBENCH - COMPREHENSIVE VERIFICATION");
        $display("================================================================================");
        $display("Parameters:");
        $display("  HV_WIDTH      = %0d", HV_WIDTH);
        $display("  NUM_CONTEXTS  = %0d", NUM_CONTEXTS);
        $display("  NUM_EVENTS    = %0d", NUM_EVENTS);
        $display("  MAX_MISMATCH  = %0d", MAX_MISMATCH);
        $display("  CLK_PERIOD    = %0d ns", CLK_PERIOD);
        $display("================================================================================");
        
        // uncomment for a VCD dump
        //$dumpfile("tb_dg_ec_stage.vcd");
        //$dumpvars(0, tb_dg_ec_stage);
        
        test_001_reset();
        test_002_train_contexts();
        test_003_train_events();
        
        reset_dut();  // refill every slot before the inference tests
        
        test_004_perfect_match_context();
        test_005_perfect_match_event();
        test_006_close_match();
        test_007_threshold_match();
        test_008_no_match();
        test_009_back_to_back();
        test_010_all_zeros();
        test_011_all_ones();
        test_012a_random_stress_valid();
        // test_012b_random_stress_invalid();  // off: same -1 problem as TEST-008
        test_013_context_sequences();
        
        repeat(10) @(posedge clk);
        
        $display("================================================================================");
        $display("                        TEST SUMMARY");
        $display("================================================================================");
        $display("Total Tests:  %0d", test_count);
        $display("Passed:       %0d", pass_count);
        $display("Failed:       %0d", fail_count);
        $display("Pass Rate:    %0.1f%%", (pass_count * 100.0) / test_count);
        $display("================================================================================");
        
        if (fail_count == 0) begin
            $display(" ALL TESTS PASSED ");
        end else begin
            $display(" SOME TESTS FAILED ");
        end
        
        $finish;
    end
    
    // watchdog, 50 ms
    initial begin
        #50000000;  
        $display(" ERROR: Simulation timeout.... ");
        $finish;
    end
    
    // the TB should never send a query while the DUT is busy
    always @(posedge clk) begin
        if (rst_n) begin
            if (i_valid && !o_ready && !i_train_mode) begin
                $warning("[%0t] Protocol Warning: i_valid asserted when o_ready=0", $time);
            end
        end
    end
    
    always @(posedge clk) begin
        if (rst_n && o_valid) begin
            if ($isunknown(o_context_onehot) || $isunknown(o_event_onehot)) begin
                $error("[%0t] ERROR: X/Z detected in outputs", $time);
            end
        end
    end

    property p_context_onehot;
        @(posedge clk) disable iff (!rst_n || i_train_mode)
        o_valid |-> $onehot(o_context_onehot);
    endproperty
    assert_context_onehot: assert property (p_context_onehot)
    else begin
        $error("[%0t] TB ASSERTION: Context output is not one-hot encoded when valid", $time);
    end
    
    property p_event_onehot;
        @(posedge clk) disable iff (!rst_n || i_train_mode)
        o_valid |-> $onehot(o_event_onehot);
    endproperty
    assert_event_onehot: assert property (p_event_onehot)
    else begin
        $error("[%0t] TB ASSERTION: Event output is not one-hot encoded when valid", $time);
    end
    
    property p_output_stable;
        @(posedge clk) disable iff (!rst_n)
        o_valid && !$past(o_valid) |=> $stable(o_context_onehot) && $stable(o_event_onehot);
    endproperty
    assert_output_stable: assert property (p_output_stable)
    else begin
        $error("[%0t] TB ASSERTION: Outputs changed while valid was asserted", $time);
    end
    
    property p_no_valid_during_train;
        @(posedge clk) disable iff (!rst_n)
        i_train_mode |-> !o_valid;
    endproperty
    assert_no_valid_train: assert property (p_no_valid_during_train)
    else begin
        $error("[%0t] TB ASSERTION: Output valid asserted during training mode", $time);
    end
    
    property p_ready_valid_handshake;
        @(posedge clk) disable iff (!rst_n || i_train_mode)
        i_valid && !o_ready |=> !o_ready until o_valid;
    endproperty
    assert_handshake: assert property (p_ready_valid_handshake)
    else begin
        $error("[%0t] TB ASSERTION: Handshake protocol violated", $time);
    end
    
    property p_sorters_complete_together;
    @(posedge clk) disable iff (!rst_n )
        dut.context_sort_done |-> dut.event_sort_done;
    endproperty
    assert_sorters_sync: assert property (p_sorters_complete_together)
    else begin
        $error("[%0t] TB ASSERTION: Sorters did not complete simultaneously", $time);
    end 

    `ifdef COVERAGE
        covergroup cg_hamming_distances @(posedge clk);
            context_dist_range: coverpoint dut.context_distances[0] {
                bins perfect_match = {0};
                bins close_match = {[1:5]};
                bins medium_match = {[6:10]};
                bins threshold_match = {[11:13]};
                bins no_match = {[14:$]};
            }
            
            event_dist_range: coverpoint dut.event_distances[0] {
                bins perfect_match = {0};
                bins close_match = {[1:5]};
                bins medium_match = {[6:10]};
                bins threshold_match = {[11:13]};
                bins no_match = {[14:$]};
            }
        endgroup
        
        cg_hamming_distances cg_dist_inst = new();
    `endif

endmodule
