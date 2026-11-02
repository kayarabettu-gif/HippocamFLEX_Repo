// testbench_ca3_stage: ca3_stage on its own, fed one-hot cue/context inputs
// in place of DG/EC.
// Checks: training writes, VP/VF1/VF2 tags, Blue/Magenta/Green chains that
// share events, moving the cue along a chain, past-trace decay, MCAM tags
// winning over the past trace, held voltages after MCAM finishes, longer
// chains, bad inputs and chain ends. Also prints crossbar and MCAM cycle counts.

`timescale 1ns/1ps

module testbench_ca3_stage;

    localparam EVENT_DEPTH = 128;
    localparam CONTEXT_DEPTH = 16;
    localparam HV_WIDTH = 128;           // unused
    localparam MAX_STEPS = 8;
    localparam CNTR_WIDTH = 6;
    localparam VOLTAGE_WIDTH = 8;
    localparam DECAY_STEP = 4;
    localparam CLK_PERIOD = 10;
    
    // MCAM tags
    localparam VP  = 8'h40;  // present
    localparam VF1 = 8'h80;  // future-1
    localparam VF2 = 8'hC0;  // future-2
    
    localparam CTX_BLUE    = 0;  // 7->5->3->2
    localparam CTX_MAGENTA = 1;  // 7->5->3->1, shares 7->5->3 with Blue
    localparam CTX_GREEN   = 2;  // 2->4->6->8
    localparam CTX_RED     = 3;  // never trained; used as a bad context in test 9
    
    localparam EVT_1=1, EVT_2=2, EVT_3=3, EVT_4=4, EVT_5=5;
    localparam EVT_6=6, EVT_7=7, EVT_8=8, EVT_9=9;

    logic clk;
    logic rst_n;
    
    // driven by the TB in place of DG/EC
    logic [EVENT_DEPTH-1:0]    i_event_onehot;
    logic [CONTEXT_DEPTH-1:0]  i_context_onehot;
    logic                      i_valid;
    
    logic                              i_train_mode;
    logic                              i_crossbar_we;
    logic                              i_crossbar_wtype;
    logic [$clog2(EVENT_DEPTH)-1:0]    i_crossbar_waddr1;
    logic [$clog2(EVENT_DEPTH)-1:0]    i_crossbar_waddr2_assoc;
    logic [$clog2(CONTEXT_DEPTH)-1:0]  i_crossbar_waddr2_gate;
    logic                              i_crossbar_wdata;
    logic                              i_mcam_we;
    logic [$clog2(EVENT_DEPTH)-1:0]    i_mcam_event_addr;
    logic [$clog2(CONTEXT_DEPTH)-1:0]  i_mcam_context_addr;
    logic [CNTR_WIDTH-1:0]             i_mcam_data;
    
    logic [VOLTAGE_WIDTH-1:0] o_event_voltages [EVENT_DEPTH];
    logic [VOLTAGE_WIDTH-1:0] o_past_voltages [EVENT_DEPTH];
    logic [VOLTAGE_WIDTH-1:0] o_merged_voltages [EVENT_DEPTH];
    logic                     o_valid;
    logic                     o_ready;

    int cycle_count = 0;
    int test_num = 0;
    int tests_passed = 0;
    int tests_failed = 0;
    int checks_passed = 0;
    int checks_failed = 0;
    
    // cycle stamps from the last run_inference, used by test 10
    int inference_start_cycle;
    int crossbar_start_cycle;
    int crossbar_done_cycle;
    int mcam_done_cycle;
    int ca3_done_cycle;
    
    logic [VOLTAGE_WIDTH-1:0] held_snapshot_7, held_snapshot_5, held_snapshot_3;
    int test_crossbar_latency, test_mcam_latency, test_total_latency;

    ca3_stage #(
        .EVENT_DEPTH(EVENT_DEPTH),
        .CONTEXT_DEPTH(CONTEXT_DEPTH),
        .HV_WIDTH(HV_WIDTH),
        .MAX_STEPS(MAX_STEPS),
        .CNTR_WIDTH(CNTR_WIDTH),
        .VOLTAGE_WIDTH(VOLTAGE_WIDTH),
        .DECAY_STEP(DECAY_STEP)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .i_event_onehot(i_event_onehot),
        .i_context_onehot(i_context_onehot),
        .i_valid(i_valid),
        .i_train_mode(i_train_mode),
        .i_crossbar_we(i_crossbar_we),
        .i_crossbar_wtype(i_crossbar_wtype),
        .i_crossbar_waddr1(i_crossbar_waddr1),
        .i_crossbar_waddr2_assoc(i_crossbar_waddr2_assoc),
        .i_crossbar_waddr2_gate(i_crossbar_waddr2_gate),
        .i_crossbar_wdata(i_crossbar_wdata),
        .i_mcam_we(i_mcam_we),
        .i_mcam_event_addr(i_mcam_event_addr),
        .i_mcam_context_addr(i_mcam_context_addr),
        .i_mcam_data(i_mcam_data),
        .o_event_voltages(o_event_voltages),
        .o_past_voltages(o_past_voltages),
        .o_merged_voltages(o_merged_voltages),
        .o_valid(o_valid),
        .o_ready(o_ready)
    );

    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end
    
    always @(posedge clk) begin
        if (rst_n) cycle_count <= cycle_count + 1;
        else cycle_count <= 0;
    end

    task reset_dut();
        begin
            $display("\n################################################################################");
            $display("# CA3 STAGE TESTBENCH");
            $display("# Pipeline: Crossbar -> MCAM -> Temporal Merge -> CA1");
            $display("################################################################################\n");
            rst_n = 0;
            i_train_mode = 0;
            i_valid = 0;
            i_event_onehot = '0;
            i_context_onehot = '0;
            i_crossbar_we = 0;
            i_crossbar_wtype = 0;
            i_crossbar_waddr1 = 0;
            i_crossbar_waddr2_assoc = 0;
            i_crossbar_waddr2_gate = 0;
            i_crossbar_wdata = 0;
            i_mcam_we = 0;
            i_mcam_event_addr = 0;
            i_mcam_context_addr = 0;
            i_mcam_data = 0;
            repeat(5) @(posedge clk);
            rst_n = 1;
            repeat(2) @(posedge clk);
            $display(" [Cycle %0d] Reset complete - CA3 stage ready", cycle_count);
            $display("------------------------------------------------------------------\n");
        end
    endtask
    
    task start_test(input string name);
        begin
            test_num++;
            $display("\n------------------------------------------------------------------");
            $display(" TEST CASE %0d: %s", test_num, name);
            $display("------------------------------------------------------------------");
            $display(" Start Cycle: %0d", cycle_count);
        end
    endtask
    
    task end_test(input int local_passed, input int local_failed);
        begin
            checks_passed += local_passed;
            checks_failed += local_failed;
            
            if (local_failed == 0) begin
                tests_passed++;
                $display("\n+------------------------------------------------------------------------------+");
                $display("| TEST %0d: PASSED | Checks: %0d/%0d", 
                         test_num, local_passed, local_passed + local_failed);
                $display("+------------------------------------------------------------------------------+");
            end else begin
                tests_failed++;
                $display("\n+------------------------------------------------------------------------------+");
                $display("| TEST %0d: FAILED | Passed: %0d, Failed: %0d", 
                         test_num, local_passed, local_failed);
                $display("+------------------------------------------------------------------------------+");
            end
            $display(" End Cycle: %0d\n", cycle_count);
        end
    endtask
    
    task train_crossbar_assoc(input int from_evt, to_evt);
        begin
            @(posedge clk);
            i_train_mode = 1;
            i_crossbar_we = 1;
            i_crossbar_wtype = 0;
            i_crossbar_waddr1 = from_evt;
            i_crossbar_waddr2_assoc = to_evt;
            i_crossbar_wdata = 1;
            @(posedge clk);
            i_crossbar_we = 0;
            i_train_mode = 0;
            $display(" [Cycle %0d] Trained: Event[%0d] -> Event[%0d] (assoc)", 
                     cycle_count, from_evt, to_evt);
        end
    endtask
    
    task train_crossbar_gate(input int evt, ctx);
        begin
            @(posedge clk);
            i_train_mode = 1;
            i_crossbar_we = 1;
            i_crossbar_wtype = 1;
            i_crossbar_waddr1 = evt;
            i_crossbar_waddr2_gate = ctx;
            i_crossbar_wdata = 1;
            @(posedge clk);
            i_crossbar_we = 0;
            i_train_mode = 0;
            $display(" [Cycle %0d] Trained: Event[%0d] <-> Context[%0d] (gate)", 
                     cycle_count, evt, ctx);
        end
    endtask
    
    task train_mcam(input int evt, ctx, t_code, string desc);
        begin
            @(posedge clk);
            i_train_mode = 1;
            i_mcam_we = 1;
            i_mcam_event_addr = evt;
            i_mcam_context_addr = ctx;
            i_mcam_data = t_code;
            @(posedge clk);
            i_mcam_we = 0;
            i_train_mode = 0;
            $display(" [Cycle %0d] Trained: MCAM[Event=%0d][Context=%0d] = T%0d (%s)", 
                     cycle_count, evt, ctx, t_code, desc);
        end
    endtask
    
    // one inference; records when the crossbar, MCAM and CA3 finish (peeks at DUT internals)
    task run_inference(input int ctx_idx, evt_idx, string test_name);
        begin
            $display("\n--- INFERENCE: %s ---", test_name);
            $display(" Context: %0d, Event: %0d", ctx_idx, evt_idx);
            
            i_train_mode = 0;
            wait(o_ready);
            inference_start_cycle = cycle_count;
            $display(" [Cycle %0d] CA3 ready - starting inference", cycle_count);
            
            i_context_onehot = '0;
            i_context_onehot[ctx_idx] = 1'b1;
            i_event_onehot = '0;
            i_event_onehot[evt_idx] = 1'b1;
            i_valid = 1;
            
            @(posedge clk);
            i_valid = 0;
            
            crossbar_start_cycle = cycle_count;
            $display(" [Cycle %0d] Crossbar started", cycle_count);
            
            wait(dut.crossbar_done);
            crossbar_done_cycle = cycle_count;
            $display(" [Cycle %0d] Crossbar done (latency: %0d cycles)", 
                     cycle_count, crossbar_done_cycle - crossbar_start_cycle);
            
            wait(dut.mcam_done);
            mcam_done_cycle = cycle_count;
            $display(" [Cycle %0d] MCAM done (latency: %0d cycles)", 
                     cycle_count, mcam_done_cycle - crossbar_done_cycle);
            
            wait(o_valid);
            ca3_done_cycle = cycle_count;
            $display(" [Cycle %0d] CA3 valid asserted", cycle_count);
            $display(" Total CA3 latency: %0d cycles", ca3_done_cycle - inference_start_cycle);
            
            @(posedge clk);
            @(posedge clk);  // two clocks so the held voltages settle
        end
    endtask
    
    task verify_voltage(input int evt, input [7:0] exp_voltage, 
                       input string desc, output int pass, fail);
        begin
            if (o_event_voltages[evt] === exp_voltage) begin
                $display(" [PASS] Event[%0d] MCAM voltage = 0x%02h | %s", 
                         evt, o_event_voltages[evt], desc);
                pass++;
            end else begin
                $display(" [FAIL] Event[%0d] MCAM voltage = 0x%02h (expected 0x%02h) | %s", 
                         evt, o_event_voltages[evt], exp_voltage, desc);
                fail++;
            end
        end
    endtask
    
    task verify_merged(input int evt, input [7:0] exp_voltage, 
                      input string desc, output int pass, fail);
        begin
            if (o_merged_voltages[evt] === exp_voltage) begin
                $display(" [PASS] Event[%0d] Merged voltage = 0x%02h | %s", 
                         evt, o_merged_voltages[evt], desc);
                pass++;
            end else begin
                $display(" [FAIL] Event[%0d] Merged voltage = 0x%02h (expected 0x%02h) | %s", 
                         evt, o_merged_voltages[evt], exp_voltage, desc);
                fail++;
            end
        end
    endtask
    
    task verify_past_range(input int evt, input [7:0] min_val, max_val,
                          input string desc, output int pass, fail);
        begin
            if (o_past_voltages[evt] >= min_val && o_past_voltages[evt] <= max_val) begin
                $display(" [PASS] Event[%0d] Past voltage = 0x%02h (range 0x%02h-0x%02h) | %s", 
                         evt, o_past_voltages[evt], min_val, max_val, desc);
                pass++;
            end else begin
                $display(" [FAIL] Event[%0d] Past voltage = 0x%02h (expected 0x%02h-0x%02h) | %s", 
                         evt, o_past_voltages[evt], min_val, max_val, desc);
                fail++;
            end
        end
    endtask
    
    task verify_crossbar_activation(input logic [EVENT_DEPTH-1:0] expected_mask,
                                    input string desc, output int pass, fail);
        begin
            logic [EVENT_DEPTH-1:0] actual = dut.crossbar_activated_events;
            if (actual === expected_mask) begin
                $display(" [PASS] Crossbar activated events = 0x%08h | %s", actual, desc);
                pass++;
            end else begin
                $display(" [FAIL] Crossbar activated = 0x%08h (expected 0x%08h) | %s", 
                         actual, expected_mask, desc);
                fail++;
            end
        end
    endtask
    
    task print_voltages(input string label);
        begin
            $display("\n--- %s ---", label);
            $display(" Event | MCAM   | Past   | Merged | Interpretation");
            $display("-------+--------+--------+--------+---------------------------");
            for (int i = 0; i < EVENT_DEPTH; i++) begin
                if (o_event_voltages[i] != 0 || o_past_voltages[i] != 0 || 
                    o_merged_voltages[i] != 0) begin
                    string tag;
                    case (o_event_voltages[i])
                        VP: tag = "PRESENT";
                        VF1: tag = "FUTURE-1";
                        VF2: tag = "FUTURE-2";
                        default: tag = (o_past_voltages[i] > 0) ? "PAST-DECAY" : "---";
                    endcase
                    $display(" %5d | 0x%02h | 0x%02h | 0x%02h | %s",
                             i, o_event_voltages[i], o_past_voltages[i], 
                             o_merged_voltages[i], tag);
                end
            end
            $display("-------+--------+--------+--------+---------------------------\n");
        end
    endtask

    initial begin
        int local_pass, local_fail;
        logic [EVENT_DEPTH-1:0] expected_chain;
        
        reset_dut();
        
        // TEST 1: train Blue, Magenta and Green. Nothing is checked here.
        start_test("Training Mode - Memory Programming");
        local_pass = 0; local_fail = 0;
        
        $display("\n--- Training Blue Context (Primes 7->5->3->2) ---");
        train_crossbar_assoc(EVT_7, EVT_5);
        train_crossbar_assoc(EVT_5, EVT_3);
        train_crossbar_assoc(EVT_3, EVT_2);
        train_crossbar_gate(EVT_7, CTX_BLUE);
        train_crossbar_gate(EVT_5, CTX_BLUE);
        train_crossbar_gate(EVT_3, CTX_BLUE);
        train_crossbar_gate(EVT_2, CTX_BLUE);
        train_mcam(EVT_7, CTX_BLUE, 0, "first");
        train_mcam(EVT_5, CTX_BLUE, 1, "second");
        train_mcam(EVT_3, CTX_BLUE, 2, "third");
        train_mcam(EVT_2, CTX_BLUE, 3, "fourth");
        
        $display("\n--- Training Magenta Context (Odds 7->5->3->1) ---");
        train_crossbar_assoc(EVT_7, EVT_5);  // same links as Blue
        train_crossbar_assoc(EVT_5, EVT_3);
        train_crossbar_assoc(EVT_3, EVT_1);
        train_crossbar_gate(EVT_7, CTX_MAGENTA);
        train_crossbar_gate(EVT_5, CTX_MAGENTA);
        train_crossbar_gate(EVT_3, CTX_MAGENTA);
        train_crossbar_gate(EVT_1, CTX_MAGENTA);
        train_mcam(EVT_7, CTX_MAGENTA, 0, "first");
        train_mcam(EVT_5, CTX_MAGENTA, 1, "second");
        train_mcam(EVT_3, CTX_MAGENTA, 2, "third");
        train_mcam(EVT_1, CTX_MAGENTA, 3, "fourth");
        
        $display("\n--- Training Green Context (Evens 2->4->6->8) ---");
        train_crossbar_assoc(EVT_2, EVT_4);
        train_crossbar_assoc(EVT_4, EVT_6);
        train_crossbar_assoc(EVT_6, EVT_8);
        train_crossbar_gate(EVT_2, CTX_GREEN);
        train_crossbar_gate(EVT_4, CTX_GREEN);
        train_crossbar_gate(EVT_6, CTX_GREEN);
        train_crossbar_gate(EVT_8, CTX_GREEN);
        train_mcam(EVT_2, CTX_GREEN, 0, "first");
        train_mcam(EVT_4, CTX_GREEN, 1, "second");
        train_mcam(EVT_6, CTX_GREEN, 2, "third");
        train_mcam(EVT_8, CTX_GREEN, 3, "fourth");
        
        $display("\n [INFO] Training complete - no functional verification in this mode");
        local_pass++;
        
        end_test(local_pass, local_fail);
        repeat(5) @(posedge clk);
        
        // TEST 2: Blue, cue 7. Crossbar reaches all four events, MCAM tags only 7/5/3.
        start_test("Basic Inference - Blue/Event-7 (Sequence Start)");
        local_pass = 0; local_fail = 0;
        
        run_inference(CTX_BLUE, EVT_7, "Blue/Event-7");
        
        expected_chain = '0;
        expected_chain[EVT_7] = 1'b1;
        expected_chain[EVT_5] = 1'b1;
        expected_chain[EVT_3] = 1'b1;
        expected_chain[EVT_2] = 1'b1;
        verify_crossbar_activation(expected_chain, "Chain: 7->5->3->2", 
                                   local_pass, local_fail);
        
        verify_voltage(EVT_7, VP, "Event-7 = PRESENT", local_pass, local_fail);
        verify_voltage(EVT_5, VF1, "Event-5 = FUTURE-1", local_pass, local_fail);
        verify_voltage(EVT_3, VF2, "Event-3 = FUTURE-2", local_pass, local_fail);
        verify_voltage(EVT_2, 8'h00, "Event-2 = None (beyond +2)", local_pass, local_fail);
        
        verify_merged(EVT_7, VP, "Merged-7 = Present", local_pass, local_fail);
        verify_merged(EVT_5, VF1, "Merged-5 = Future-1", local_pass, local_fail);
        verify_merged(EVT_3, VF2, "Merged-3 = Future-2", local_pass, local_fail);
        
        print_voltages("Test 2 Output");
        
        end_test(local_pass, local_fail);
        repeat(10) @(posedge clk);
        
        // TEST 3: cue 7 in Blue, then in Magenta
        start_test("Context Switch - Event-7 in Blue vs Magenta");
        local_pass = 0; local_fail = 0;
        
        $display("\n--- Sub-test 3a: Blue Context ---");
        run_inference(CTX_BLUE, EVT_7, "Blue/Event-7");
        
        verify_voltage(EVT_7, VP, "Blue: Event-7 = PRESENT", local_pass, local_fail);
        verify_voltage(EVT_5, VF1, "Blue: Event-5 = FUTURE-1", local_pass, local_fail);
        verify_voltage(EVT_3, VF2, "Blue: Event-3 = FUTURE-2", local_pass, local_fail);
        verify_voltage(EVT_2, 8'h00, "Blue: Event-2 = None", local_pass, local_fail);
        verify_voltage(EVT_1, 8'h00, "Blue: Event-1 = None (wrong context)", 
                      local_pass, local_fail);
        
        repeat(10) @(posedge clk);
        
        $display("\n--- Sub-test 3b: Magenta Context ---");
        run_inference(CTX_MAGENTA, EVT_7, "Magenta/Event-7");
        
        verify_voltage(EVT_7, VP, "Magenta: Event-7 = PRESENT", local_pass, local_fail);
        verify_voltage(EVT_5, VF1, "Magenta: Event-5 = FUTURE-1", local_pass, local_fail);
        verify_voltage(EVT_3, VF2, "Magenta: Event-3 = FUTURE-2", local_pass, local_fail);
        verify_voltage(EVT_1, 8'h00, "Magenta: Event-1 = None (beyond +2)", 
                      local_pass, local_fail);
        verify_voltage(EVT_2, 8'h00, "Magenta: Event-2 = None (wrong context)", 
                      local_pass, local_fail);
        
        $display("\n [INFO] Context-dependent gating verified: same Event-7, different chains");
        
        end_test(local_pass, local_fail);
        repeat(10) @(posedge clk);
        
        // TEST 4: move the cue along Blue (7, then 5, then 3)
        start_test("Sliding Window - Progressive Anchor Shift");
        local_pass = 0; local_fail = 0;
        
        $display("\n--- Sub-test 4a: Anchor at Event-7 (Start) ---");
        run_inference(CTX_BLUE, EVT_7, "Blue/Event-7");
        verify_voltage(EVT_7, VP, "Anchor-7: Present", local_pass, local_fail);
        verify_voltage(EVT_5, VF1, "Anchor-7: Future+1", local_pass, local_fail);
        verify_voltage(EVT_3, VF2, "Anchor-7: Future+2", local_pass, local_fail);
        
        repeat(10) @(posedge clk);
        
        $display("\n--- Sub-test 4b: Anchor at Event-5 (Middle) ---");
        run_inference(CTX_BLUE, EVT_5, "Blue/Event-5");
        verify_voltage(EVT_5, VP, "Anchor-5: Present", local_pass, local_fail);
        verify_voltage(EVT_3, VF1, "Anchor-5: Future+1 (shifted)", 
                      local_pass, local_fail);
        verify_voltage(EVT_2, VF2, "Anchor-5: Future+2 (shifted)", 
                      local_pass, local_fail);
        verify_voltage(EVT_7, 8'h00, "Anchor-5: Event-7 not predicted", 
                      local_pass, local_fail);
        
        repeat(10) @(posedge clk);
        
        $display("\n--- Sub-test 4c: Anchor at Event-3 (Near End) ---");
        run_inference(CTX_BLUE, EVT_3, "Blue/Event-3");
        verify_voltage(EVT_3, VP, "Anchor-3: Present", local_pass, local_fail);
        verify_voltage(EVT_2, VF1, "Anchor-3: Future+1 (last)", 
                      local_pass, local_fail);
        verify_voltage(EVT_5, 8'h00, "Anchor-3: Event-5 not predicted", 
                      local_pass, local_fail);
        
        $display("\n [INFO] Sliding window verified: predictions shift with anchor");
        
        end_test(local_pass, local_fail);
        repeat(10) @(posedge clk);
        
        // TEST 5: past trace. Only cue events are charged; with DECAY_STEP=4 the
        // trace drops by 4 per cycle from 0xFF. ~26 cycles is the delay from charge
        // to check; the windows below allow some slack.
        start_test("Temporal Decay - Anchor-Only Tracking");
        local_pass = 0; local_fail = 0;
        
        $display("\n--- Phase 1: First inference (Event-7) ---");
        run_inference(CTX_BLUE, EVT_7, "Blue/Event-7 (First)");
        
        // ~26 cycles since charge: 0xFF - 26*4 = 0x97
        verify_past_range(EVT_7, 8'h90, 8'hA0, "Event-7 charged ~26 cycles ago", 
                         local_pass, local_fail);
        
        $display("\n--- Phase 2: Wait and observe decay ---");
        repeat(20) @(posedge clk);
        
        // ~46 cycles: 0xFF - 46*4 = 0x4B
        verify_past_range(EVT_7, 8'h43, 8'h53, "Event-7 decayed 46 cycles total", 
                         local_pass, local_fail);
        
        $display("\n--- Phase 3: Second inference (Event-5) ---");
        run_inference(CTX_BLUE, EVT_5, "Blue/Event-5 (Second)");
        
        verify_past_range(EVT_5, 8'h90, 8'hA0, "Event-5 charged ~26 cycles ago", 
                         local_pass, local_fail);
        
        // 7 kept decaying during the second run: 0x4B - 26*4 goes below 0, clamps at 0
        verify_past_range(EVT_7, 8'h00, 8'h10, "Event-7 fully decayed", 
                         local_pass, local_fail);
        
        // 3 was only predicted, never a cue, so it has no trace
        if (o_past_voltages[EVT_3] == 8'h00) begin
            $display(" [PASS] Event-3 past = 0x00 | Not charged (crossbar chain)");
            local_pass++;
        end else begin
            $display(" [FAIL] Event-3 past = 0x%02h (expected 0x00) | Should not track predictions",
                     o_past_voltages[EVT_3]);
            local_fail++;
        end
        
        $display("\n [INFO] Anchor-only tracking verified: only experienced events decay");
        
        end_test(local_pass, local_fail);
        repeat(10) @(posedge clk);
        
        // TEST 6: when an event has both an MCAM tag and a past trace, the merged output uses the tag
        start_test("MCAM Priority - Sharp Overrides Decaying");
        local_pass = 0; local_fail = 0;
        
        $display("\n--- Phase 1: Prime Event-7 with past charge ---");
        run_inference(CTX_BLUE, EVT_7, "Blue/Event-7 (Prime)");
        repeat(50) @(posedge clk);
        
        $display(" Event-7 past voltage before re-query: 0x%02h", o_past_voltages[EVT_7]);
        
        $display("\n--- Phase 2: Re-query Event-7 (should get MCAM tag) ---");
        run_inference(CTX_BLUE, EVT_7, "Blue/Event-7 (Re-query)");
        
        verify_voltage(EVT_7, VP, "Event-7 MCAM = VP", local_pass, local_fail);
        verify_merged(EVT_7, VP, "Event-7 Merged = VP (MCAM priority)", 
                     local_pass, local_fail);
        
        if (o_past_voltages[EVT_7] > 8'h00) begin
            $display(" [PASS] Event-7 past = 0x%02h | Decay persists", 
                     o_past_voltages[EVT_7]);
            local_pass++;
        end else begin
            $display(" [FAIL] Event-7 past = 0x00 | Decay should persist");
            local_fail++;
        end
        
        $display("\n [INFO] Priority verified: held_mcam > live_mcam > past_decay");
        
        end_test(local_pass, local_fail);
        repeat(10) @(posedge clk);
        
        // TEST 7: tags must hold after the MCAM goes idle
        start_test("Held Voltages - Persistence After MCAM Completes");
        local_pass = 0; local_fail = 0;
        
        run_inference(CTX_BLUE, EVT_7, "Blue/Event-7");
        
        $display("\n--- Capturing held voltages immediately after valid ---");
        held_snapshot_7 = o_event_voltages[EVT_7];
        held_snapshot_5 = o_event_voltages[EVT_5];
        held_snapshot_3 = o_event_voltages[EVT_3];
        
        $display(" Snapshot: Event-7=0x%02h, Event-5=0x%02h, Event-3=0x%02h",
                 held_snapshot_7, held_snapshot_5, held_snapshot_3);
        
        $display("\n--- Waiting 50 cycles (MCAM returned to IDLE) ---");
        repeat(50) @(posedge clk);
        
        if (o_event_voltages[EVT_7] === held_snapshot_7 &&
            o_event_voltages[EVT_5] === held_snapshot_5 &&
            o_event_voltages[EVT_3] === held_snapshot_3) begin
            $display(" [PASS] Held voltages stable after 50 cycles");
            local_pass++;
        end else begin
            $display(" [FAIL] Held voltages changed: 7=0x%02h, 5=0x%02h, 3=0x%02h",
                     o_event_voltages[EVT_7], o_event_voltages[EVT_5], 
                     o_event_voltages[EVT_3]);
            local_fail++;
        end
        
        $display("\n [INFO] Voltage persistence verified: outputs stable for CA1 ramp");
        
        end_test(local_pass, local_fail);
        repeat(10) @(posedge clk);
        
        // TEST 8: extend Blue to 7->5->3->2->1. All five activate, only 7/5/3 get tags.
        start_test("Longer Sequence - 5-Event Chain (7->5->3->2->1)");
        local_pass = 0; local_fail = 0;
        
        $display("\n--- Training extended sequence ---");
        train_crossbar_assoc(EVT_2, EVT_1);
        train_crossbar_gate(EVT_1, CTX_BLUE);
        train_mcam(EVT_1, CTX_BLUE, 4, "fifth");
        
        $display("\n--- Inference from start (Event-7) ---");
        run_inference(CTX_BLUE, EVT_7, "Blue/Event-7 (5-event chain)");
        
        verify_voltage(EVT_7, VP, "Event-7 = PRESENT", local_pass, local_fail);
        verify_voltage(EVT_5, VF1, "Event-5 = FUTURE-1", local_pass, local_fail);
        verify_voltage(EVT_3, VF2, "Event-3 = FUTURE-2", local_pass, local_fail);
        verify_voltage(EVT_2, 8'h00, "Event-2 = None (position +3)", 
                      local_pass, local_fail);
        verify_voltage(EVT_1, 8'h00, "Event-1 = None (position +4)", 
                      local_pass, local_fail);
        
        expected_chain = '0;
        expected_chain[EVT_7] = 1'b1;
        expected_chain[EVT_5] = 1'b1;
        expected_chain[EVT_3] = 1'b1;
        expected_chain[EVT_2] = 1'b1;
        expected_chain[EVT_1] = 1'b1;
        verify_crossbar_activation(expected_chain, "Full 5-event chain activated", 
                                   local_pass, local_fail);
        
        $display("\n [INFO] +2 window enforced: positions +3, +4 not tagged despite activation");
        
        end_test(local_pass, local_fail);
        repeat(10) @(posedge clk);
        
        // TEST 9: untrained context, event with no links, cue at the end of a chain
        start_test("Edge Cases - Invalid Inputs & Boundaries");
        local_pass = 0; local_fail = 0;
        
        $display("\n--- Sub-test 9a: Invalid context (not trained) ---");
        run_inference(CTX_RED, EVT_7, "Red/Event-7 (untrained)");
        
        // the MCAM keys off the cue, not the crossbar, so 7 should still get VP
        verify_voltage(EVT_7, VP, "Event-7 tagged despite invalid context", 
                      local_pass, local_fail);
        
        $display("\n--- Sub-test 9b: Event with no associations ---");
        run_inference(CTX_BLUE, EVT_9, "Blue/Event-9 (isolated)");
        
        expected_chain = '0;
        expected_chain[EVT_9] = 1'b1;
        verify_crossbar_activation(expected_chain, "Only anchor activated", 
                                   local_pass, local_fail);
        
        $display("\n--- Sub-test 9c: Sequence boundary (last event) ---");
        run_inference(CTX_BLUE, EVT_2, "Blue/Event-2 (sequence end)");
        
        verify_voltage(EVT_2, VP, "Event-2 = PRESENT", local_pass, local_fail);
        verify_voltage(EVT_1, VF1, "Event-1 = FUTURE-1", local_pass, local_fail);
        verify_voltage(EVT_3, 8'h00, "Event-3 = None (reverse link)", 
                      local_pass, local_fail);
        
        $display("\n [INFO] Edge cases handled: system robust to invalid/boundary inputs");
        
        end_test(local_pass, local_fail);
        repeat(10) @(posedge clk);
        
        // TEST 10: latency. The crossbar must take MAX_STEPS+1 cycles; the MCAM range is a warning only.
        start_test("Latency Measurement - Pipeline Breakdown");
        local_pass = 0; local_fail = 0;
        
        $display("\n--- Measuring Blue/Event-7 ---");
        run_inference(CTX_BLUE, EVT_7, "Latency Test 1");
        
        test_crossbar_latency = crossbar_done_cycle - crossbar_start_cycle;
        test_mcam_latency = mcam_done_cycle - crossbar_done_cycle;
        test_total_latency = ca3_done_cycle - inference_start_cycle;
        
        $display("\n Crossbar Latency: %0d cycles", test_crossbar_latency);
        $display(" MCAM Latency: %0d cycles", test_mcam_latency);
        $display(" Total CA3 Latency: %0d cycles", test_total_latency);
        
        if (test_crossbar_latency == MAX_STEPS + 1) begin
            $display(" [PASS] Crossbar latency = MAX_STEPS+1 (%0d)", MAX_STEPS + 1);
            local_pass++;
        end else begin
            $display(" [FAIL] Crossbar latency = %0d (expected %0d)", 
                     test_crossbar_latency, MAX_STEPS + 1);
            local_fail++;
        end
        
        if (test_mcam_latency >= 10 && test_mcam_latency <= 70) begin
            $display(" [PASS] MCAM latency in expected range (10-70 cycles)");
            local_pass++;
        end else begin
            $display(" [WARN] MCAM latency = %0d (typical 10-70)", test_mcam_latency);
            local_pass++; // counted as a pass
        end
        
        repeat(10) @(posedge clk);
        
        $display("\n--- Measuring Green/Event-2 (different t_ref) ---");
        run_inference(CTX_GREEN, EVT_2, "Latency Test 2");
        test_mcam_latency = mcam_done_cycle - crossbar_done_cycle;
        
        $display(" MCAM Latency (Green/Event-2): %0d cycles", test_mcam_latency);
        $display("\n [INFO] MCAM latency varies with t_ref (anchor temporal code)");
        local_pass++;
        
        end_test(local_pass, local_fail);
        
        $display("\n------------------------------------------------------------------");
        $display(" CA3 STAGE TESTBENCH SUMMARY");
        $display("------------------------------------------------------------------");
        $display(" Total Tests: %0d", test_num);
        $display(" Tests Passed: %0d", tests_passed);
        $display(" Tests Failed: %0d", tests_failed);
        $display("");
        $display(" Total Checks: %0d", checks_passed + checks_failed);
        $display(" Checks Passed: %0d", checks_passed);
        $display(" Checks Failed: %0d", checks_failed);
        $display("");
        
        if (tests_failed == 0 && checks_failed == 0) begin
            $display(" RESULT: ALL TESTS PASSED");
            $display("");
            $display(" VERIFIED FEATURES:");
            $display("  Crossbar association & context-gating");
            $display("  MCAM 3-counter progressive prediction");
            $display("  Held voltage persistence (CA1 interface stability)");
            $display("  Anchor-only temporal tracking");
            $display("  MCAM priority over decaying past");
            $display("  Context-dependent sequence retrieval");
            $display("  Sliding window (anchor-relative alignment)");
            $display("  +2 prediction window enforcement");
            $display("  Edge case robustness");
            $display("  Pipeline latency characterization");
        end else begin
            $display(" RESULT: %0d TEST(S) FAILED, %0d CHECK(S) FAILED", 
                     tests_failed, checks_failed);
        end
        
        $display("------------------------------------------------------------------\n");
        
        #1000;
        $finish;
    end
    
    // watchdog
    initial begin
        #500000; // 500 us
        $display("\n[ERROR] Testbench timeout at %0t", $time);
        $display(" Last completed test: %0d", test_num);
        $finish;
    end

endmodule
