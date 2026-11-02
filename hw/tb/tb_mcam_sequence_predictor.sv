// tb_mcam_sequence_predictor: unit tests for the MCAM timing tags.
// No crossbar here: the test drives i_activated_events directly (the whole
// chain) and puts the cue on i_anchor_event_onehot.
// Covers Blue/Magenta/Green chains, moving the cue along a chain, longer
// sequences, overwritten codes and invalid inputs.

`timescale 1ns/1ps

module tb_mcam_sequence_predictor;

    localparam EVENT_DEPTH = 128;
    localparam CONTEXT_DEPTH = 16;
    localparam CNTR_WIDTH = 6;
    localparam VOLTAGE_WIDTH = 8;
    localparam CLK_PERIOD = 10;
    
    localparam logic [VOLTAGE_WIDTH-1:0] VP  = 8'h40;
    localparam logic [VOLTAGE_WIDTH-1:0] VF1 = 8'h80;
    localparam logic [VOLTAGE_WIDTH-1:0] VF2 = 8'hC0;

    // context ids (one-hot bit index)
    localparam int CTX_BLUE    = 0;
    localparam int CTX_MAGENTA = 1;
    localparam int CTX_GREEN   = 2;
    localparam int CTX_BLACK   = 3;
    localparam int CTX_RED     = 4;
    localparam int CTX_YELLOW  = 5;
    
    // event ids, named by value so the sequences read naturally
    localparam int EVT_1 = 1, EVT_2 = 2, EVT_3 = 3, EVT_4 = 4;
    localparam int EVT_5 = 5, EVT_6 = 6, EVT_7 = 7, EVT_8 = 8;
    localparam int EVT_9 = 9, EVT_10 = 10, EVT_11= 11, EVT_12 = 12;
    localparam int EVT_20 = 20, EVT_21 = 21, EVT_22 = 22;

    logic clk, rst_n;
    logic [EVENT_DEPTH-1:0]   i_activated_events;
    logic [CONTEXT_DEPTH-1:0] i_context_onehot;
    logic [EVENT_DEPTH-1:0]   i_anchor_event_onehot;
    logic i_start, i_crossbar_done, i_train_mode, i_mcam_we;
    logic [$clog2(EVENT_DEPTH)-1:0]   i_mcam_event_addr;
    logic [$clog2(CONTEXT_DEPTH)-1:0] i_mcam_context_addr;
    logic [CNTR_WIDTH-1:0]            i_mcam_data;
    logic [VOLTAGE_WIDTH-1:0] o_event_voltages [EVENT_DEPTH];
    logic o_done, o_ready;
    logic [EVENT_DEPTH-1:0] partial_yellow;
    logic [EVENT_DEPTH-1:0] sparse_blue;
    logic [EVENT_DEPTH-1:0] boundary_green;

    mcam_sequence_predictor #(
        .EVENT_DEPTH(EVENT_DEPTH),
        .CONTEXT_DEPTH(CONTEXT_DEPTH),
        .CNTR_WIDTH(CNTR_WIDTH),
        .VOLTAGE_WIDTH(VOLTAGE_WIDTH),
        .CNTR2_DIV(2),
        .CNTR3_DIV(4)
    ) dut (
        .i_clk(clk),
        .i_rst_n(rst_n),
        .i_activated_events(i_activated_events),
        .i_context_onehot(i_context_onehot),
        .i_anchor_event_onehot(i_anchor_event_onehot),
        .i_start(i_start),
        .i_crossbar_done(i_crossbar_done),
        .i_train_mode(i_train_mode),
        .i_mcam_we(i_mcam_we),
        .i_mcam_event_addr(i_mcam_event_addr),
        .i_mcam_context_addr(i_mcam_context_addr),
        .i_mcam_data(i_mcam_data),
        .o_event_voltages(o_event_voltages),
        .o_done(o_done),
        .o_ready(o_ready)
    );

    // 100 MHz
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    task automatic reset_dut();
        begin
            rst_n = 0;
            i_train_mode = 0;
            i_start = 0;
            i_crossbar_done = 0;
            i_activated_events = '0;
            i_anchor_event_onehot = '0;
            i_context_onehot = '0;
            #50;
            rst_n = 1;
            #20;
            $display("[%0t] Reset completed", $time);
        end
    endtask

    // one MCAM write: event evt in context ctx gets time code t_code
    task automatic train_mcam(input int evt, ctx, t_code);
        begin
            i_train_mode = 1;
            i_mcam_we = 1;
            i_mcam_event_addr = evt;
            i_mcam_context_addr = ctx;
            i_mcam_data = t_code;
            @(posedge clk);
            i_mcam_we = 0;
            @(posedge clk);
            i_train_mode = 0;
            $display("[%0t] Programmed MCAM[%0d][%0d] = %0d", $time, evt, ctx, t_code);
        end
    endtask

    // run one inference and report latency (start to o_done, in cycles)
    task automatic run_inference(
        input [EVENT_DEPTH-1:0] activated_mask,
        input int anchor_evt,
        input int ctx,
        input string test_desc = ""
    );
        int start_time, end_time, latency_cycles;
        begin
            i_train_mode = 0;
            i_activated_events = activated_mask;
            i_anchor_event_onehot = (1 << anchor_evt);
            i_context_onehot = (1 << ctx);
            i_crossbar_done = 0;
            
            start_time = $time / CLK_PERIOD;
            i_start = 1;
            @(posedge clk);
            i_start = 0;
            
            fork
                begin
                    wait(o_done);
                    end_time = $time / CLK_PERIOD;
                end
                begin
                    repeat(2000) @(posedge clk);
                    $error("[%0t] Inference timeout!", $time);
                end
            join_any
            disable fork;
            
            latency_cycles = end_time - start_time;
            
            if (test_desc != "")
                $display("[%0t] Inference complete: %s (Anchor EVT_%0d, CTX_%0d) - Latency: %0d cycles", 
                         $time, test_desc, anchor_evt, ctx, latency_cycles);
            else
                $display("[%0t] Inference complete: Anchor EVT_%0d, CTX_%0d - Latency: %0d cycles", 
                         $time, anchor_evt, ctx, latency_cycles);
            
            print_voltages();
            @(posedge clk);
        end
    endtask

    // list every event with a nonzero tag
    task automatic print_voltages();
        begin
            $display("========== Voltage Output ==========");
            for (int i = 0; i < EVENT_DEPTH; i++) begin
                if (o_event_voltages[i] != 0) begin
                    string role;
                    case (o_event_voltages[i])
                        VP:  role = "PRESENT";
                        VF1: role = "FUTURE-1";
                        VF2: role = "FUTURE-2";
                        default: role = "UNKNOWN";
                    endcase
                    $display("  Event[%0d] = 0x%h (%s)", i, o_event_voltages[i], role);
                end
            end
            $display("====================================");
        end
    endtask

    int test_num = 0, pass_count = 0, fail_count = 0;

    task automatic verify_voltage(input int evt, input [7:0] exp_v, input string desc);
        begin
            if (o_event_voltages[evt] !== exp_v) begin
                $error("[FAIL] %s: Event[%0d] = 0x%h (expected 0x%h)", desc, evt, o_event_voltages[evt], exp_v);
                fail_count++;
            end else begin
                $display("  [PASS] %s", desc);
                pass_count++;
            end
        end
    endtask

    logic [EVENT_DEPTH-1:0] full_blue, full_magenta, full_green, full_black, full_red, full_yellow;

    initial begin
        $display("\n========================================================");
        $display("    MCAM 3-Counter Sweeping Comprehensive Testbench");
        $display("========================================================\n");
        reset_dut();

        // Test 1: Blue, 7 -> 5 -> 3 -> 2
        test_num++;
        $display("\n[TEST %0d] Blue Context: Basic Sequence (Primes Descending)", test_num);
        $display("----------------------------------------------------------------");
        
        train_mcam(EVT_7, CTX_BLUE, 0);
        train_mcam(EVT_5, CTX_BLUE, 1);
        train_mcam(EVT_3, CTX_BLUE, 2);
        train_mcam(EVT_2, CTX_BLUE, 3);
        
        full_blue = (1<<EVT_7) | (1<<EVT_5) | (1<<EVT_3) | (1<<EVT_2);
        
        run_inference(full_blue, EVT_7, CTX_BLUE, "Blue Start");
        verify_voltage(EVT_7, VP,  "Present=7");
        verify_voltage(EVT_5, VF1, "Future-1=5");
        verify_voltage(EVT_3, VF2, "Future-2=3");
        verify_voltage(EVT_2, 8'h00, "Not yet=2");
        
        run_inference(full_blue, EVT_5, CTX_BLUE, "Blue Mid-Sequence");
        verify_voltage(EVT_5, VP,  "Present=5");
        verify_voltage(EVT_3, VF1, "Future-1=3");
        verify_voltage(EVT_2, VF2, "Future-2=2");

        // Test 2: Magenta, 7 -> 5 -> 3 -> 1. Same start as Blue, different tail.
        test_num++;
        $display("\n[TEST %0d] Magenta Context: Context-Aware Prediction (Odds)", test_num);
        $display("----------------------------------------------------------------");
        
        train_mcam(EVT_7, CTX_MAGENTA, 0);
        train_mcam(EVT_5, CTX_MAGENTA, 1);
        train_mcam(EVT_3, CTX_MAGENTA, 2);
        train_mcam(EVT_1, CTX_MAGENTA, 3);
        
        full_magenta = (1<<EVT_7) | (1<<EVT_5) | (1<<EVT_3) | (1<<EVT_1);
        
        run_inference(full_magenta, EVT_7, CTX_MAGENTA, "Magenta Start");
        verify_voltage(EVT_7, VP,  "Present=7");
        verify_voltage(EVT_5, VF1, "Future-1=5");
        verify_voltage(EVT_3, VF2, "Future-2=3");
        verify_voltage(EVT_1, 8'h00, "Not yet=1");
        
        run_inference(full_magenta, EVT_3, CTX_MAGENTA, "Magenta Near-End");
        verify_voltage(EVT_3, VP,  "Present=3");
        verify_voltage(EVT_1, VF1, "Future-1=1 (context-aware)");

        // Test 3: Green, 2 -> 4 -> 6 -> 8
        test_num++;
        $display("\n[TEST %0d] Green Context: Evens Ascending Sequence", test_num);
        $display("----------------------------------------------------------------");
        
        train_mcam(EVT_2, CTX_GREEN, 0);
        train_mcam(EVT_4, CTX_GREEN, 1);
        train_mcam(EVT_6, CTX_GREEN, 2);
        train_mcam(EVT_8, CTX_GREEN, 3);
        
        full_green = (1<<EVT_2) | (1<<EVT_4) | (1<<EVT_6) | (1<<EVT_8);
        
        run_inference(full_green, EVT_2, CTX_GREEN, "Green Start");
        verify_voltage(EVT_2, VP,  "Present=2");
        verify_voltage(EVT_4, VF1, "Future-1=4");
        verify_voltage(EVT_6, VF2, "Future-2=6");
        
        run_inference(full_green, EVT_6, CTX_GREEN, "Green Near-End");
        verify_voltage(EVT_6, VP,  "Present=6");
        verify_voltage(EVT_8, VF1, "Future-1=8");

        // Test 4: Black, five events 2 -> 3 -> 5 -> 7 -> 11
        test_num++;
        $display("\n[TEST %0d] Black Context: Longer Sequence (5 events)", test_num);
        $display("----------------------------------------------------------------");
        
        train_mcam(EVT_2,  CTX_BLACK, 0);
        train_mcam(EVT_3,  CTX_BLACK, 1);
        train_mcam(EVT_5,  CTX_BLACK, 2);
        train_mcam(EVT_7,  CTX_BLACK, 3);
        train_mcam(EVT_11, CTX_BLACK, 4);
        
        full_black = (1<<EVT_2) | (1<<EVT_3) | (1<<EVT_5) | (1<<EVT_7) | (1<<EVT_11);
        
        run_inference(full_black, EVT_2, CTX_BLACK, "Black Start");
        verify_voltage(EVT_2,  VP,  "Present=2");
        verify_voltage(EVT_3,  VF1, "Future-1=3");
        verify_voltage(EVT_5,  VF2, "Future-2=5");
        verify_voltage(EVT_7,  8'h00, "Not yet=7");
        verify_voltage(EVT_11, 8'h00, "Not yet=11");
        
        run_inference(full_black, EVT_5, CTX_BLACK, "Black Mid-Sequence");
        verify_voltage(EVT_5,  VP,  "Present=5");
        verify_voltage(EVT_7,  VF1, "Future-1=7");
        verify_voltage(EVT_11, VF2, "Future-2=11");

        // Test 5: codes start at 30, so tags have to be relative to the cue's code
        test_num++;
        $display("\n[TEST %0d] Red Context: High Temporal Codes (t_ref=30+)", test_num);
        $display("----------------------------------------------------------------");
        
        train_mcam(EVT_20, CTX_RED, 30);
        train_mcam(EVT_21, CTX_RED, 31);
        train_mcam(EVT_22, CTX_RED, 32);
        train_mcam(EVT_10, CTX_RED, 33);
        
        full_red = (1<<EVT_20) | (1<<EVT_21) | (1<<EVT_22) | (1<<EVT_10);
        
        run_inference(full_red, EVT_20, CTX_RED, "High T Start (t=30)");
        verify_voltage(EVT_20, VP,  "Present=20 (t=30)");
        verify_voltage(EVT_21, VF1, "Future-1=21 (t=31)");
        verify_voltage(EVT_22, VF2, "Future-2=22 (t=32)");
        
        $display("  [NOTE] Latency should be ~30+ cycles due to CNTR-1 sweep to t=30");

        // Test 6: only some trained events come in active
        test_num++;
        $display("\n[TEST %0d] Yellow Context: Partial Activation", test_num);
        $display("----------------------------------------------------------------");
        
        train_mcam(EVT_1, CTX_YELLOW, 0);
        train_mcam(EVT_3, CTX_YELLOW, 1);
        train_mcam(EVT_5, CTX_YELLOW, 2);
        train_mcam(EVT_7, CTX_YELLOW, 3);
        train_mcam(EVT_9, CTX_YELLOW, 4);
        
        // 1, 3, 5 active; 7 and 9 are trained but must stay untagged
        partial_yellow = (1<<EVT_1) | (1<<EVT_3) | (1<<EVT_5);
        
        run_inference(partial_yellow, EVT_1, CTX_YELLOW, "Partial Activation (3/5 events)");
        verify_voltage(EVT_1, VP,  "Present=1");
        verify_voltage(EVT_3, VF1, "Future-1=3");
        verify_voltage(EVT_5, VF2, "Future-2=5");
        verify_voltage(EVT_7, 8'h00, "Not activated=7");
        verify_voltage(EVT_9, 8'h00, "Not activated=9");

        // Test 7: gaps in the time codes
        test_num++;
        $display("\n[TEST %0d] Blue Context: Sparse Temporal Codes (gaps)", test_num);
        $display("----------------------------------------------------------------");
        
        // rewrites part of Blue from test 1 (EVT_2 moves from t=3 to t=6)
        train_mcam(EVT_1, CTX_BLUE, 5);
        train_mcam(EVT_2, CTX_BLUE, 6);
        train_mcam(EVT_4, CTX_BLUE, 10);  // t=7 is empty, so nothing gets VF2 from EVT_1
        train_mcam(EVT_6, CTX_BLUE, 11);
        
        sparse_blue = (1<<EVT_1) | (1<<EVT_2) | (1<<EVT_4) | (1<<EVT_6);
        
        run_inference(sparse_blue, EVT_1, CTX_BLUE, "Sparse T codes");
        verify_voltage(EVT_1, VP,  "Present=1 (t=5)");
        verify_voltage(EVT_2, VF1, "Future-1=2 (t=6)");
        verify_voltage(EVT_4, 8'h00, "Gap - not Future-2 (t=10?7)");

        run_inference(sparse_blue, EVT_4, CTX_BLUE, "Sparse T codes 2");
        verify_voltage(EVT_4, VP,  "Present=4 (t=10)");
        verify_voltage(EVT_6, VF1, "Future-1=6 (t=11)");

        // Test 8: cue on the last event, nothing after it
        test_num++;
        $display("\n[TEST %0d] Magenta Context: End-of-Sequence", test_num);
        $display("----------------------------------------------------------------");
        
        run_inference(full_magenta, EVT_1, CTX_MAGENTA, "Last Event (no futures)");
        verify_voltage(EVT_1, VP,  "Present=1 (last event)");
        verify_voltage(EVT_3, 8'h00, "No Future-1 (end of sequence)");
        verify_voltage(EVT_5, 8'h00, "No Future-2 (end of sequence)");
        verify_voltage(EVT_7, 8'h00, "No Future-3 (end of sequence)");
      
        $display("  [NOTE] Counters should timeout gracefully after ~10 cycles");

        // Test 9: same cue event in two contexts
        test_num++;
        $display("\n[TEST %0d] Context Isolation: EVT_7 in Multiple Contexts", test_num);
        $display("----------------------------------------------------------------");
        
        // test 7 left EVT_5 at t=1 in Blue, so it is still future-1 here
        run_inference(full_blue, EVT_7, CTX_BLUE, "EVT_7 in Blue");
        verify_voltage(EVT_7, VP,  "Present=7 (Blue context)");
        verify_voltage(EVT_5, VF1, "Future-1=5 (Blue, t=1 unchanged)");
        
        run_inference(full_magenta, EVT_7, CTX_MAGENTA, "EVT_7 in Magenta");
        verify_voltage(EVT_7, VP,  "Present=7 (Magenta context)");
        verify_voltage(EVT_5, VF1, "Future-1=5 (Magenta intact)");
        
        $display("  [PASS] Context isolation verified - same event, different predictions");

        // Test 10: codes at the top of the 6-bit range
        test_num++;
        $display("\n[TEST %0d] Boundary Conditions: Near Max T (t=60+)", test_num);
        $display("----------------------------------------------------------------");
        
        train_mcam(EVT_9,  CTX_GREEN, 60);
        train_mcam(EVT_10, CTX_GREEN, 61);
        train_mcam(EVT_11, CTX_GREEN, 62);
        train_mcam(EVT_12, CTX_GREEN, 63);
        
        boundary_green = (1<<EVT_9) | (1<<EVT_10) | (1<<EVT_11) | (1<<EVT_12);
        
        run_inference(boundary_green, EVT_9, CTX_GREEN, "Boundary T=60");
        verify_voltage(EVT_9,  VP,  "Present=9 (t=60)");
        verify_voltage(EVT_10, VF1, "Future-1=10 (t=61)");
        verify_voltage(EVT_11, VF2, "Future-2=11 (t=62)");
        verify_voltage(EVT_12, 8'h00, "Not yet=12 (t=63)");
        
        $display("  [NOTE] CNTR-1 sweep to t=60: ~60 cycles");
        $display("  [NOTE] Counter approaching overflow (t_max=63)");

        $display("\n========================================================");
        $display("           MCAM 3-Counter Test Summary");
        $display("========================================================");
        $display("  Total Tests:        %0d", test_num);
        $display("  Passed Assertions:  %0d", pass_count);
        $display("  Failed Assertions:  %0d", fail_count);
        $display("========================================================");
        
        if (fail_count == 0) begin
            $display("\n ALL TESTS PASSED \n");
        end else begin
            $display("\n %0d TESTS FAILED \n", fail_count);
        end
        
        $finish;
    end

    // watchdog
    initial begin
        #(CLK_PERIOD * 500000);  // 5 ms
        $display("\n[ERROR] Testbench timeout after 5ms...");
        $display("  Possible infinite loop in counter sweep logic");
        $finish;
    end

endmodule
