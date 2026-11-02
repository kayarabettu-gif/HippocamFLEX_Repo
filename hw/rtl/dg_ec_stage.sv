// dg_ec_stage: pattern recognition (stage 1).
// Two proximity CAMs, one for contexts and one for events, searched in
// parallel. A distance above MAX_MISMATCH scores 0; otherwise the score is
// 255 - distance, so higher = better. A bitonic sorter per CAM picks the top-1
// and the result leaves as one-hot plus index.
// Latency: 1 cycle for the search, then both sorters run together. The event
// sorter is the slower one (23 cycles at 64 events); o_valid follows once
// both are done. Parameter defaults are overridden by hippocampex_top.

module dg_ec_stage #(
    parameter HV_WIDTH = 128,
    parameter NUM_CONTEXTS = 128,
    parameter NUM_EVENTS = 128,
    parameter DIST_WIDTH = 8,
    parameter MAX_MISMATCH = 13,            // 13/128 bits, ~90% similarity
    parameter TIMEOUT_CYCLES = 256      // not used
)(
    input clk,
    input rst_n,                    // active-low

    input  logic i_valid,                  // one-cycle start from control_fsm
    input  logic [HV_WIDTH-1:0] i_context_hv,
    input  logic [HV_WIDTH-1:0] i_event_hv,

    input  logic i_train_mode,

    input  logic i_train_wr_context,
    input  logic [$clog2(NUM_CONTEXTS)-1:0] i_train_context_addr,
    input  logic [HV_WIDTH-1:0] i_train_context_data,

    input  logic i_train_wr_event,
    input  logic [$clog2(NUM_EVENTS)-1:0] i_train_event_addr,
    input  logic [HV_WIDTH-1:0] i_train_event_data,

    // to CA3
    output logic [NUM_CONTEXTS-1:0] o_context_onehot,
    output logic [NUM_EVENTS-1:0] o_event_onehot,
    output logic [$clog2(NUM_CONTEXTS)-1:0] o_context_idx,
    output logic [$clog2(NUM_EVENTS)-1:0] o_event_idx,

    // analysis only, not connected in hippocampex_top
    output logic [NUM_EVENTS-1:0] o_event_multi_hot,              // every event within threshold
    output logic [DIST_WIDTH-1:0] o_event_distances[NUM_EVENTS],

    output logic o_valid,
    output logic o_ready

);

    localparam context_ADDR_WIDTH = $clog2(NUM_CONTEXTS);
    localparam event_ADDR_WIDTH = $clog2(NUM_EVENTS);
    localparam MAX_VOLTAGE = (1 << DIST_WIDTH) - 1;  // 255

    // context path
    logic [DIST_WIDTH-1:0] context_distances [NUM_CONTEXTS];
    logic [DIST_WIDTH-1:0] context_voltages [NUM_CONTEXTS];   // score, higher = better
    logic [context_ADDR_WIDTH-1:0] context_sorted_addrs [NUM_CONTEXTS];
    logic [DIST_WIDTH-1:0] context_sorted_dists [NUM_CONTEXTS];
    logic context_sort_start;
    logic context_sort_done;
    logic context_sort_ready;

    // event path
    logic [DIST_WIDTH-1:0] event_distances [NUM_EVENTS];
    logic [DIST_WIDTH-1:0] event_voltages [NUM_EVENTS];
    logic [event_ADDR_WIDTH-1:0] event_sorted_addrs [NUM_EVENTS];
    logic [DIST_WIDTH-1:0] event_sorted_dists [NUM_EVENTS];
    logic event_sort_start;
    logic event_sort_done;
    logic event_sort_ready;

    logic valid_reg;
    logic search_trigger;
    logic safe_search_trigger;              // not used


    ap_cam #(
        .WIDTH(HV_WIDTH),
        .DEPTH(NUM_CONTEXTS),
        .MAX_MISMATCH(MAX_MISMATCH),
        .ADDR_WIDTH(context_ADDR_WIDTH),
        .DIST_WIDTH(DIST_WIDTH)
    ) context_ap_cam_inst (
        .i_clk(clk),
        .i_rst_n(rst_n),

        .i_we(i_train_mode && i_train_wr_context),
        .i_waddr(i_train_context_addr),
        .i_wdata(i_train_context_data),

        .i_search_en(search_trigger),
        .i_search_data(i_context_hv),

        .o_dist(context_distances)
    );

    ap_cam #(
        .WIDTH(HV_WIDTH),
        .DEPTH(NUM_EVENTS),
        .MAX_MISMATCH(MAX_MISMATCH),
        .ADDR_WIDTH(event_ADDR_WIDTH),
        .DIST_WIDTH(DIST_WIDTH)
    ) event_ap_cam_inst (
        .i_clk(clk),
        .i_rst_n(rst_n),

        .i_we(i_train_mode && i_train_wr_event),
        .i_waddr(i_train_event_addr),
        .i_wdata(i_train_event_data),

        .i_search_en(search_trigger),
        .i_search_data(i_event_hv),

        .o_dist(event_distances)
    );

    // top-1 context
    sort_comparator #(
        .DEPTH(NUM_CONTEXTS),
        .DIST_WIDTH(DIST_WIDTH),
        .ADDR_WIDTH(context_ADDR_WIDTH),
        .WIDTH(HV_WIDTH),
        .TOP_K(1)
    ) context_sort_inst (
        .i_clk(clk),
        .i_rst_n(rst_n),
        .i_start(context_sort_start),
        .i_dist_(context_voltages),
        .o_sorted_addrs(context_sorted_addrs),
        .o_sorted_dists(context_sorted_dists),
        .o_sorted_one_hot(o_context_onehot),
        .o_done(context_sort_done),
        .o_ready(context_sort_ready)
    );


    // top-1 event
    sort_comparator #(
        .DEPTH(NUM_EVENTS),
        .DIST_WIDTH(DIST_WIDTH),
        .ADDR_WIDTH(event_ADDR_WIDTH),
        .WIDTH(HV_WIDTH),
        .TOP_K(1)
    ) event_sort_inst (
        .i_clk(clk),
        .i_rst_n(rst_n),
        .i_start(event_sort_start),
        .i_dist_(event_voltages),
        .o_sorted_addrs(event_sorted_addrs),
        .o_sorted_dists(event_sorted_dists),
        .o_sorted_one_hot(o_event_onehot),
        .o_done(event_sort_done),
        .o_ready(event_sort_ready)
    );

    // distance -> score; anything past the threshold scores 0
    always_comb begin
        for (int i = 0; i < NUM_CONTEXTS; i++) begin
            if (context_distances[i] <= MAX_MISMATCH) begin
                context_voltages[i] = MAX_VOLTAGE - context_distances[i];
            end else begin
                context_voltages[i] = '0;
            end
        end
    end

    always_comb begin
        for (int i = 0; i < NUM_EVENTS; i++) begin
            if (event_distances[i] <= MAX_MISMATCH) begin
                event_voltages[i] = MAX_VOLTAGE - event_distances[i];
            end else begin
                event_voltages[i] = '0;
            end
        end
    end

    assign o_event_distances = event_distances;
    always_comb begin
        for (int i = 0; i < NUM_EVENTS; i++) begin
            o_event_multi_hot[i] = (event_distances[i] <= MAX_MISMATCH);
        end
    end

    assign search_trigger = i_valid && !i_train_mode;
    assign safe_search_trigger = search_trigger && !i_train_mode && !i_train_wr_context && !i_train_wr_event;

    // remember the request until both sorters have taken it
    always_ff @(posedge clk ) begin
        if (!rst_n) begin
            valid_reg <= 1'b0;
        end else begin
            if (search_trigger) begin
                valid_reg <= 1'b1;
            end else if (context_sort_start && event_sort_start) begin
                valid_reg <= 1'b0;
            end
        end
    end

    // both sorters start together, on the cycle after the CAM search
    assign context_sort_start = valid_reg && context_sort_ready && event_sort_ready;
    assign event_sort_start = valid_reg && context_sort_ready && event_sort_ready;

    // The context sorter (16 entries) finishes before the event sorter, so
    // hold each done until both are in. ANDing the raw one-cycle dones would
    // never fire.
    logic ctx_done_latch, evt_done_latch;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            ctx_done_latch <= 1'b0;
            evt_done_latch <= 1'b0;
        end else begin
            if (ctx_done_latch && evt_done_latch) begin
                ctx_done_latch <= 1'b0;
                evt_done_latch <= 1'b0;
            end else begin
                if (context_sort_done) ctx_done_latch <= 1'b1;
                if (event_sort_done)   evt_done_latch <= 1'b1;
            end
        end
    end

    // one-cycle pulse: the latches clear on the next edge
    assign o_valid = ctx_done_latch && evt_done_latch && !i_train_mode;

    assign o_ready = context_sort_ready && event_sort_ready && !valid_reg
                     && !ctx_done_latch && !evt_done_latch && !i_train_mode;

    // one-hot -> index
    always_comb begin
        o_context_idx = '0;
        for (int i = 0; i < NUM_CONTEXTS; i++) begin
            if (o_context_onehot[i]) begin
                o_context_idx = i[context_ADDR_WIDTH-1:0];
            end
        end
    end

    always_comb begin
        o_event_idx = '0;
        for (int i = 0; i < NUM_EVENTS; i++) begin
            if (o_event_onehot[i]) begin
                o_event_idx = i[event_ADDR_WIDTH-1:0];
            end
        end
    end


endmodule
