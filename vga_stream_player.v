// -----------------------------------------------------------------------------
// vga_stream_player
// -----------------------------------------------------------------------------
// Data path summary:
// 1) Per display frame, fetches 1024 samples from active frame source.
// 2) Pairs two adjacent samples into one min/max column (512 columns).
// 3) Writes columns into fill cache bank and swaps cache banks only on frame boundary.
//
// Timing-critical notes:
// - fetch_req_idx_d1/d2/d3 form a response-tag pipeline that aligns sample indices to data.
// - live path and flash-view path use different effective latencies and therefore different tags.
// - This alignment avoids odd/even pair corruption and visible seam drift.
// -----------------------------------------------------------------------------

module vga_stream_player #(
    parameter DATA_W   = 8,
    parameter ADDR_W   = 10,
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    input  wire                  clk_25m,
    input  wire                  rst_n,

    input  wire                  frame_valid,
    input  wire [ADDR_W-1:0]     active_frame_start_addr,
    output reg  [ADDR_W-1:0]     raddr,
    input  wire [DATA_W-1:0]     rdata_ch1,
    input  wire [DATA_W-1:0]     rdata_ch2,
    input  wire [DATA_W-1:0]     rdata_ch3,
    input  wire [DATA_W-1:0]     rdata_ch4,

    input  wire [65:0]           dbg_status,

    input  wire [3:0]            ui_page,
    input  wire [3:0]            ui_cursor,
    input  wire                  ui_curr_edit_mode,
    input  wire [3:0]            ui_curr_edit_value,
    input  wire [2:0]            ui_active_src_sel,
    input  wire                  trig_mode,
    input  wire                  trig_edge,
    input  wire [7:0]            trig_level,
    input  wire [31:0]           sample_div,
    input  wire [2:0]            sel_trig,
    input  wire [2:0]            flash_ui_state,
    input  wire                  flash_view_enable,
    output reg  [ADDR_W-1:0]     flash_view_addr,
    input  wire [DATA_W-1:0]     flash_view_sample,
    input  wire                  flash_view_valid,
    input  wire [23:0]           flash_jedec_id,
    input  wire                  flash_jedec_valid,

    output reg                   rd_frame_done,
    output reg                   fetch_sample_valid,
    output reg  [ADDR_W-1:0]     fetch_sample_idx,
    output reg  [31:0]           fetch_sample_packed,
    output reg                   fetch_frame_done,
    output wire                  frame_start,
    output wire                  hsync,
    output wire                  vsync,
    output wire [15:0]           rgb565
);

    localparam UI_W       = 11'd128;
    localparam WAVE_W     = 11'd512;
    localparam SAMPLE_CNT = (1 << ADDR_W);

    wire         de_timing;
    wire [10:0]  pix_x_timing;
    wire [9:0]   pix_y_timing;

    reg          de_d1;
    reg [10:0]   pix_x_d1;
    reg [9:0]    pix_y_d1;

    reg          axis_tvalid_d1;
    reg [63:0]   axis_tdata_d1;
    reg [63:0]   axis_tprev_d1;
    reg          axis_tlast_d1;
    reg          axis_tuser_d1;

    // Base address frozen per display frame to keep a stable read window.
    reg [ADDR_W-1:0] frame_base_addr_latched;

    // cache0/cache1: ping-pong min/max column caches for current rendering.
    // persist_cache: previous-frame copy for persistence overlay.
    (* ram_style = "block" *) reg [63:0] minmax_cache0 [0:WAVE_W-1];
    (* ram_style = "block" *) reg [63:0] minmax_cache1 [0:WAVE_W-1];
    (* ram_style = "block" *) reg [63:0] persist_cache [0:WAVE_W-1];
    reg        active_cache_sel;
    reg        fill_cache_sel;
    reg        cache_valid;
    reg        fill_done;
    reg [8:0]  cache0_rd_addr;
    reg [8:0]  cache1_rd_addr;
    reg [8:0]  prev_cache_rd_addr;
    reg [63:0] cache0_rd_data;
    reg [63:0] cache1_rd_data;
    reg [63:0] prev_cache_rd_data;

    reg        persist_copy_active;
    reg        persist_copy_src_sel;
    reg [8:0]  persist_copy_req_idx;
    reg        persist_copy_resp_valid;
    reg [8:0]  persist_copy_resp_idx;

    // Fetch-side request generator and response tag pipeline.
    reg                  fetching;
    reg [10:0]           fetch_req_idx;
    reg [ADDR_W-1:0]     fetch_base_addr;
    reg                  fetch_req_valid_d1;
    reg [10:0]           fetch_req_idx_d1;
    reg                  fetch_req_valid_d2;
    reg [10:0]           fetch_req_idx_d2;
    reg                  fetch_req_valid_d3;
    reg [10:0]           fetch_req_idx_d3;
    reg                  fetch_use_flash_view;
    reg [31:0]           sample_even_hold;
    reg                  fetch_start_pending;
    reg [ADDR_W-1:0]     fetch_start_base_addr;

    wire [31:0] sample_packed_live = {rdata_ch4, rdata_ch3, rdata_ch2, rdata_ch1};
    wire [31:0] sample_packed_view = {24'd0, flash_view_sample};
    wire [31:0] sample_packed_fetch =
        (flash_view_enable && flash_view_valid) ? sample_packed_view : sample_packed_live;

    // Response tag selection:
    // - live ping-pong BRAM path: use d3
    // - flash view path         : use d2
    wire fetch_resp_valid_now = fetch_use_flash_view ? fetch_req_valid_d2 : fetch_req_valid_d3;
    wire [10:0] fetch_resp_idx_now = fetch_use_flash_view ? fetch_req_idx_d2 : fetch_req_idx_d3;
    wire cache_write_fire = fetch_resp_valid_now && fetch_resp_idx_now[0];
    wire [8:0] cache_write_addr = fetch_resp_idx_now[9:1];
    wire [63:0] cache_write_data = pack_minmax_pair(sample_even_hold, sample_packed_fetch);
    wire cache0_we = cache_write_fire && ~fill_cache_sel;
    wire cache1_we = cache_write_fire &&  fill_cache_sel;
    wire persist_copy_fire = persist_copy_resp_valid;
    wire [63:0] persist_copy_data = persist_copy_src_sel ? cache1_rd_data : cache0_rd_data;

    wire last_active_pixel = de_d1 &&
                             (pix_x_d1 == H_ACTIVE - 1) &&
                             (pix_y_d1 == V_ACTIVE - 1);
    wire frame_source_valid = (flash_view_enable && flash_view_valid) || frame_valid;

    wire in_wave_area_timing = de_timing &&
                               (pix_x_timing >= UI_W) &&
                               (pix_x_timing < (UI_W + WAVE_W));
    wire [10:0] wave_col_idx_timing_ext = pix_x_timing - UI_W;
    wire [8:0] wave_col_idx_timing = wave_col_idx_timing_ext[8:0];

    function [7:0] min8;
        input [7:0] a;
        input [7:0] b;
        begin
            min8 = (a <= b) ? a : b;
        end
    endfunction

    function [7:0] max8;
        input [7:0] a;
        input [7:0] b;
        begin
            max8 = (a >= b) ? a : b;
        end
    endfunction

    function [63:0] pack_minmax_pair;
        input [31:0] even_s;
        input [31:0] odd_s;
        reg [7:0] e1, e2, e3, e4;
        reg [7:0] o1, o2, o3, o4;
        begin
            e1 = even_s[7:0];
            e2 = even_s[15:8];
            e3 = even_s[23:16];
            e4 = even_s[31:24];
            o1 = odd_s[7:0];
            o2 = odd_s[15:8];
            o3 = odd_s[23:16];
            o4 = odd_s[31:24];

            pack_minmax_pair = {
                max8(e4, o4), min8(e4, o4),
                max8(e3, o3), min8(e3, o3),
                max8(e2, o2), min8(e2, o2),
                max8(e1, o1), min8(e1, o1)
            };
        end
    endfunction

    reg in_wave_area_d1;

    // Synchronous cache read/write block.
    always @(posedge clk_25m) begin
        if (in_wave_area_timing) begin
            cache0_rd_addr     <= wave_col_idx_timing;
            cache1_rd_addr     <= wave_col_idx_timing;
            prev_cache_rd_addr <= wave_col_idx_timing;
        end else begin
            cache0_rd_addr     <= 9'd0;
            cache1_rd_addr     <= 9'd0;
            prev_cache_rd_addr <= 9'd0;
        end

        if (persist_copy_active) begin
            if (persist_copy_src_sel)
                cache1_rd_addr <= persist_copy_req_idx;
            else
                cache0_rd_addr <= persist_copy_req_idx;
        end

        cache0_rd_data <= minmax_cache0[cache0_rd_addr];
        if (cache0_we)
            minmax_cache0[cache_write_addr] <= cache_write_data;

        cache1_rd_data <= minmax_cache1[cache1_rd_addr];
        if (cache1_we)
            minmax_cache1[cache_write_addr] <= cache_write_data;

        prev_cache_rd_data <= persist_cache[prev_cache_rd_addr];
        if (persist_copy_fire)
            persist_cache[persist_copy_resp_idx] <= persist_copy_data;
    end

    vga_timing u_vga_timing (
        .clk_pix    (clk_25m),
        .rst_n      (rst_n),
        .hsync      (hsync),
        .vsync      (vsync),
        .de         (de_timing),
        .pix_x      (pix_x_timing),
        .pix_y      (pix_y_timing),
        .frame_start(frame_start)
    );

    // Fetch/cache control FSM:
    // - launches one read request per cycle while fetching
    // - consumes aligned responses into min/max cache
    // - swaps active/fill cache only on frame_start after fill_done
    always @(posedge clk_25m or negedge rst_n) begin
        if (!rst_n) begin
            frame_base_addr_latched <= {ADDR_W{1'b0}};
            raddr                   <= {ADDR_W{1'b0}};
            flash_view_addr         <= {ADDR_W{1'b0}};
            active_cache_sel        <= 1'b0;
            fill_cache_sel          <= 1'b1;
            cache_valid             <= 1'b0;
            fill_done               <= 1'b0;
            fetching                <= 1'b0;
            fetch_req_idx           <= 11'd0;
            fetch_base_addr         <= {ADDR_W{1'b0}};
            fetch_req_valid_d1      <= 1'b0;
            fetch_req_idx_d1        <= 11'd0;
            fetch_req_valid_d2      <= 1'b0;
            fetch_req_idx_d2        <= 11'd0;
            fetch_req_valid_d3      <= 1'b0;
            fetch_req_idx_d3        <= 11'd0;
            fetch_use_flash_view    <= 1'b0;
            sample_even_hold        <= 32'd0;
            fetch_start_pending     <= 1'b0;
            fetch_start_base_addr   <= {ADDR_W{1'b0}};
            fetch_sample_valid      <= 1'b0;
            fetch_sample_idx        <= {ADDR_W{1'b0}};
            fetch_sample_packed     <= 32'd0;
            fetch_frame_done        <= 1'b0;
            persist_copy_active     <= 1'b0;
            persist_copy_src_sel    <= 1'b0;
            persist_copy_req_idx    <= 9'd0;
            persist_copy_resp_valid <= 1'b0;
            persist_copy_resp_idx   <= 9'd0;
        end else begin
            fetch_sample_valid <= 1'b0;
            fetch_frame_done   <= 1'b0;
            persist_copy_resp_valid <= 1'b0;

            if (fetch_resp_valid_now) begin
                fetch_sample_valid  <= 1'b1;
                fetch_sample_idx    <= fetch_resp_idx_now[ADDR_W-1:0];
                fetch_sample_packed <= sample_packed_fetch;

                if (!fetch_resp_idx_now[0]) begin
                    sample_even_hold <= sample_packed_fetch;
                end

                if (fetch_resp_idx_now == SAMPLE_CNT - 1) begin
                    fill_done <= 1'b1;
                    fetch_frame_done <= 1'b1;
                end
            end

            // Shift request tags so response index matches data arrival latency.
            fetch_req_valid_d3 <= fetch_req_valid_d2;
            fetch_req_idx_d3   <= fetch_req_idx_d2;
            fetch_req_valid_d2 <= fetch_req_valid_d1;
            fetch_req_idx_d2   <= fetch_req_idx_d1;
            fetch_req_valid_d1 <= 1'b0;
            fetch_req_idx_d1   <= 11'd0;

            if (frame_start) begin
                frame_base_addr_latched <= active_frame_start_addr;

                if (fill_done) begin
                    // Snapshot previous active cache for persistence before switching.
                    persist_copy_active   <= 1'b1;
                    persist_copy_src_sel  <= active_cache_sel;
                    persist_copy_req_idx  <= 9'd0;

                    active_cache_sel <= fill_cache_sel;
                    fill_cache_sel   <= ~fill_cache_sel;
                    fill_done        <= 1'b0;
                    cache_valid      <= 1'b1;
                end

                if (frame_source_valid) begin
                    // Delay fetch launch until persistence copy is idle to avoid port conflict.
                    fetch_start_pending   <= 1'b1;
                    fetch_start_base_addr <= flash_view_enable ? {ADDR_W{1'b0}} : active_frame_start_addr;
                end else begin
                    fetching            <= 1'b0;
                    fetch_start_pending <= 1'b0;
                end
            end

            if (fetching) begin
                raddr            <= fetch_base_addr + fetch_req_idx[ADDR_W-1:0];
                flash_view_addr  <= fetch_req_idx[ADDR_W-1:0];
                fetch_req_valid_d1 <= 1'b1;
                fetch_req_idx_d1   <= fetch_req_idx;

                if (fetch_req_idx == SAMPLE_CNT - 1) begin
                    fetching <= 1'b0;
                end else begin
                    fetch_req_idx <= fetch_req_idx + 1'b1;
                end
            end else begin
                raddr <= frame_base_addr_latched;
                flash_view_addr <= {ADDR_W{1'b0}};
            end

            // One column copy per cycle from old active cache to persistence cache.
            if (persist_copy_active) begin
                persist_copy_resp_valid <= 1'b1;
                persist_copy_resp_idx   <= persist_copy_req_idx;
                if (persist_copy_req_idx == (WAVE_W - 1)) begin
                    persist_copy_active <= 1'b0;
                end else begin
                    persist_copy_req_idx <= persist_copy_req_idx + 1'b1;
                end
            end

            if (!fetching && fetch_start_pending && !persist_copy_active) begin
                fetching            <= 1'b1;
                fetch_req_idx       <= 11'd0;
                fetch_base_addr     <= fetch_start_base_addr;
                fetch_use_flash_view <= flash_view_enable;
                fetch_start_pending <= 1'b0;
            end
        end
    end

    // Timing-aligned stream output for renderer.
    always @(posedge clk_25m or negedge rst_n) begin
        if (!rst_n) begin
            de_d1          <= 1'b0;
            pix_x_d1       <= 11'd0;
            pix_y_d1       <= 10'd0;
            in_wave_area_d1 <= 1'b0;
            axis_tvalid_d1 <= 1'b0;
            axis_tdata_d1  <= 64'd0;
            axis_tprev_d1  <= 64'd0;
            axis_tlast_d1  <= 1'b0;
            axis_tuser_d1  <= 1'b0;
            rd_frame_done  <= 1'b0;
        end else begin
            de_d1    <= de_timing;
            pix_x_d1 <= pix_x_timing;
            pix_y_d1 <= pix_y_timing;
            in_wave_area_d1 <= in_wave_area_timing;

            axis_tvalid_d1 <= in_wave_area_d1 && frame_source_valid && cache_valid;
            axis_tdata_d1  <= active_cache_sel ? cache1_rd_data : cache0_rd_data;
            axis_tprev_d1  <= prev_cache_rd_data;
            axis_tlast_d1  <= in_wave_area_d1 && (pix_x_d1 == (UI_W + WAVE_W - 1));
            axis_tuser_d1  <= in_wave_area_d1 && (pix_x_d1 == UI_W) && (pix_y_d1 == 10'd0);

            rd_frame_done <= last_active_pixel;
        end
    end

    wire renderer_axis_tready_unused;

    waveform_renderer #(
        .H_ACTIVE(H_ACTIVE),
        .V_ACTIVE(V_ACTIVE)
    ) u_waveform_renderer (
        .clk_pix            (clk_25m),
        .de                 (de_d1),
        .pix_x              (pix_x_d1),
        .pix_y              (pix_y_d1),
        .s_axis_tvalid      (axis_tvalid_d1),
        .s_axis_tready      (renderer_axis_tready_unused),
        .s_axis_tdata       (axis_tdata_d1),
        .s_axis_prev_tdata  (axis_tprev_d1),
        .s_axis_tlast       (axis_tlast_d1),
        .s_axis_tuser       (axis_tuser_d1),
        .debug_status       (dbg_status),
        .ui_page            (ui_page),
        .ui_cursor          (ui_cursor),
        .ui_curr_edit_mode  (ui_curr_edit_mode),
        .ui_curr_edit_value (ui_curr_edit_value),
        .ui_active_src_sel  (ui_active_src_sel),
        .trig_mode          (trig_mode),
        .trig_edge          (trig_edge),
        .trig_level         (trig_level),
        .sample_div         (sample_div),
        .sel_trig           (sel_trig),
        .flash_ui_state     (flash_ui_state),
        .flash_view_enable  (flash_view_enable),
        .flash_jedec_id     (flash_jedec_id),
        .flash_jedec_valid  (flash_jedec_valid),
        .rgb565             (rgb565)
    );

endmodule
