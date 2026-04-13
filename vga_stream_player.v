// -----------------------------------------------------------------------------
// vga_stream_player
// -----------------------------------------------------------------------------
// Purpose:
//   1) Build 1024-sample -> 512-column min/max cache in read clock domain.
//   2) Feed renderer with one min/max column per active waveform pixel.
// -----------------------------------------------------------------------------
module vga_stream_player #(
    parameter DATA_W   = 8,
    parameter ADDR_W   = 10,
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    input  wire                  clk_25m,
    input  wire                  rst_n,

    // Frame source (from pingpong_buffer)
    input  wire                  frame_valid,
    input  wire [ADDR_W-1:0]     active_frame_start_addr,
    output reg  [ADDR_W-1:0]     raddr,
    input  wire [DATA_W-1:0]     rdata_ch1,
    input  wire [DATA_W-1:0]     rdata_ch2,
    input  wire [DATA_W-1:0]     rdata_ch3,
    input  wire [DATA_W-1:0]     rdata_ch4,

    input  wire [65:0]           dbg_status,

    // UI/control signals for renderer
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
    localparam SAMPLE_CNT = (1 << ADDR_W); // 1024

    wire         de_timing;
    wire [10:0]  pix_x_timing;
    wire [9:0]   pix_y_timing;

    reg          de_d1;
    reg [10:0]   pix_x_d1;
    reg [9:0]    pix_y_d1;

    // AXIS-like stream for renderer: one beat = one wave column min/max (4 channels packed).
    reg          axis_tvalid_d1;
    reg [63:0]   axis_tdata_d1;
    reg          axis_tlast_d1;
    reg          axis_tuser_d1;

    reg [ADDR_W-1:0] frame_base_addr_latched;

    // Two cache banks for atomic per-frame swap.
    // Packed layout per column:
    // {ch4_max,ch4_min,ch3_max,ch3_min,ch2_max,ch2_min,ch1_max,ch1_min}
    (* ram_style = "block" *) reg [63:0] minmax_cache0 [0:WAVE_W-1];
    (* ram_style = "block" *) reg [63:0] minmax_cache1 [0:WAVE_W-1];
    reg        active_cache_sel;
    reg        fill_cache_sel;
    reg        cache_valid;
    reg        fill_done;
    reg [8:0]  cache_rd_addr;
    reg [63:0] cache0_rd_data;
    reg [63:0] cache1_rd_data;

    // Fetch engine (reads 1024 samples from BRAM port B every display frame).
    reg                  fetching;
    reg [10:0]           fetch_req_idx;
    reg [ADDR_W-1:0]     fetch_base_addr;
    reg                  fetch_resp_valid;
    reg [10:0]           fetch_resp_idx;
    reg [31:0]           sample_even_hold;

    wire [31:0] sample_packed_live = {rdata_ch4, rdata_ch3, rdata_ch2, rdata_ch1};
    wire [31:0] sample_packed_view = {24'd0, flash_view_sample};
    wire [31:0] sample_packed_fetch =
        (flash_view_enable && flash_view_valid) ? sample_packed_view : sample_packed_live;
    wire cache_write_fire = fetch_resp_valid && fetch_resp_idx[0];
    wire [8:0] cache_write_addr = fetch_resp_idx[9:1];
    wire [63:0] cache_write_data = pack_minmax_pair(sample_even_hold, sample_packed_fetch);
    wire cache0_we = cache_write_fire && ~fill_cache_sel;
    wire cache1_we = cache_write_fire &&  fill_cache_sel;

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

    // Cache memories: synchronous read + synchronous write for BRAM inference.
    always @(posedge clk_25m) begin
        if (in_wave_area_timing)
            cache_rd_addr <= wave_col_idx_timing;
        else
            cache_rd_addr <= 9'd0;

        cache0_rd_data <= minmax_cache0[cache_rd_addr];
        if (cache0_we)
            minmax_cache0[cache_write_addr] <= cache_write_data;

        cache1_rd_data <= minmax_cache1[cache_rd_addr];
        if (cache1_we)
            minmax_cache1[cache_write_addr] <= cache_write_data;

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

    // Fetch + cache FSM.
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
            fetch_resp_valid        <= 1'b0;
            fetch_resp_idx          <= 11'd0;
            sample_even_hold        <= 32'd0;
            fetch_sample_valid      <= 1'b0;
            fetch_sample_idx        <= {ADDR_W{1'b0}};
            fetch_sample_packed     <= 32'd0;
            fetch_frame_done        <= 1'b0;
        end else begin
            fetch_sample_valid <= 1'b0;
            fetch_frame_done   <= 1'b0;
            // Consume previous-cycle fetch response (1-cycle BRAM read latency).
            if (fetch_resp_valid) begin
                fetch_sample_valid  <= 1'b1;
                fetch_sample_idx    <= fetch_resp_idx[ADDR_W-1:0];
                fetch_sample_packed <= sample_packed_live;

                if (!fetch_resp_idx[0]) begin
                    sample_even_hold <= sample_packed_fetch;
                end

                if (fetch_resp_idx == SAMPLE_CNT - 1) begin
                    fill_done <= 1'b1;
                    fetch_frame_done <= 1'b1;
                end
            end

            // Default: no new response tag unless a request is launched.
            fetch_resp_valid <= 1'b0;

            // Frame boundary:
            // 1) swap cache bank if previous fill finished
            // 2) latch new frame base
            // 3) start filling next cache bank
            if (frame_start) begin
                frame_base_addr_latched <= active_frame_start_addr;

                if (fill_done) begin
                    active_cache_sel <= fill_cache_sel;
                    fill_cache_sel   <= ~fill_cache_sel;
                    fill_done        <= 1'b0;
                    cache_valid      <= 1'b1;
                end

                if (frame_source_valid) begin
                    fetching        <= 1'b1;
                    fetch_req_idx   <= 11'd0;
                    fetch_base_addr <= flash_view_enable ? {ADDR_W{1'b0}} : active_frame_start_addr;
                end else begin
                    fetching <= 1'b0;
                end
            end

            // Launch one read request per clock while fetching.
            if (fetching) begin
                raddr            <= fetch_base_addr + fetch_req_idx[ADDR_W-1:0];
                flash_view_addr  <= fetch_req_idx[ADDR_W-1:0];
                fetch_resp_valid <= 1'b1;
                fetch_resp_idx   <= fetch_req_idx;

                if (fetch_req_idx == SAMPLE_CNT - 1) begin
                    fetching <= 1'b0;
                end else begin
                    fetch_req_idx <= fetch_req_idx + 1'b1;
                end
            end else begin
                raddr <= frame_base_addr_latched;
                flash_view_addr <= {ADDR_W{1'b0}};
            end
        end
    end

    // Timing + renderer stream pipeline.
    always @(posedge clk_25m or negedge rst_n) begin
        if (!rst_n) begin
            de_d1          <= 1'b0;
            pix_x_d1       <= 11'd0;
            pix_y_d1       <= 10'd0;
            in_wave_area_d1 <= 1'b0;
            axis_tvalid_d1 <= 1'b0;
            axis_tdata_d1  <= 64'd0;
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
