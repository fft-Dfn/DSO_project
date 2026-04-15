// -----------------------------------------------------------------------------
// VGA_top
// -----------------------------------------------------------------------------
// Thin wrapper that keeps external VGA-facing ports stable while delegating
// frame fetch, cacheing, and rendering to vga_stream_player.
// -----------------------------------------------------------------------------

module VGA_top #(
    parameter DATA_W   = 8,
    parameter ADDR_W   = 10,
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    input  wire                  clk_25m,
    input  wire                  rst_n,

    input  wire                  frame_valid,
    input  wire [ADDR_W-1:0]     active_frame_start_addr,
    output wire [ADDR_W-1:0]     raddr,
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
    output wire [ADDR_W-1:0]     flash_view_addr,
    input  wire [DATA_W-1:0]     flash_view_sample,
    input  wire                  flash_view_valid,
    input  wire [23:0]           flash_jedec_id,
    input  wire                  flash_jedec_valid,
    output wire                  rd_frame_done,
    output wire                  fetch_sample_valid,
    output wire [ADDR_W-1:0]     fetch_sample_idx,
    output wire [31:0]           fetch_sample_packed,
    output wire                  fetch_frame_done,

    output wire                  hsync,
    output wire                  vsync,
    output wire [15:0]           rgb565

);

    wire frame_start_unused;

    vga_stream_player #(
        .DATA_W  (DATA_W),
        .ADDR_W  (ADDR_W),
        .H_ACTIVE(H_ACTIVE),
        .V_ACTIVE(V_ACTIVE)
    ) u_vga_stream_player (
        .clk_25m                (clk_25m),
        .rst_n                  (rst_n),
        .frame_valid            (frame_valid),
        .active_frame_start_addr(active_frame_start_addr),
        .raddr                  (raddr),
        .rdata_ch1              (rdata_ch1),
        .rdata_ch2              (rdata_ch2),
        .rdata_ch3              (rdata_ch3),
        .rdata_ch4              (rdata_ch4),
        .dbg_status             (dbg_status),
        .ui_page                (ui_page),
        .ui_cursor              (ui_cursor),
        .ui_curr_edit_mode      (ui_curr_edit_mode),
        .ui_curr_edit_value     (ui_curr_edit_value),
        .ui_active_src_sel      (ui_active_src_sel),
        .trig_mode              (trig_mode),
        .trig_edge              (trig_edge),
        .trig_level             (trig_level),
        .sample_div             (sample_div),
        .sel_trig               (sel_trig),
        .flash_ui_state         (flash_ui_state),
        .flash_view_enable      (flash_view_enable),
        .flash_view_addr        (flash_view_addr),
        .flash_view_sample      (flash_view_sample),
        .flash_view_valid       (flash_view_valid),
        .flash_jedec_id         (flash_jedec_id),
        .flash_jedec_valid      (flash_jedec_valid),
        .rd_frame_done          (rd_frame_done),
        .fetch_sample_valid     (fetch_sample_valid),
        .fetch_sample_idx       (fetch_sample_idx),
        .fetch_sample_packed    (fetch_sample_packed),
        .fetch_frame_done       (fetch_frame_done),
        .frame_start            (frame_start_unused),
        .hsync                  (hsync),
        .vsync                  (vsync),
        .rgb565                 (rgb565)
    );

endmodule
