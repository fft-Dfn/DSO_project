// -----------------------------------------------------------------------------
// VGA_top
// -----------------------------------------------------------------------------
// Purpose:
//   Thin wrapper around vga_stream_player.
//   Keeps the external VGA-facing interface stable.
// -----------------------------------------------------------------------------
module VGA_top #(
    parameter DATA_W   = 8,
    parameter ADDR_W   = 10,
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    input  wire                  clk_25m,
    input  wire                  rst_n,

    // Inputs from pingpong_buffer read path.
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
    input  wire [2:0]            view_ch_sel,
    input  wire                  trig_mode,
    input  wire                  trig_edge,
    input  wire [7:0]            trig_level,
    input  wire [31:0]           sample_div,
    input  wire [2:0]            sel_trig,
    output wire                  rd_frame_done,

    // VGA outputs.
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
        .view_ch_sel            (view_ch_sel),
        .trig_mode              (trig_mode),
        .trig_edge              (trig_edge),
        .trig_level             (trig_level),
        .sample_div             (sample_div),
        .sel_trig               (sel_trig),
        .rd_frame_done          (rd_frame_done),
        .frame_start            (frame_start_unused),
        .hsync                  (hsync),
        .vsync                  (vsync),
        .rgb565                 (rgb565)
    );

endmodule
