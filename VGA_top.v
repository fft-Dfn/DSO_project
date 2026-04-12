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
        .rd_frame_done          (rd_frame_done),
        .frame_start            (frame_start_unused),
        .hsync                  (hsync),
        .vsync                  (vsync),
        .rgb565                 (rgb565)
    );

endmodule
