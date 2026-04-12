// -----------------------------------------------------------------------------
// vga_stream_player
// -----------------------------------------------------------------------------
// Purpose:
//   Converts VGA scan timing into BRAM read addresses and feeds renderer via
//   an AXI4-Stream-like pixel bus (tvalid/tdata/tlast/tuser).
//
// Key Signals (English):
//   - frame_base_addr_latched: per-display-frame frozen read base.
//   - raddr: BRAM read address (prefetched for sync read latency).
//   - axis_tuser_d1: first active pixel of frame.
//   - axis_tlast_d1: last active pixel of each line.
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

    output reg                   rd_frame_done,
    output wire                  frame_start,
    output wire                  hsync,
    output wire                  vsync,
    output wire [15:0]           rgb565
);

    wire         de_timing;
    wire [10:0]  pix_x_timing;
    wire [9:0]   pix_y_timing;

    reg          de_d1;
    reg [10:0]   pix_x_d1;
    reg [9:0]    pix_y_d1;

    // AXIS stream for renderer (aligned to d1 timing)
    reg          axis_tvalid_d1;
    reg [31:0]   axis_tdata_d1;
    reg          axis_tlast_d1;
    reg          axis_tuser_d1;

    reg [ADDR_W-1:0] frame_base_addr_latched;

    wire last_active_pixel = de_d1 &&
                             (pix_x_d1 == H_ACTIVE - 1) &&
                             (pix_y_d1 == V_ACTIVE - 1);

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

    always @(posedge clk_25m or negedge rst_n) begin
        if (!rst_n) begin
            frame_base_addr_latched <= {ADDR_W{1'b0}};
        end else if (frame_start) begin
            // Freeze this frame's sample base at display frame boundary.
            frame_base_addr_latched <= active_frame_start_addr;
        end
    end

    always @(posedge clk_25m or negedge rst_n) begin
        if (!rst_n) begin
            raddr         <= {ADDR_W{1'b0}};
            de_d1         <= 1'b0;
            pix_x_d1      <= 11'd0;
            pix_y_d1      <= 10'd0;
            axis_tvalid_d1 <= 1'b0;
            axis_tdata_d1 <= 32'd0;
            axis_tlast_d1 <= 1'b0;
            axis_tuser_d1 <= 1'b0;
            rd_frame_done <= 1'b0;
        end else begin
            // Timing and stream metadata pipeline.
            de_d1    <= de_timing;
            pix_x_d1 <= pix_x_timing;
            pix_y_d1 <= pix_y_timing;

            axis_tvalid_d1 <= de_timing && frame_valid;
            axis_tdata_d1  <= {rdata_ch4, rdata_ch3, rdata_ch2, rdata_ch1};
            axis_tlast_d1  <= de_timing && (pix_x_timing == H_ACTIVE - 1);
            axis_tuser_d1  <= de_timing && (pix_x_timing == 11'd0) && (pix_y_timing == 10'd0);

            // BRAM read address prefetch for 1-cycle read latency.
            if (frame_valid) begin
                if (de_timing) begin
                    if (pix_x_timing == H_ACTIVE - 1)
                        raddr <= frame_base_addr_latched + pix_x_timing[ADDR_W-1:0];
                    else
                        raddr <= frame_base_addr_latched +
                                 pix_x_timing[ADDR_W-1:0] +
                                 {{(ADDR_W-1){1'b0}}, 1'b1};
                end else begin
                    // During blanking keep sample[0] primed for next active line.
                    raddr <= frame_base_addr_latched;
                end
            end else begin
                raddr <= {ADDR_W{1'b0}};
            end

            rd_frame_done <= last_active_pixel;
        end
    end

    wire renderer_axis_tready_unused;

    waveform_renderer #(
        .H_ACTIVE(H_ACTIVE),
        .V_ACTIVE(V_ACTIVE)
    ) u_waveform_renderer (
        .clk_pix      (clk_25m),
        .de           (de_d1),
        .pix_x        (pix_x_d1),
        .pix_y        (pix_y_d1),
        .s_axis_tvalid(axis_tvalid_d1),
        .s_axis_tready(renderer_axis_tready_unused),
        .s_axis_tdata (axis_tdata_d1),
        .s_axis_tlast (axis_tlast_d1),
        .s_axis_tuser (axis_tuser_d1),
        .debug_status (dbg_status),
        .rgb565       (rgb565)
    );

endmodule
