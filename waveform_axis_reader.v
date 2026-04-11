// -----------------------------------------------------------------------------
// waveform_axis_reader
// -----------------------------------------------------------------------------
// Purpose:
//   Optional reader that turns ping-pong BRAM samples into AXI4-Stream pixels.
//   (Current top uses vga_stream_player directly; this module is kept for reuse.)
// -----------------------------------------------------------------------------
module waveform_axis_reader #(
    parameter DATA_W   = 8,
    parameter ADDR_W   = 10,
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    input  wire                  clk_read,
    input  wire                  rst_n,

    // Frame switch control from display timing domain
    input  wire                  disp_frame_start,
    input  wire                  frame_valid,
    input  wire [ADDR_W-1:0]     active_frame_start_addr,

    // Ping-pong BRAM read port
    output reg  [ADDR_W-1:0]     raddr,
    input  wire [DATA_W-1:0]     rdata_ch1,
    input  wire [DATA_W-1:0]     rdata_ch2,
    input  wire [DATA_W-1:0]     rdata_ch3,
    input  wire [DATA_W-1:0]     rdata_ch4,

    // AXI4-Stream master (video active area only)
    output wire                  m_axis_tvalid,
    input  wire                  m_axis_tready,
    output wire [31:0]           m_axis_tdata,
    output wire                  m_axis_tlast,
    output wire                  m_axis_tuser
);

    reg                  stream_armed;
    reg [ADDR_W-1:0]     frame_base_addr_latched;
    reg [10:0]           x_cnt;
    reg [9:0]            y_cnt;

    wire first_pixel = (x_cnt == 11'd0) && (y_cnt == 10'd0);
    wire end_of_line = (x_cnt == H_ACTIVE - 1);
    assign m_axis_tvalid = stream_armed;
    assign m_axis_tdata  = {rdata_ch4, rdata_ch3, rdata_ch2, rdata_ch1};
    assign m_axis_tuser  = first_pixel;
    assign m_axis_tlast  = end_of_line;

    always @(posedge clk_read or negedge rst_n) begin
        if (!rst_n) begin
            stream_armed             <= 1'b0;
            frame_base_addr_latched  <= {ADDR_W{1'b0}};
            raddr                    <= {ADDR_W{1'b0}};
            x_cnt                    <= 11'd0;
            y_cnt                    <= 10'd0;
        end else begin
            // Lock frame source only on display frame boundary to avoid tearing.
            if (disp_frame_start) begin
                x_cnt <= 11'd0;
                y_cnt <= 10'd0;

                if (frame_valid) begin
                    stream_armed            <= 1'b1;
                    frame_base_addr_latched <= active_frame_start_addr;
                    raddr                   <= active_frame_start_addr;
                end else begin
                    stream_armed <= 1'b0;
                    raddr        <= {ADDR_W{1'b0}};
                end
            end else if (stream_armed) begin
                if (m_axis_tready) begin
                    if (end_of_line) begin
                        x_cnt <= 11'd0;
                        if (y_cnt == V_ACTIVE - 1)
                            y_cnt <= 10'd0;
                        else
                            y_cnt <= y_cnt + 10'd1;

                        // Park on x=0 sample so next line starts without bubble.
                        raddr <= frame_base_addr_latched;
                    end else begin
                        x_cnt <= x_cnt + 11'd1;
                        raddr <= frame_base_addr_latched +
                                 x_cnt[ADDR_W-1:0] +
                                 {{(ADDR_W-1){1'b0}}, 1'b1};
                    end
                end
            end
        end
    end

endmodule
