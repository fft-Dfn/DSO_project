module waveform_renderer #(
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    input  wire         clk_pix,
    input  wire         de,
    input  wire [10:0]  pix_x,
    input  wire [9:0]   pix_y,

    input  wire         sample_valid,
    input  wire [7:0]   sample_ch1,
    input  wire [7:0]   sample_ch2,
    input  wire [7:0]   sample_ch3,
    input  wire [7:0]   sample_ch4,
    input  wire [7:0]   debug_status,

    output reg  [15:0]  rgb565
);

    localparam [15:0] COLOR_BLACK   = 16'h0000;
    localparam [15:0] COLOR_GRID    = 16'h2104;
    localparam [15:0] COLOR_CENTER  = 16'h39E7;
    localparam [15:0] COLOR_CH1     = 16'hF800;
    localparam [15:0] COLOR_CH2     = 16'h07E0;
    localparam [15:0] COLOR_CH3     = 16'h001F;
    localparam [15:0] COLOR_CH4     = 16'hFFE0;
    localparam [15:0] COLOR_DBG_ON  = 16'h07E0;
    localparam [15:0] COLOR_DBG_OFF = 16'h8000;

    wire [9:0] y_ch1 = (V_ACTIVE - 1) - ((sample_ch1 * (V_ACTIVE - 1)) >> 8);
    wire [9:0] y_ch2 = (V_ACTIVE - 1) - ((sample_ch2 * (V_ACTIVE - 1)) >> 8);
    wire [9:0] y_ch3 = (V_ACTIVE - 1) - ((sample_ch3 * (V_ACTIVE - 1)) >> 8);
    wire [9:0] y_ch4 = (V_ACTIVE - 1) - ((sample_ch4 * (V_ACTIVE - 1)) >> 8);

    reg [9:0] prev_y_ch1, prev_y_ch2, prev_y_ch3, prev_y_ch4;
    reg       prev_valid;

    reg ch1_hit, ch2_hit, ch3_hit, ch4_hit;
    reg grid_hit, center_hit;
    reg dbg_hit, dbg_on;

    function in_segment;
        input [9:0] y;
        input [9:0] y0;
        input [9:0] y1;
        begin
            if (y0 <= y1)
                in_segment = (y >= y0) && (y <= y1);
            else
                in_segment = (y >= y1) && (y <= y0);
        end
    endfunction

    always @(posedge clk_pix) begin
        if (!de) begin
            prev_valid <= 1'b0;
        end else if (sample_valid) begin
            if (pix_x == 11'd0)
                prev_valid <= 1'b0;
            else
                prev_valid <= 1'b1;

            prev_y_ch1 <= y_ch1;
            prev_y_ch2 <= y_ch2;
            prev_y_ch3 <= y_ch3;
            prev_y_ch4 <= y_ch4;
        end
    end

    always @(*) begin
        grid_hit   = ((pix_x % 80) == 0) || ((pix_y % 60) == 0);
        center_hit = (pix_x == (H_ACTIVE >> 1)) || (pix_y == (V_ACTIVE >> 1));
        dbg_hit    = 1'b0;
        dbg_on     = 1'b0;

        // Draw point + interpolated segment from x-1 to x.
        ch1_hit = sample_valid && ((pix_y == y_ch1) || ((pix_x != 11'd0) && prev_valid && in_segment(pix_y, prev_y_ch1, y_ch1)));
        ch2_hit = sample_valid && ((pix_y == y_ch2) || ((pix_x != 11'd0) && prev_valid && in_segment(pix_y, prev_y_ch2, y_ch2)));
        ch3_hit = sample_valid && ((pix_y == y_ch3) || ((pix_x != 11'd0) && prev_valid && in_segment(pix_y, prev_y_ch3, y_ch3)));
        ch4_hit = sample_valid && ((pix_y == y_ch4) || ((pix_x != 11'd0) && prev_valid && in_segment(pix_y, prev_y_ch4, y_ch4)));

        // Top-left 8 debug boxes: bit0..bit7.
        if ((pix_y >= 10) && (pix_y < 22)) begin
            if      ((pix_x >= 10)  && (pix_x < 22))  begin dbg_hit = 1'b1; dbg_on = debug_status[0]; end
            else if ((pix_x >= 24)  && (pix_x < 36))  begin dbg_hit = 1'b1; dbg_on = debug_status[1]; end
            else if ((pix_x >= 38)  && (pix_x < 50))  begin dbg_hit = 1'b1; dbg_on = debug_status[2]; end
            else if ((pix_x >= 52)  && (pix_x < 64))  begin dbg_hit = 1'b1; dbg_on = debug_status[3]; end
            else if ((pix_x >= 66)  && (pix_x < 78))  begin dbg_hit = 1'b1; dbg_on = debug_status[4]; end
            else if ((pix_x >= 80)  && (pix_x < 92))  begin dbg_hit = 1'b1; dbg_on = debug_status[5]; end
            else if ((pix_x >= 94)  && (pix_x < 106)) begin dbg_hit = 1'b1; dbg_on = debug_status[6]; end
            else if ((pix_x >= 108) && (pix_x < 120)) begin dbg_hit = 1'b1; dbg_on = debug_status[7]; end
        end

        if (!de) begin
            rgb565 = COLOR_BLACK;
        end else if (dbg_hit) begin
            rgb565 = dbg_on ? COLOR_DBG_ON : COLOR_DBG_OFF;
        end else if (ch1_hit) begin
            rgb565 = COLOR_CH1;
        end else if (ch2_hit) begin
            rgb565 = COLOR_CH2;
        end else if (ch3_hit) begin
            rgb565 = COLOR_CH3;
        end else if (ch4_hit) begin
            rgb565 = COLOR_CH4;
        end else if (center_hit) begin
            rgb565 = COLOR_CENTER;
        end else if (grid_hit) begin
            rgb565 = COLOR_GRID;
        end else begin
            rgb565 = COLOR_BLACK;
        end
    end

endmodule
