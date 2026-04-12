// -----------------------------------------------------------------------------
// waveform_renderer
// -----------------------------------------------------------------------------
// Purpose:
//   Renders oscilloscope channels, grid, center cross, and in-screen debug UI.
//
// Input Style:
//   Pixel samples arrive on an AXI4-Stream-like bus:
//   - s_axis_tvalid/tdata: channel samples per active pixel
//   - s_axis_tuser: first pixel of frame
//   - s_axis_tlast: last pixel of line
//
// Debug Overlay:
//   - 8x8 debug-bit matrix at top-left (debug_status[63:0]).
//   - Extra trigger-type blocks (debug_status[64]=N, [65]=T).
//   - stripe panel at top for bank/address activity quick check.
// -----------------------------------------------------------------------------
module waveform_renderer #(
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    input  wire         clk_pix,
    input  wire         de,
    input  wire [10:0]  pix_x,
    input  wire [9:0]   pix_y,

    // AXI4-Stream video samples: one beat per active pixel.
    input  wire         s_axis_tvalid,
    output wire         s_axis_tready,
    input  wire [31:0]  s_axis_tdata,
    input  wire         s_axis_tlast,
    input  wire         s_axis_tuser,
    input  wire [65:0]  debug_status,

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
    localparam [9:0]  MAX_SEG_DY    = 10'd24;

    assign s_axis_tready = 1'b1;
    wire sample_valid = s_axis_tvalid & s_axis_tready;
    wire [7:0] sample_ch1 = s_axis_tdata[7:0];
    wire [7:0] sample_ch2 = s_axis_tdata[15:8];
    wire [7:0] sample_ch3 = s_axis_tdata[23:16];
    wire [7:0] sample_ch4 = s_axis_tdata[31:24];

    wire [9:0] y_ch1 = (V_ACTIVE - 1) - ((sample_ch1 * (V_ACTIVE - 1)) >> 8);
    wire [9:0] y_ch2 = (V_ACTIVE - 1) - ((sample_ch2 * (V_ACTIVE - 1)) >> 8);
    wire [9:0] y_ch3 = (V_ACTIVE - 1) - ((sample_ch3 * (V_ACTIVE - 1)) >> 8);
    wire [9:0] y_ch4 = (V_ACTIVE - 1) - ((sample_ch4 * (V_ACTIVE - 1)) >> 8);

    reg [9:0] prev_raw_y_ch1, prev_raw_y_ch2, prev_raw_y_ch3, prev_raw_y_ch4;
    reg [9:0] prev_smooth_y_ch1, prev_smooth_y_ch2, prev_smooth_y_ch3, prev_smooth_y_ch4;
    reg       prev_valid;

    wire [11:0] smooth_acc_ch1 = {2'b00, y_ch1} + {2'b00, y_ch1} + {2'b00, y_ch1} + {2'b00, prev_raw_y_ch1};
    wire [11:0] smooth_acc_ch2 = {2'b00, y_ch2} + {2'b00, y_ch2} + {2'b00, y_ch2} + {2'b00, prev_raw_y_ch2};
    wire [11:0] smooth_acc_ch3 = {2'b00, y_ch3} + {2'b00, y_ch3} + {2'b00, y_ch3} + {2'b00, prev_raw_y_ch3};
    wire [11:0] smooth_acc_ch4 = {2'b00, y_ch4} + {2'b00, y_ch4} + {2'b00, y_ch4} + {2'b00, prev_raw_y_ch4};

    wire [9:0] y_ch1_smooth = prev_valid ? ((smooth_acc_ch1 + 12'd2) >> 2) : y_ch1;
    wire [9:0] y_ch2_smooth = prev_valid ? ((smooth_acc_ch2 + 12'd2) >> 2) : y_ch2;
    wire [9:0] y_ch3_smooth = prev_valid ? ((smooth_acc_ch3 + 12'd2) >> 2) : y_ch3;
    wire [9:0] y_ch4_smooth = prev_valid ? ((smooth_acc_ch4 + 12'd2) >> 2) : y_ch4;

    reg ch1_hit, ch2_hit, ch3_hit, ch4_hit;
    reg grid_hit, center_hit;
    reg dbg_hit, dbg_on;
    reg trig_type_hit;
    reg [15:0] trig_type_color;
    reg dbg_stripe_hit;
    reg [15:0] dbg_stripe_color;

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

    function near_1px;
        input [9:0] y;
        input [9:0] y0;
        begin
            if (y >= y0)
                near_1px = ((y - y0) <= 10'd1);
            else
                near_1px = ((y0 - y) <= 10'd1);
        end
    endfunction

    function in_segment_1px;
        input [9:0] y;
        input [9:0] y0;
        input [9:0] y1;
        begin
            in_segment_1px = in_segment(y, y0, y1);

            if (!in_segment_1px && (y != 10'd0))
                in_segment_1px = in_segment(y - 10'd1, y0, y1);

            if (!in_segment_1px && (y < (V_ACTIVE - 1)))
                in_segment_1px = in_segment(y + 10'd1, y0, y1);
        end
    endfunction

    function [9:0] abs_diff_10;
        input [9:0] a;
        input [9:0] b;
        begin
            if (a >= b)
                abs_diff_10 = a - b;
            else
                abs_diff_10 = b - a;
        end
    endfunction

    wire ch1_connect_ok = (abs_diff_10(prev_smooth_y_ch1, y_ch1_smooth) <= MAX_SEG_DY);
    wire ch2_connect_ok = (abs_diff_10(prev_smooth_y_ch2, y_ch2_smooth) <= MAX_SEG_DY);
    wire ch3_connect_ok = (abs_diff_10(prev_smooth_y_ch3, y_ch3_smooth) <= MAX_SEG_DY);
    wire ch4_connect_ok = (abs_diff_10(prev_smooth_y_ch4, y_ch4_smooth) <= MAX_SEG_DY);

    always @(posedge clk_pix) begin
        if (!de) begin
            prev_valid <= 1'b0;
        end else if (sample_valid) begin
            if ((pix_x == 11'd0) || s_axis_tuser || s_axis_tlast)
                prev_valid <= 1'b0;
            else
                prev_valid <= 1'b1;

            prev_raw_y_ch1 <= y_ch1;
            prev_raw_y_ch2 <= y_ch2;
            prev_raw_y_ch3 <= y_ch3;
            prev_raw_y_ch4 <= y_ch4;

            prev_smooth_y_ch1 <= y_ch1_smooth;
            prev_smooth_y_ch2 <= y_ch2_smooth;
            prev_smooth_y_ch3 <= y_ch3_smooth;
            prev_smooth_y_ch4 <= y_ch4_smooth;
        end
    end

    always @(*) begin
        grid_hit   = ((pix_x % 80) == 0) || ((pix_y % 60) == 0);
        center_hit = (pix_x == (H_ACTIVE >> 1)) || (pix_y == (V_ACTIVE >> 1));
        dbg_hit    = 1'b0;
        dbg_on     = 1'b0;
        trig_type_hit = 1'b0;
        trig_type_color = COLOR_GRID;
        dbg_stripe_hit   = 1'b0;
        dbg_stripe_color = COLOR_BLACK;

        // 1px thickness (+/-1) with weighted smoothing and x-1 to x interpolation.
        ch1_hit = sample_valid &&
                  (near_1px(pix_y, y_ch1_smooth) ||
                   (prev_valid && ch1_connect_ok &&
                    in_segment_1px(pix_y, prev_smooth_y_ch1, y_ch1_smooth)));
        ch2_hit = sample_valid &&
                  (near_1px(pix_y, y_ch2_smooth) ||
                   (prev_valid && ch2_connect_ok &&
                    in_segment_1px(pix_y, prev_smooth_y_ch2, y_ch2_smooth)));
        ch3_hit = sample_valid &&
                  (near_1px(pix_y, y_ch3_smooth) ||
                   (prev_valid && ch3_connect_ok &&
                    in_segment_1px(pix_y, prev_smooth_y_ch3, y_ch3_smooth)));
        ch4_hit = sample_valid &&
                  (near_1px(pix_y, y_ch4_smooth) ||
                   (prev_valid && ch4_connect_ok &&
                    in_segment_1px(pix_y, prev_smooth_y_ch4, y_ch4_smooth)));

        // Top-left 64 debug boxes (8x8): rowN maps bit[N*8 + 0 .. N*8 + 7].
        if ((pix_y >= 10) && (pix_y < 22)) begin
            if      ((pix_x >= 10)  && (pix_x < 22))  begin dbg_hit = 1'b1; dbg_on = debug_status[0]; end
            else if ((pix_x >= 24)  && (pix_x < 36))  begin dbg_hit = 1'b1; dbg_on = debug_status[1]; end
            else if ((pix_x >= 38)  && (pix_x < 50))  begin dbg_hit = 1'b1; dbg_on = debug_status[2]; end
            else if ((pix_x >= 52)  && (pix_x < 64))  begin dbg_hit = 1'b1; dbg_on = debug_status[3]; end
            else if ((pix_x >= 66)  && (pix_x < 78))  begin dbg_hit = 1'b1; dbg_on = debug_status[4]; end
            else if ((pix_x >= 80)  && (pix_x < 92))  begin dbg_hit = 1'b1; dbg_on = debug_status[5]; end
            else if ((pix_x >= 94)  && (pix_x < 106)) begin dbg_hit = 1'b1; dbg_on = debug_status[6]; end
            else if ((pix_x >= 108) && (pix_x < 120)) begin dbg_hit = 1'b1; dbg_on = debug_status[7]; end
        end else if ((pix_y >= 24) && (pix_y < 36)) begin
            if      ((pix_x >= 10)  && (pix_x < 22))  begin dbg_hit = 1'b1; dbg_on = debug_status[8]; end
            else if ((pix_x >= 24)  && (pix_x < 36))  begin dbg_hit = 1'b1; dbg_on = debug_status[9]; end
            else if ((pix_x >= 38)  && (pix_x < 50))  begin dbg_hit = 1'b1; dbg_on = debug_status[10]; end
            else if ((pix_x >= 52)  && (pix_x < 64))  begin dbg_hit = 1'b1; dbg_on = debug_status[11]; end
            else if ((pix_x >= 66)  && (pix_x < 78))  begin dbg_hit = 1'b1; dbg_on = debug_status[12]; end
            else if ((pix_x >= 80)  && (pix_x < 92))  begin dbg_hit = 1'b1; dbg_on = debug_status[13]; end
            else if ((pix_x >= 94)  && (pix_x < 106)) begin dbg_hit = 1'b1; dbg_on = debug_status[14]; end
            else if ((pix_x >= 108) && (pix_x < 120)) begin dbg_hit = 1'b1; dbg_on = debug_status[15]; end
        end else if ((pix_y >= 38) && (pix_y < 50)) begin
            if      ((pix_x >= 10)  && (pix_x < 22))  begin dbg_hit = 1'b1; dbg_on = debug_status[16]; end
            else if ((pix_x >= 24)  && (pix_x < 36))  begin dbg_hit = 1'b1; dbg_on = debug_status[17]; end
            else if ((pix_x >= 38)  && (pix_x < 50))  begin dbg_hit = 1'b1; dbg_on = debug_status[18]; end
            else if ((pix_x >= 52)  && (pix_x < 64))  begin dbg_hit = 1'b1; dbg_on = debug_status[19]; end
            else if ((pix_x >= 66)  && (pix_x < 78))  begin dbg_hit = 1'b1; dbg_on = debug_status[20]; end
            else if ((pix_x >= 80)  && (pix_x < 92))  begin dbg_hit = 1'b1; dbg_on = debug_status[21]; end
            else if ((pix_x >= 94)  && (pix_x < 106)) begin dbg_hit = 1'b1; dbg_on = debug_status[22]; end
            else if ((pix_x >= 108) && (pix_x < 120)) begin dbg_hit = 1'b1; dbg_on = debug_status[23]; end
        end else if ((pix_y >= 52) && (pix_y < 64)) begin
            if      ((pix_x >= 10)  && (pix_x < 22))  begin dbg_hit = 1'b1; dbg_on = debug_status[24]; end
            else if ((pix_x >= 24)  && (pix_x < 36))  begin dbg_hit = 1'b1; dbg_on = debug_status[25]; end
            else if ((pix_x >= 38)  && (pix_x < 50))  begin dbg_hit = 1'b1; dbg_on = debug_status[26]; end
            else if ((pix_x >= 52)  && (pix_x < 64))  begin dbg_hit = 1'b1; dbg_on = debug_status[27]; end
            else if ((pix_x >= 66)  && (pix_x < 78))  begin dbg_hit = 1'b1; dbg_on = debug_status[28]; end
            else if ((pix_x >= 80)  && (pix_x < 92))  begin dbg_hit = 1'b1; dbg_on = debug_status[29]; end
            else if ((pix_x >= 94)  && (pix_x < 106)) begin dbg_hit = 1'b1; dbg_on = debug_status[30]; end
            else if ((pix_x >= 108) && (pix_x < 120)) begin dbg_hit = 1'b1; dbg_on = debug_status[31]; end
        end else if ((pix_y >= 66) && (pix_y < 78)) begin
            if      ((pix_x >= 10)  && (pix_x < 22))  begin dbg_hit = 1'b1; dbg_on = debug_status[32]; end
            else if ((pix_x >= 24)  && (pix_x < 36))  begin dbg_hit = 1'b1; dbg_on = debug_status[33]; end
            else if ((pix_x >= 38)  && (pix_x < 50))  begin dbg_hit = 1'b1; dbg_on = debug_status[34]; end
            else if ((pix_x >= 52)  && (pix_x < 64))  begin dbg_hit = 1'b1; dbg_on = debug_status[35]; end
            else if ((pix_x >= 66)  && (pix_x < 78))  begin dbg_hit = 1'b1; dbg_on = debug_status[36]; end
            else if ((pix_x >= 80)  && (pix_x < 92))  begin dbg_hit = 1'b1; dbg_on = debug_status[37]; end
            else if ((pix_x >= 94)  && (pix_x < 106)) begin dbg_hit = 1'b1; dbg_on = debug_status[38]; end
            else if ((pix_x >= 108) && (pix_x < 120)) begin dbg_hit = 1'b1; dbg_on = debug_status[39]; end
        end else if ((pix_y >= 80) && (pix_y < 92)) begin
            if      ((pix_x >= 10)  && (pix_x < 22))  begin dbg_hit = 1'b1; dbg_on = debug_status[40]; end
            else if ((pix_x >= 24)  && (pix_x < 36))  begin dbg_hit = 1'b1; dbg_on = debug_status[41]; end
            else if ((pix_x >= 38)  && (pix_x < 50))  begin dbg_hit = 1'b1; dbg_on = debug_status[42]; end
            else if ((pix_x >= 52)  && (pix_x < 64))  begin dbg_hit = 1'b1; dbg_on = debug_status[43]; end
            else if ((pix_x >= 66)  && (pix_x < 78))  begin dbg_hit = 1'b1; dbg_on = debug_status[44]; end
            else if ((pix_x >= 80)  && (pix_x < 92))  begin dbg_hit = 1'b1; dbg_on = debug_status[45]; end
            else if ((pix_x >= 94)  && (pix_x < 106)) begin dbg_hit = 1'b1; dbg_on = debug_status[46]; end
            else if ((pix_x >= 108) && (pix_x < 120)) begin dbg_hit = 1'b1; dbg_on = debug_status[47]; end
        end else if ((pix_y >= 94) && (pix_y < 106)) begin
            if      ((pix_x >= 10)  && (pix_x < 22))  begin dbg_hit = 1'b1; dbg_on = debug_status[48]; end
            else if ((pix_x >= 24)  && (pix_x < 36))  begin dbg_hit = 1'b1; dbg_on = debug_status[49]; end
            else if ((pix_x >= 38)  && (pix_x < 50))  begin dbg_hit = 1'b1; dbg_on = debug_status[50]; end
            else if ((pix_x >= 52)  && (pix_x < 64))  begin dbg_hit = 1'b1; dbg_on = debug_status[51]; end
            else if ((pix_x >= 66)  && (pix_x < 78))  begin dbg_hit = 1'b1; dbg_on = debug_status[52]; end
            else if ((pix_x >= 80)  && (pix_x < 92))  begin dbg_hit = 1'b1; dbg_on = debug_status[53]; end
            else if ((pix_x >= 94)  && (pix_x < 106)) begin dbg_hit = 1'b1; dbg_on = debug_status[54]; end
            else if ((pix_x >= 108) && (pix_x < 120)) begin dbg_hit = 1'b1; dbg_on = debug_status[55]; end
        end else if ((pix_y >= 108) && (pix_y < 120)) begin
            if      ((pix_x >= 10)  && (pix_x < 22))  begin dbg_hit = 1'b1; dbg_on = debug_status[56]; end
            else if ((pix_x >= 24)  && (pix_x < 36))  begin dbg_hit = 1'b1; dbg_on = debug_status[57]; end
            else if ((pix_x >= 38)  && (pix_x < 50))  begin dbg_hit = 1'b1; dbg_on = debug_status[58]; end
            else if ((pix_x >= 52)  && (pix_x < 64))  begin dbg_hit = 1'b1; dbg_on = debug_status[59]; end
            else if ((pix_x >= 66)  && (pix_x < 78))  begin dbg_hit = 1'b1; dbg_on = debug_status[60]; end
            else if ((pix_x >= 80)  && (pix_x < 92))  begin dbg_hit = 1'b1; dbg_on = debug_status[61]; end
            else if ((pix_x >= 94)  && (pix_x < 106)) begin dbg_hit = 1'b1; dbg_on = debug_status[62]; end
            else if ((pix_x >= 108) && (pix_x < 120)) begin dbg_hit = 1'b1; dbg_on = debug_status[63]; end
        end

        // Trigger type blocks below matrix:
        // N block: last trigger came from normal edge (bit64)
        // T block: last trigger came from auto-timeout (bit65)
        if ((pix_x >= 10) && (pix_x < 22) && (pix_y >= 124) && (pix_y < 136)) begin
            trig_type_hit = 1'b1;
            trig_type_color = debug_status[64] ? 16'h07E0 : 16'h2104;
        end else if ((pix_x >= 24) && (pix_x < 36) && (pix_y >= 124) && (pix_y < 136)) begin
            trig_type_hit = 1'b1;
            trig_type_color = debug_status[65] ? 16'hF800 : 16'h2104;
        end

        // Stripe debug panel (x:130..250):
        // row0 active_bank_rd(bit32), row1..4 raddr[0..3](bit40..43), row5 raddr_changed_rt(bit35).
        if ((pix_x >= 130) && (pix_x < 250)) begin
            if ((pix_y >= 10) && (pix_y < 18)) begin
                dbg_stripe_hit = 1'b1;
                dbg_stripe_color = debug_status[32] ? 16'h07FF : 16'h2104;
            end else if ((pix_y >= 20) && (pix_y < 28)) begin
                dbg_stripe_hit = 1'b1;
                dbg_stripe_color = debug_status[40] ? 16'hFFE0 : 16'h2104;
            end else if ((pix_y >= 30) && (pix_y < 38)) begin
                dbg_stripe_hit = 1'b1;
                dbg_stripe_color = debug_status[41] ? 16'hFFE0 : 16'h2104;
            end else if ((pix_y >= 40) && (pix_y < 48)) begin
                dbg_stripe_hit = 1'b1;
                dbg_stripe_color = debug_status[42] ? 16'hFFE0 : 16'h2104;
            end else if ((pix_y >= 50) && (pix_y < 58)) begin
                dbg_stripe_hit = 1'b1;
                dbg_stripe_color = debug_status[43] ? 16'hFFE0 : 16'h2104;
            end else if ((pix_y >= 60) && (pix_y < 68)) begin
                dbg_stripe_hit = 1'b1;
                dbg_stripe_color = debug_status[35] ? 16'hF800 : 16'h2104;
            end
        end

        if (!de) begin
            rgb565 = COLOR_BLACK;
        end else if (dbg_hit) begin
            rgb565 = dbg_on ? COLOR_DBG_ON : COLOR_DBG_OFF;
        end else if (trig_type_hit) begin
            rgb565 = trig_type_color;
        end else if (dbg_stripe_hit) begin
            rgb565 = dbg_stripe_color;
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
