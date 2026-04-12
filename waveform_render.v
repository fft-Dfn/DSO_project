// -----------------------------------------------------------------------------
// waveform_renderer
// -----------------------------------------------------------------------------
// Purpose:
//   - Left panel (128x480): simple UI buttons + real-time sampled parameters.
//   - Right panel (512x480): oscilloscope waveform using min/max vertical bars.
//   - Debug matrix kept (size unchanged) and moved to top-right.
// -----------------------------------------------------------------------------
module waveform_renderer #(
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    input  wire         clk_pix,
    input  wire         de,
    input  wire [10:0]  pix_x,
    input  wire [9:0]   pix_y,

    // AXI4-Stream-like waveform column input (one beat per wave-area pixel).
    input  wire         s_axis_tvalid,
    output wire         s_axis_tready,
    input  wire [63:0]  s_axis_tdata, // {ch4_max,ch4_min,...,ch1_max,ch1_min}
    input  wire         s_axis_tlast,
    input  wire         s_axis_tuser,

    input  wire [65:0]  debug_status,

    // UI/control values
    input  wire [3:0]   ui_page,
    input  wire [3:0]   ui_cursor,
    input  wire         ui_curr_edit_mode,
    input  wire [3:0]   ui_curr_edit_value,
    input  wire [2:0]   ui_active_src_sel,
    input  wire [2:0]   view_ch_sel,
    input  wire         trig_mode,
    input  wire         trig_edge,
    input  wire [7:0]   trig_level,
    input  wire [31:0]  sample_div,
    input  wire [2:0]   sel_trig,

    output reg  [15:0]  rgb565
);

    localparam UI_W   = 11'd128;
    localparam WAVE_W = 11'd512;
    localparam PAGE_MAIN    = 4'd0;
    localparam PAGE_SRC     = 4'd1;
    localparam PAGE_SRC_CFG = 4'd2;
    localparam PAGE_TRIG    = 4'd3;
    localparam PAGE_DISP    = 4'd4;

    localparam [15:0] COLOR_BLACK      = 16'h0000;
    localparam [15:0] COLOR_WAVE_BG    = 16'h0000;
    localparam [15:0] COLOR_GRID       = 16'h2104;
    localparam [15:0] COLOR_CENTER     = 16'h39E7;
    localparam [15:0] COLOR_CH1        = 16'hF800;
    localparam [15:0] COLOR_CH2        = 16'h07E0;
    localparam [15:0] COLOR_CH3        = 16'h001F;
    localparam [15:0] COLOR_CH4        = 16'hFFE0;
    localparam [15:0] COLOR_UI_BG      = 16'h18C3;
    localparam [15:0] COLOR_UI_BTN     = 16'h2128;
    localparam [15:0] COLOR_UI_BTN_ACT = 16'h055F;
    localparam [15:0] COLOR_UI_BTN_CUR = 16'h07FF;
    localparam [15:0] COLOR_UI_TEXT    = 16'hAFE5;
    localparam [15:0] COLOR_VIEW_SEL   = 16'hFFFF;
    localparam [15:0] COLOR_DBG_ON     = 16'h07E0;
    localparam [15:0] COLOR_DBG_OFF    = 16'h8000;

    localparam DBG_X0     = 11'd520;
    localparam DBG_Y0     = 10'd10;
    localparam DBG_BOX    = 11'd12;
    localparam DBG_STRIDE = 11'd14;
    localparam DBG_W      = 11'd110; // 8*14-2
    localparam DBG_H      = 10'd110;

    assign s_axis_tready = 1'b1;

    // Decode one waveform column min/max pack.
    wire [7:0] ch1_min = s_axis_tdata[7:0];
    wire [7:0] ch1_max = s_axis_tdata[15:8];
    wire [7:0] ch2_min = s_axis_tdata[23:16];
    wire [7:0] ch2_max = s_axis_tdata[31:24];
    wire [7:0] ch3_min = s_axis_tdata[39:32];
    wire [7:0] ch3_max = s_axis_tdata[47:40];
    wire [7:0] ch4_min = s_axis_tdata[55:48];
    wire [7:0] ch4_max = s_axis_tdata[63:56];

    function [9:0] sample_to_y;
        input [7:0] s;
        begin
            sample_to_y = (V_ACTIVE - 1) - ((s * (V_ACTIVE - 1)) >> 8);
        end
    endfunction

    function in_span_1px;
        input [9:0] y;
        input [9:0] y_top;
        input [9:0] y_bot;
        reg [9:0] yt, yb;
        begin
            yt = y_top;
            yb = y_bot;
            if (yt > yb) begin
                yt = y_bot;
                yb = y_top;
            end
            in_span_1px = (y >= (yt - (yt != 0))) && (y <= (yb + (yb < (V_ACTIVE - 1))));
        end
    endfunction

    wire [9:0] ch1_top = sample_to_y(ch1_max);
    wire [9:0] ch1_bot = sample_to_y(ch1_min);
    wire [9:0] ch2_top = sample_to_y(ch2_max);
    wire [9:0] ch2_bot = sample_to_y(ch2_min);
    wire [9:0] ch3_top = sample_to_y(ch3_max);
    wire [9:0] ch3_bot = sample_to_y(ch3_min);
    wire [9:0] ch4_top = sample_to_y(ch4_max);
    wire [9:0] ch4_bot = sample_to_y(ch4_min);

    wire in_ui_area   = (pix_x < UI_W);
    wire in_wave_area = (pix_x >= UI_W) && (pix_x < (UI_W + WAVE_W));
    wire wave_valid   = s_axis_tvalid && s_axis_tready && in_wave_area;

    wire [10:0] wave_x = pix_x - UI_W;
    wire grid_v_hit = (wave_x[5:0] == 6'd0);
    wire grid_h_hit = (pix_y == 10'd0)   || (pix_y == 10'd60)  ||
                      (pix_y == 10'd120) || (pix_y == 10'd180) ||
                      (pix_y == 10'd240) || (pix_y == 10'd300) ||
                      (pix_y == 10'd360) || (pix_y == 10'd420);
    wire grid_hit   = in_wave_area && (grid_v_hit || grid_h_hit);
    wire center_hit = in_wave_area && ((pix_x == (UI_W + (WAVE_W >> 1))) || (pix_y == (V_ACTIVE >> 1)));

    wire ch1_hit = wave_valid && in_span_1px(pix_y, ch1_top, ch1_bot);
    wire ch2_hit = wave_valid && in_span_1px(pix_y, ch2_top, ch2_bot);
    wire ch3_hit = wave_valid && in_span_1px(pix_y, ch3_top, ch3_bot);
    wire ch4_hit = wave_valid && in_span_1px(pix_y, ch4_top, ch4_bot);

    // UI value-to-text mapping (centralized).
    wire [31:0] mode_text;
    wire [7:0]  edge_char;
    wire [23:0] level_text;
    wire [31:0] div_text;
    wire [7:0]  trig_src_char;
    wire [23:0] view_text;

    ui_value_map u_ui_value_map (
        .trig_mode      (trig_mode),
        .trig_edge      (trig_edge),
        .trig_level     (trig_level),
        .sample_div     (sample_div),
        .sel_trig       (sel_trig),
        .view_ch_sel    (view_ch_sel),
        .mode_text      (mode_text),
        .edge_char      (edge_char),
        .level_text     (level_text),
        .div_text       (div_text),
        .trig_src_char  (trig_src_char),
        .view_text      (view_text)
    );

    wire view_sel_ch1 = (view_ch_sel == 3'd0);
    wire view_sel_ch2 = (view_ch_sel == 3'd1);
    wire view_sel_ch3 = (view_ch_sel == 3'd2);
    wire view_sel_ch4 = (view_ch_sel == 3'd3);

    // Text rendering on left panel: 8x12 glyph in 8x16 cell.
    wire [3:0] cell_row = pix_y[3:0];
    wire [4:0] cell_line = pix_y[8:4]; // 0..29
    wire [3:0] cell_col = pix_x[6:3];  // 0..15 for UI area
    wire [2:0] cell_bit = pix_x[2:0];

    reg [7:0] ui_char_code;
    reg [7:0] font_char_code;
    reg [3:0] font_row_idx;
    wire [7:0] font_row_bits;
    reg ui_text_hit;

    function [7:0] pick32;
        input [31:0] s;
        input [1:0] idx;
        begin
            case (idx)
                2'd0: pick32 = s[31:24];
                2'd1: pick32 = s[23:16];
                2'd2: pick32 = s[15:8];
                default: pick32 = s[7:0];
            endcase
        end
    endfunction

    function [7:0] pick24;
        input [23:0] s;
        input [1:0] idx;
        begin
            case (idx)
                2'd0: pick24 = s[23:16];
                2'd1: pick24 = s[15:8];
                default: pick24 = s[7:0];
            endcase
        end
    endfunction

    function [7:0] pick40;
        input [39:0] s;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: pick40 = s[39:32];
                3'd1: pick40 = s[31:24];
                3'd2: pick40 = s[23:16];
                3'd3: pick40 = s[15:8];
                default: pick40 = s[7:0];
            endcase
        end
    endfunction

    function [39:0] str5;
        input [7:0] c0;
        input [7:0] c1;
        input [7:0] c2;
        input [7:0] c3;
        input [7:0] c4;
        begin
            str5 = {c0, c1, c2, c3, c4};
        end
    endfunction

    function [7:0] src_char3;
        input [2:0] v;
        begin
            if (v <= 3'd4)
                src_char3 = 8'h61 + v;
            else
                src_char3 = 8'h3f;
        end
    endfunction

    reg [39:0] btn_label0, btn_label1, btn_label2, btn_label3, btn_label4, btn_label5, btn_label6;
    reg [2:0]  btn_count;

    always @(*) begin
        btn_count = 3'd0;
        btn_label0 = str5(8'h20,8'h20,8'h20,8'h20,8'h20);
        btn_label1 = str5(8'h20,8'h20,8'h20,8'h20,8'h20);
        btn_label2 = str5(8'h20,8'h20,8'h20,8'h20,8'h20);
        btn_label3 = str5(8'h20,8'h20,8'h20,8'h20,8'h20);
        btn_label4 = str5(8'h20,8'h20,8'h20,8'h20,8'h20);
        btn_label5 = str5(8'h20,8'h20,8'h20,8'h20,8'h20);
        btn_label6 = str5(8'h20,8'h20,8'h20,8'h20,8'h20);

        if (ui_curr_edit_mode) begin
            case (ui_page)
                PAGE_SRC_CFG: begin
                    case (ui_cursor)
                        4'd0: begin // freq
                            btn_count  = 3'd4;
                            btn_label0 = str5(8'h31,8'h30,8'h6b,8'h20,8'h20);
                            btn_label1 = str5(8'h32,8'h30,8'h6b,8'h20,8'h20);
                            btn_label2 = str5(8'h33,8'h30,8'h6b,8'h20,8'h20);
                            btn_label3 = str5(8'h34,8'h30,8'h6b,8'h20,8'h20);
                        end
                        4'd1: begin // type
                            btn_count  = 3'd4;
                            btn_label0 = str5(8'h73,8'h69,8'h6e,8'h65,8'h20);
                            btn_label1 = str5(8'h73,8'h71,8'h72,8'h65,8'h20);
                            btn_label2 = str5(8'h74,8'h72,8'h69,8'h61,8'h20);
                            btn_label3 = str5(8'h73,8'h61,8'h77,8'h20,8'h20);
                        end
                        default: begin // phase
                            btn_count  = 3'd4;
                            btn_label0 = str5(8'h30,8'h20,8'h20,8'h20,8'h20);
                            btn_label1 = str5(8'h39,8'h30,8'h20,8'h20,8'h20);
                            btn_label2 = str5(8'h31,8'h38,8'h30,8'h20,8'h20);
                            btn_label3 = str5(8'h32,8'h37,8'h30,8'h20,8'h20);
                        end
                    endcase
                end
                PAGE_TRIG: begin
                    case (ui_cursor)
                        4'd0: begin
                            btn_count  = 3'd2;
                            btn_label0 = str5(8'h6e,8'h6f,8'h72,8'h6d,8'h20);
                            btn_label1 = str5(8'h61,8'h75,8'h74,8'h6f,8'h20);
                        end
                        4'd1: begin
                            btn_count  = 3'd2;
                            btn_label0 = str5(8'h72,8'h69,8'h73,8'h65,8'h20);
                            btn_label1 = str5(8'h66,8'h61,8'h6c,8'h6c,8'h20);
                        end
                        4'd2: begin
                            btn_count  = 3'd5;
                            btn_label0 = str5(8'h30,8'h30,8'h30,8'h20,8'h20);
                            btn_label1 = str5(8'h30,8'h36,8'h34,8'h20,8'h20);
                            btn_label2 = str5(8'h31,8'h32,8'h38,8'h20,8'h20);
                            btn_label3 = str5(8'h31,8'h39,8'h32,8'h20,8'h20);
                            btn_label4 = str5(8'h32,8'h35,8'h35,8'h20,8'h20);
                        end
                        default: begin
                            btn_count  = 3'd4;
                            btn_label0 = str5(8'h31,8'h20,8'h20,8'h20,8'h20);
                            btn_label1 = str5(8'h35,8'h20,8'h20,8'h20,8'h20);
                            btn_label2 = str5(8'h31,8'h32,8'h20,8'h20,8'h20);
                            btn_label3 = str5(8'h35,8'h30,8'h20,8'h20,8'h20);
                        end
                    endcase
                end
                PAGE_DISP: begin
                    if (ui_cursor <= 4'd4) begin
                        btn_count  = 3'd5;
                        btn_label0 = str5(8'h61,8'h20,8'h20,8'h20,8'h20);
                        btn_label1 = str5(8'h62,8'h20,8'h20,8'h20,8'h20);
                        btn_label2 = str5(8'h63,8'h20,8'h20,8'h20,8'h20);
                        btn_label3 = str5(8'h64,8'h20,8'h20,8'h20,8'h20);
                        btn_label4 = str5(8'h65,8'h20,8'h20,8'h20,8'h20);
                    end else if (ui_cursor == 4'd5) begin
                        btn_count  = 3'd5;
                        btn_label0 = str5(8'h63,8'h68,8'h31,8'h20,8'h20);
                        btn_label1 = str5(8'h63,8'h68,8'h32,8'h20,8'h20);
                        btn_label2 = str5(8'h63,8'h68,8'h33,8'h20,8'h20);
                        btn_label3 = str5(8'h63,8'h68,8'h34,8'h20,8'h20);
                        btn_label4 = str5(8'h66,8'h6c,8'h73,8'h20,8'h20);
                    end else begin
                        btn_count  = 3'd4;
                        btn_label0 = str5(8'h63,8'h68,8'h31,8'h20,8'h20);
                        btn_label1 = str5(8'h63,8'h68,8'h32,8'h20,8'h20);
                        btn_label2 = str5(8'h63,8'h68,8'h33,8'h20,8'h20);
                        btn_label3 = str5(8'h63,8'h68,8'h34,8'h20,8'h20);
                    end
                end
                default: begin
                    btn_count  = 3'd0;
                end
            endcase
        end else begin
            case (ui_page)
                PAGE_MAIN: begin
                    btn_count  = 3'd5;
                    btn_label0 = str5(8'h73,8'h72,8'h63,8'h20,8'h20);
                    btn_label1 = str5(8'h74,8'h72,8'h69,8'h67,8'h20);
                    btn_label2 = str5(8'h64,8'h69,8'h73,8'h70,8'h20);
                    btn_label3 = str5(8'h73,8'h74,8'h6f,8'h72,8'h65);
                    btn_label4 = str5(8'h6c,8'h6f,8'h61,8'h64,8'h20);
                end
                PAGE_SRC: begin
                    btn_count  = 3'd5;
                    btn_label0 = str5(8'h61,8'h20,8'h20,8'h20,8'h20);
                    btn_label1 = str5(8'h62,8'h20,8'h20,8'h20,8'h20);
                    btn_label2 = str5(8'h63,8'h20,8'h20,8'h20,8'h20);
                    btn_label3 = str5(8'h64,8'h20,8'h20,8'h20,8'h20);
                    btn_label4 = str5(8'h65,8'h20,8'h20,8'h20,8'h20);
                end
                PAGE_SRC_CFG: begin
                    btn_count  = 3'd3;
                    btn_label0 = str5(8'h66,8'h72,8'h65,8'h71,8'h20);
                    btn_label1 = str5(8'h74,8'h79,8'h70,8'h65,8'h20);
                    btn_label2 = str5(8'h70,8'h68,8'h61,8'h73,8'h65);
                end
                PAGE_TRIG: begin
                    btn_count  = 3'd4;
                    btn_label0 = str5(8'h6d,8'h6f,8'h64,8'h65,8'h20);
                    btn_label1 = str5(8'h65,8'h64,8'h67,8'h65,8'h20);
                    btn_label2 = str5(8'h6c,8'h65,8'h76,8'h65,8'h6c);
                    btn_label3 = str5(8'h64,8'h69,8'h76,8'h20,8'h20);
                end
                PAGE_DISP: begin
                    btn_count  = 3'd7;
                    btn_label0 = str5(8'h63,8'h68,8'h31,8'h69,8'h6e);
                    btn_label1 = str5(8'h63,8'h68,8'h32,8'h69,8'h6e);
                    btn_label2 = str5(8'h63,8'h68,8'h33,8'h69,8'h6e);
                    btn_label3 = str5(8'h63,8'h68,8'h34,8'h69,8'h6e);
                    btn_label4 = str5(8'h74,8'h72,8'h67,8'h69,8'h6e);
                    btn_label5 = str5(8'h76,8'h69,8'h65,8'h77,8'h20);
                    btn_label6 = str5(8'h73,8'h74,8'h6f,8'h72,8'h65);
                end
                default: begin
                    btn_count  = 3'd0;
                end
            endcase
        end
    end

    // Button activity/color.
    reg btn_hit;
    reg [2:0] btn_id;
    reg btn_active;
    reg btn_cursor;
    reg [15:0] btn_color;
    always @(*) begin
        btn_hit    = 1'b0;
        btn_id     = 3'd0;
        btn_active = 1'b0;
        btn_cursor = 1'b0;
        btn_color  = COLOR_UI_BTN;

        if ((pix_x >= 11'd8) && (pix_x < 11'd120)) begin
            if ((pix_y >= 10'd12) && (pix_y < 10'd32) && (btn_count > 0)) begin
                btn_hit = 1'b1; btn_id = 3'd0;
            end else if ((pix_y >= 10'd44) && (pix_y < 10'd64) && (btn_count > 1)) begin
                btn_hit = 1'b1; btn_id = 3'd1;
            end else if ((pix_y >= 10'd76) && (pix_y < 10'd96) && (btn_count > 2)) begin
                btn_hit = 1'b1; btn_id = 3'd2;
            end else if ((pix_y >= 10'd108) && (pix_y < 10'd128) && (btn_count > 3)) begin
                btn_hit = 1'b1; btn_id = 3'd3;
            end else if ((pix_y >= 10'd140) && (pix_y < 10'd160) && (btn_count > 4)) begin
                btn_hit = 1'b1; btn_id = 3'd4;
            end else if ((pix_y >= 10'd172) && (pix_y < 10'd192) && (btn_count > 5)) begin
                btn_hit = 1'b1; btn_id = 3'd5;
            end else if ((pix_y >= 10'd204) && (pix_y < 10'd224) && (btn_count > 6)) begin
                btn_hit = 1'b1; btn_id = 3'd6;
            end
        end

        if (btn_hit) begin
            if (ui_curr_edit_mode) begin
                btn_active = (btn_id == ui_curr_edit_value[2:0]);
                btn_cursor = btn_active;
            end else begin
                btn_cursor = (btn_id == ui_cursor[2:0]);
                if (ui_page == PAGE_SRC)
                    btn_active = (btn_id == ui_active_src_sel);
                else
                    btn_active = 1'b0;
            end

            btn_color = btn_active ? COLOR_UI_BTN_ACT : COLOR_UI_BTN;
            if (btn_cursor)
                btn_color = COLOR_UI_BTN_CUR;
        end
    end

    always @(*) begin
        ui_char_code = 8'h20; // default space

        case (cell_line)
            5'd1: begin
                if ((cell_col >= 4'd5) && (cell_col <= 4'd9))
                    ui_char_code = pick40(btn_label0, cell_col - 4'd5);
            end
            5'd3: begin
                if ((cell_col >= 4'd5) && (cell_col <= 4'd9))
                    ui_char_code = pick40(btn_label1, cell_col - 4'd5);
            end
            5'd5: begin
                if ((cell_col >= 4'd5) && (cell_col <= 4'd9))
                    ui_char_code = pick40(btn_label2, cell_col - 4'd5);
            end
            5'd7: begin
                if ((cell_col >= 4'd5) && (cell_col <= 4'd9))
                    ui_char_code = pick40(btn_label3, cell_col - 4'd5);
            end
            5'd9: begin
                if ((cell_col >= 4'd5) && (cell_col <= 4'd9))
                    ui_char_code = pick40(btn_label4, cell_col - 4'd5);
            end
            5'd11: begin
                if ((cell_col >= 4'd5) && (cell_col <= 4'd9))
                    ui_char_code = pick40(btn_label5, cell_col - 4'd5);
            end
            5'd13: begin
                if ((cell_col >= 4'd5) && (cell_col <= 4'd9))
                    ui_char_code = pick40(btn_label6, cell_col - 4'd5);
            end
            5'd16: begin // mode:auto/norm
                case (cell_col)
                    4'd0: ui_char_code = 8'h6d; // m
                    4'd1: ui_char_code = 8'h6f; // o
                    4'd2: ui_char_code = 8'h64; // d
                    4'd3: ui_char_code = 8'h65; // e
                    4'd4: ui_char_code = 8'h3a; // :
                    4'd5: ui_char_code = pick32(mode_text, 2'd0);
                    4'd6: ui_char_code = pick32(mode_text, 2'd1);
                    4'd7: ui_char_code = pick32(mode_text, 2'd2);
                    4'd8: ui_char_code = pick32(mode_text, 2'd3);
                endcase
            end
            5'd17: begin // edge:r/f
                case (cell_col)
                    4'd0: ui_char_code = 8'h65; // e
                    4'd1: ui_char_code = 8'h64; // d
                    4'd2: ui_char_code = 8'h67; // g
                    4'd3: ui_char_code = 8'h65; // e
                    4'd4: ui_char_code = 8'h3a; // :
                    4'd5: ui_char_code = edge_char;
                endcase
            end
            5'd18: begin // level:xxx
                case (cell_col)
                    4'd0: ui_char_code = 8'h6c;
                    4'd1: ui_char_code = 8'h65;
                    4'd2: ui_char_code = 8'h76;
                    4'd3: ui_char_code = 8'h65;
                    4'd4: ui_char_code = 8'h6c;
                    4'd5: ui_char_code = 8'h3a;
                    4'd6: ui_char_code = pick24(level_text, 2'd0);
                    4'd7: ui_char_code = pick24(level_text, 2'd1);
                    4'd8: ui_char_code = pick24(level_text, 2'd2);
                endcase
            end
            5'd19: begin // div:xxxx
                case (cell_col)
                    4'd0: ui_char_code = 8'h64;
                    4'd1: ui_char_code = 8'h69;
                    4'd2: ui_char_code = 8'h76;
                    4'd3: ui_char_code = 8'h3a;
                    4'd4: ui_char_code = pick32(div_text, 2'd0);
                    4'd5: ui_char_code = pick32(div_text, 2'd1);
                    4'd6: ui_char_code = pick32(div_text, 2'd2);
                    4'd7: ui_char_code = pick32(div_text, 2'd3);
                endcase
            end
            5'd20: begin // trig:x
                case (cell_col)
                    4'd0: ui_char_code = 8'h74;
                    4'd1: ui_char_code = 8'h72;
                    4'd2: ui_char_code = 8'h69;
                    4'd3: ui_char_code = 8'h67;
                    4'd4: ui_char_code = 8'h3a;
                    4'd5: ui_char_code = trig_src_char;
                endcase
            end
            5'd21: begin // view:chN/fls
                case (cell_col)
                    4'd0: ui_char_code = 8'h76;
                    4'd1: ui_char_code = 8'h69;
                    4'd2: ui_char_code = 8'h65;
                    4'd3: ui_char_code = 8'h77;
                    4'd4: ui_char_code = 8'h3a;
                    4'd5: ui_char_code = pick24(view_text, 2'd0);
                    4'd6: ui_char_code = pick24(view_text, 2'd1);
                    4'd7: ui_char_code = pick24(view_text, 2'd2);
                endcase
            end
            5'd22: begin // src:x
                case (cell_col)
                    4'd0: ui_char_code = 8'h73;
                    4'd1: ui_char_code = 8'h72;
                    4'd2: ui_char_code = 8'h63;
                    4'd3: ui_char_code = 8'h3a;
                    4'd4: ui_char_code = src_char3(ui_active_src_sel);
                endcase
            end
            default: ui_char_code = 8'h20;
        endcase
    end

    always @(*) begin
        font_char_code = ui_char_code;
        font_row_idx   = cell_row;
        ui_text_hit    = 1'b0;

        if (in_ui_area && (cell_row < 4'd12) && (ui_char_code != 8'h20)) begin
            if (font_row_bits[7 - cell_bit])
                ui_text_hit = 1'b1;
        end
    end

    font8x12_rom u_font8x12_rom (
        .char_code(font_char_code),
        .row_idx  (font_row_idx),
        .row_bits (font_row_bits)
    );

    // Debug matrix (size unchanged) moved to top-right.
    reg dbg_hit, dbg_on;
    reg [7:0] dbg_idx;
    reg [10:0] dbg_rel_x;
    reg [9:0]  dbg_rel_y;
    integer dbg_col_i;
    integer dbg_row_i;
    reg [10:0] dbg_cell_x0;
    reg [9:0]  dbg_cell_y0;
    reg [7:0]  dbg_idx_next;

    always @(*) begin
        dbg_hit = 1'b0;
        dbg_on  = 1'b0;
        dbg_idx = 8'd0;
        dbg_rel_x = 11'd0;
        dbg_rel_y = 10'd0;
        dbg_cell_x0 = 11'd0;
        dbg_cell_y0 = 10'd0;
        dbg_idx_next = 8'd0;

        if ((pix_x >= DBG_X0) && (pix_x < (DBG_X0 + DBG_W)) &&
            (pix_y >= DBG_Y0) && (pix_y < (DBG_Y0 + DBG_H))) begin
            dbg_rel_x = pix_x - DBG_X0;
            dbg_rel_y = pix_y - DBG_Y0;
            for (dbg_row_i = 0; dbg_row_i < 8; dbg_row_i = dbg_row_i + 1) begin
                for (dbg_col_i = 0; dbg_col_i < 8; dbg_col_i = dbg_col_i + 1) begin
                    dbg_cell_x0 = dbg_col_i * DBG_STRIDE;
                    dbg_cell_y0 = dbg_row_i * DBG_STRIDE;
                    if ((dbg_rel_x >= dbg_cell_x0) && (dbg_rel_x < (dbg_cell_x0 + DBG_BOX)) &&
                        (dbg_rel_y >= dbg_cell_y0) && (dbg_rel_y < (dbg_cell_y0 + DBG_BOX))) begin
                        dbg_idx_next = (dbg_row_i << 3) + dbg_col_i;
                        dbg_idx = dbg_idx_next;
                        dbg_hit = 1'b1;
                        dbg_on  = debug_status[dbg_idx_next];
                    end
                end
            end
        end
    end

    // Trigger type blocks (N/T) below debug matrix.
    reg trig_type_hit;
    reg [15:0] trig_type_color;
    always @(*) begin
        trig_type_hit = 1'b0;
        trig_type_color = COLOR_GRID;
        if ((pix_x >= DBG_X0) && (pix_x < (DBG_X0 + 12)) &&
            (pix_y >= 10'd124) && (pix_y < 10'd136)) begin
            trig_type_hit = 1'b1;
            trig_type_color = debug_status[64] ? 16'h07E0 : 16'h2104; // N
        end else if ((pix_x >= (DBG_X0 + 14)) && (pix_x < (DBG_X0 + 26)) &&
                     (pix_y >= 10'd124) && (pix_y < 10'd136)) begin
            trig_type_hit = 1'b1;
            trig_type_color = debug_status[65] ? 16'hF800 : 16'h2104; // T
        end
    end

    // Final color composition.
    always @(*) begin
        if (!de) begin
            rgb565 = COLOR_BLACK;
        end else begin
            // Base layers.
            if (in_ui_area)
                rgb565 = COLOR_UI_BG;
            else
                rgb565 = COLOR_WAVE_BG;

            if (in_wave_area && grid_hit)
                rgb565 = COLOR_GRID;
            if (in_wave_area && center_hit)
                rgb565 = COLOR_CENTER;

            // Wave channels.
            if (ch4_hit && !view_sel_ch4)
                rgb565 = COLOR_CH4;
            if (ch3_hit && !view_sel_ch3)
                rgb565 = COLOR_CH3;
            if (ch2_hit && !view_sel_ch2)
                rgb565 = COLOR_CH2;
            if (ch1_hit && !view_sel_ch1)
                rgb565 = COLOR_CH1;
            if (ch1_hit && view_sel_ch1)
                rgb565 = COLOR_VIEW_SEL;
            if (ch2_hit && view_sel_ch2)
                rgb565 = COLOR_VIEW_SEL;
            if (ch3_hit && view_sel_ch3)
                rgb565 = COLOR_VIEW_SEL;
            if (ch4_hit && view_sel_ch4)
                rgb565 = COLOR_VIEW_SEL;

            // UI buttons + text.
            if (in_ui_area && btn_hit)
                rgb565 = btn_color;
            if (in_ui_area && ui_text_hit)
                rgb565 = COLOR_UI_TEXT;

            // Debug overlays (top priority).
            if (trig_type_hit)
                rgb565 = trig_type_color;
            if (dbg_hit)
                rgb565 = dbg_on ? COLOR_DBG_ON : COLOR_DBG_OFF;
        end
    end

endmodule
