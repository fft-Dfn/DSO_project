// -----------------------------------------------------------------------------
// hmi_controller
// -----------------------------------------------------------------------------
// Purpose:
//   Human-machine interface state machine.
//   Converts key pulses into:
//   - DDS source configuration
//   - Trigger/sampling configuration
//   - Display source routing
//   - Flash save/load requests
//
// Key Variables (English):
//   - curr_page/curr_cursor: current UI navigation position.
//   - curr_edit_mode: 1 while editing current item value.
//   - active_src_sel: currently selected DDS source (A..E).
//   - freq_idx/type_idx/phase_idx: per-source DDS config indices.
//   - trig_*_idx, sample_div_idx: trigger and sampling config indices.
// -----------------------------------------------------------------------------
module hmi_controller (
    input  wire        clk_50m,
    input  wire        rst_n,

    input  wire        key_up_p,
    input  wire        key_down_p,
    input  wire        key_enter_p,
    input  wire        key_back_p,

    // DDS configuration outputs.
    output reg  [31:0] dds_freq_a,
    output reg  [31:0] dds_freq_b,
    output reg  [31:0] dds_freq_c,
    output reg  [31:0] dds_freq_d,
    output reg  [31:0] dds_freq_e,

    output reg  [1:0]  dds_type_a,
    output reg  [1:0]  dds_type_b,
    output reg  [1:0]  dds_type_c,
    output reg  [1:0]  dds_type_d,
    output reg  [1:0]  dds_type_e,

    output reg  [7:0]  dds_phase_a,
    output reg  [7:0]  dds_phase_b,
    output reg  [7:0]  dds_phase_c,
    output reg  [7:0]  dds_phase_d,
    output reg  [7:0]  dds_phase_e,

    // Trigger/sampling outputs.
    output reg         trig_mode,      // 0: normal, 1: auto
    output reg         trig_edge,      // 0: rise,   1: fall
    output reg  [7:0]  trig_level,     
    output reg  [31:0] sample_div,     // sampling divider: 1/5/12/50 -> 50M/10M/4.17M/1M

    output reg  [2:0]  sel_ch1,
    output reg  [2:0]  sel_ch2,
    output reg  [2:0]  sel_ch3,
    output reg  [2:0]  sel_ch4,
    output reg  [2:0]  sel_trig,

    // Flash transaction request interface.
    output reg  [1:0]  flash_ch_sel,   // 0~3 => ch1~ch4
    output reg         flash_write_req,
    output reg         flash_read_req,

    // UI state outputs for display layer.
   
    output reg  [3:0]  ui_page,
    output reg  [3:0]  ui_cursor,
    output reg         ui_curr_edit_mode,
    output reg  [3:0]  ui_curr_edit_value,
    output reg  [2:0]  ui_active_src_sel,
    output reg  [2:0]  view_ch_sel    // 0..4 => ch1..ch4 + flash
);

    // =========================================================
    // UI page definitions
    // =========================================================
    localparam PAGE_MAIN    = 4'd0;
    localparam PAGE_SRC     = 4'd1;
    localparam PAGE_SRC_CFG = 4'd2;
    localparam PAGE_TRIG    = 4'd3;
    localparam PAGE_DISP    = 4'd4;

    // Main page items (5 entries).
    localparam MAIN_SIG_SRC = 4'd0;
    localparam MAIN_TRIG    = 4'd1;
    localparam MAIN_DISP    = 4'd2;
    localparam MAIN_STORE   = 4'd3;
    localparam MAIN_DISP_STORE = 4'd4;

    // Source-config page items (3 entries).
    localparam SRC_CFG_FREQ  = 4'd0;
    localparam SRC_CFG_TYPE  = 4'd1;
    localparam SRC_CFG_PHASE = 4'd2;

    // Trigger page items (4 entries).
    localparam TRIG_MODE_ITEM  = 4'd0;
    localparam TRIG_EDGE_ITEM  = 4'd1;
    localparam TRIG_LEVEL_ITEM = 4'd2;
    localparam TRIG_SAMP_ITEM  = 4'd3;

    // Display page items (7 entries).
    localparam DISP_CH1_IN    = 4'd0;
    localparam DISP_CH2_IN    = 4'd1;
    localparam DISP_CH3_IN    = 4'd2;
    localparam DISP_CH4_IN    = 4'd3;
    localparam DISP_TRIG_IN   = 4'd4;
    localparam DISP_VIEW_SEL  = 4'd5;
    localparam DISP_STORE_SEL = 4'd6;

    // Internal UI state registers.

    reg [3:0] curr_page;
    reg [3:0] curr_cursor;
    reg       curr_edit_mode;
    reg [3:0] curr_edit_value;
    reg [2:0] active_src_sel;

    // Per-source configuration indices (A..E).
    reg [1:0] freq_idx [0:4];   // 0:10k 1:20k 2:30k 3:40k
    reg [1:0] type_idx [0:4];   // 0:sine 1:square 2:tri 3:saw
    reg [1:0] phase_idx[0:4];   // 0:0 1:90 2:180 3:270

    // Trigger/sampling configuration indices.
    reg [2:0] trig_mode_idx;    // 0 normal 1 auto
    reg [2:0] trig_edge_idx;    // 0 rise 1 fall
    reg [2:0] trig_level_idx;   // 0~4
    reg [2:0] sample_div_idx;   // 0~3

    integer i;

    // Power-up defaults: keep behavior deterministic even if external reset is not pressed.
    initial begin
        curr_page      = PAGE_MAIN;
        curr_cursor    = 4'd0;
        curr_edit_mode = 1'b0;
        curr_edit_value = 4'd0;
        active_src_sel = 3'd0;

        freq_idx[0]  = 2'd1; type_idx[0]  = 2'd0; phase_idx[0] = 2'd0;
        freq_idx[1]  = 2'd1; type_idx[1]  = 2'd1; phase_idx[1] = 2'd0;
        freq_idx[2]  = 2'd1; type_idx[2]  = 2'd2; phase_idx[2] = 2'd0;
        freq_idx[3]  = 2'd1; type_idx[3]  = 2'd3; phase_idx[3] = 2'd0;
        freq_idx[4]  = 2'd1; type_idx[4]  = 2'd1; phase_idx[4] = 2'd0;

        trig_mode_idx  = 3'd1;
        trig_edge_idx  = 3'd0;
        trig_level_idx = 3'd2;
        sample_div_idx = 3'd2; // ~4.17MHz default, one window is about 3 cycles at 20kHz

        sel_ch1 = 3'd0;
        sel_ch2 = 3'd1;
        sel_ch3 = 3'd2;
        sel_ch4 = 3'd3;
        sel_trig = 3'd4;
        view_ch_sel = 3'd0;

        flash_ch_sel = 2'd0;
        flash_write_req = 1'b0;
        flash_read_req = 1'b0;

        ui_page = PAGE_MAIN;
        ui_cursor = 4'd0;
        ui_curr_edit_mode = 1'b0;
        ui_curr_edit_value = 4'd0;
        ui_active_src_sel = 3'd0;
    end


    // Function: max cursor index for each page.
    function [3:0] page_max_cursor;
        input [3:0] page;
        begin
            case (page)
                PAGE_MAIN:    page_max_cursor = 4'd4;
                PAGE_SRC:     page_max_cursor = 4'd4;
                PAGE_SRC_CFG: page_max_cursor = 4'd2;
                PAGE_TRIG:    page_max_cursor = 4'd3;
                PAGE_DISP:    page_max_cursor = 4'd6;
                default:      page_max_cursor = 4'd0;
            endcase
        end
    endfunction


    // Function: max editable value for current page/cursor.

    function [3:0] edit_max_value;
        input [3:0] page;
        input [3:0] cursor;
        begin
            case (page)
                PAGE_SRC_CFG: begin
                    case (cursor)
                        SRC_CFG_FREQ:  edit_max_value = 4'd3;
                        SRC_CFG_TYPE:  edit_max_value = 4'd3;
                        SRC_CFG_PHASE: edit_max_value = 4'd3;
                        default:       edit_max_value = 4'd0;
                    endcase
                end
                PAGE_TRIG: begin
                    case (cursor)
                        TRIG_MODE_ITEM:  edit_max_value = 4'd1;
                        TRIG_EDGE_ITEM:  edit_max_value = 4'd1;
                        TRIG_LEVEL_ITEM: edit_max_value = 4'd4;
                        TRIG_SAMP_ITEM:  edit_max_value = 4'd3;
                        default:         edit_max_value = 4'd0;
                    endcase
                end
                PAGE_DISP: begin
                    case (cursor)
                        DISP_CH1_IN:    edit_max_value = 4'd4;
                        DISP_CH2_IN:    edit_max_value = 4'd4;
                        DISP_CH3_IN:    edit_max_value = 4'd4;
                        DISP_CH4_IN:    edit_max_value = 4'd4;
                        DISP_TRIG_IN:   edit_max_value = 4'd4;
                        DISP_VIEW_SEL:  edit_max_value = 4'd4; // ch1~ch4 + flash
                        DISP_STORE_SEL: edit_max_value = 4'd3; // ch1~ch4
                        default:        edit_max_value = 4'd0;
                    endcase
                end
                default: edit_max_value = 4'd0;
            endcase
        end
    endfunction

    // Function: read current committed value when entering edit mode.
    function [3:0] get_current_value;
        input [3:0] page;
        input [3:0] cursor;
        input [2:0] src_sel;
        begin
            get_current_value = 4'd0;
            case (page)
                PAGE_SRC_CFG: begin
                    case (cursor)
                        SRC_CFG_FREQ:  get_current_value = {2'b00, freq_idx[src_sel]};
                        SRC_CFG_TYPE:  get_current_value = {2'b00, type_idx[src_sel]};
                        SRC_CFG_PHASE: get_current_value = {2'b00, phase_idx[src_sel]};
                        default:       get_current_value = 4'd0;
                    endcase
                end

                PAGE_TRIG: begin
                    case (cursor)
                        TRIG_MODE_ITEM:  get_current_value = {1'b0, trig_mode_idx};
                        TRIG_EDGE_ITEM:  get_current_value = {1'b0, trig_edge_idx};
                        TRIG_LEVEL_ITEM: get_current_value = {1'b0, trig_level_idx};
                        TRIG_SAMP_ITEM:  get_current_value = {1'b0, sample_div_idx};
                        default:         get_current_value = 4'd0;
                    endcase
                end

                PAGE_DISP: begin
                    case (cursor)
                        DISP_CH1_IN:    get_current_value = {1'b0, sel_ch1};
                        DISP_CH2_IN:    get_current_value = {1'b0, sel_ch2};
                        DISP_CH3_IN:    get_current_value = {1'b0, sel_ch3};
                        DISP_CH4_IN:    get_current_value = {1'b0, sel_ch4};
                        DISP_TRIG_IN:   get_current_value = {1'b0, sel_trig};
                        DISP_VIEW_SEL:  get_current_value = {1'b0, view_ch_sel};
                        DISP_STORE_SEL: get_current_value = {2'b00, flash_ch_sel};
                        default:        get_current_value = 4'd0;
                    endcase
                end
            endcase
        end
    endfunction

    // =========================================================
    // Main UI FSM.
    // =========================================================
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            curr_page      <= PAGE_MAIN;
            curr_cursor    <= 4'd0;
            curr_edit_mode      <= 1'b0;
            curr_edit_value     <= 4'd0;
            active_src_sel <= 3'd0;

          
            freq_idx[0]  <= 2'd1; //20kHz 
            type_idx[0]  <= 2'd0; 
            phase_idx[0] <= 2'd0; 

            freq_idx[1]  <= 2'd1; 
            type_idx[1]  <= 2'd1;
            phase_idx[1] <= 2'd0;

            freq_idx[2]  <= 2'd1; 
            type_idx[2]  <= 2'd2; 
            phase_idx[2] <= 2'd0; 

            freq_idx[3]  <= 2'd1; 
            type_idx[3]  <= 2'd3; 
            phase_idx[3] <= 2'd0; 
            // Default trigger source is E.
            freq_idx[4]  <= 2'd1; //20kHz
            type_idx[4]  <= 2'd1; // square wave
            phase_idx[4] <= 2'd0; // 0

            trig_mode_idx  <= 3'd1;// auto trigger mode
            trig_edge_idx  <= 3'd0;// rising-edge trigger
            trig_level_idx <= 3'd2;// 128 mid-level trigger
            sample_div_idx <= 3'd2;// ~4.17MHz, ~3 cycles/window at 20kHz

            sel_ch1        <= 3'd0; // A
            sel_ch2        <= 3'd1; // B
            sel_ch3        <= 3'd2; // C
            sel_ch4        <= 3'd3; // D
            sel_trig       <= 3'd4; // E
            view_ch_sel    <= 3'd0; // ch1

            flash_ch_sel   <= 2'd0; // ch1
            flash_write_req <= 1'b0;
            flash_read_req  <= 1'b0;
        end else begin
            // Pulse-type request outputs default low.
            flash_write_req <= 1'b0;
            flash_read_req  <= 1'b0;

            if (!curr_edit_mode) begin
                // -------------------------
                // Navigation mode.
                // -------------------------
                if (key_up_p) begin
                    if (curr_cursor > 0)
                        curr_cursor <= curr_cursor - 1'b1;
                end else if (key_down_p) begin
                    if (curr_cursor < page_max_cursor(curr_page))
                        curr_cursor <= curr_cursor + 1'b1;
                end else if (key_enter_p) begin
                    case (curr_page)
                        PAGE_MAIN: begin
                            case (curr_cursor)
                                MAIN_SIG_SRC: begin
                                    curr_page   <= PAGE_SRC;
                                    curr_cursor <= 4'd0;
                                end
                                MAIN_TRIG: begin
                                    curr_page   <= PAGE_TRIG;
                                    curr_cursor <= 4'd0;
                                end
                                MAIN_DISP: begin
                                    curr_page   <= PAGE_DISP;
                                    curr_cursor <= 4'd0;
                                end
                                MAIN_STORE: begin
                                    flash_write_req <= 1'b1;
                                end
                                MAIN_DISP_STORE:begin
                                    flash_read_req <=1'b1;
                                end
                            endcase
                        end

                        PAGE_SRC: begin
                            active_src_sel <= curr_cursor[2:0];
                            curr_page      <= PAGE_SRC_CFG;
                            curr_cursor    <= 4'd0;
                        end

                        PAGE_SRC_CFG,
                        PAGE_TRIG,
                        PAGE_DISP: begin
                            curr_edit_value <= get_current_value(curr_page, curr_cursor, active_src_sel);
                            curr_edit_mode  <= 1'b1;
                        end

                        default: begin
                            curr_page   <= PAGE_MAIN;
                            curr_cursor <= 4'd0;
                        end
                    endcase
                end else if (key_back_p) begin
                    case (curr_page)
                        PAGE_MAIN: begin
                            curr_page   <= PAGE_MAIN;
                            curr_cursor <= 4'd0;
                        end
                        PAGE_SRC: begin
                            curr_page   <= PAGE_MAIN;
                            curr_cursor <= MAIN_SIG_SRC;
                        end
                        PAGE_SRC_CFG: begin
                            curr_page   <= PAGE_SRC;
                            curr_cursor <= {1'b0, active_src_sel};
                        end
                        PAGE_TRIG: begin
                            curr_page   <= PAGE_MAIN;
                            curr_cursor <= MAIN_TRIG;
                        end
                        PAGE_DISP: begin
                            curr_page   <= PAGE_MAIN;
                            curr_cursor <= MAIN_DISP;
                        end
                        default: begin
                            curr_page   <= PAGE_MAIN;
                            curr_cursor <= 4'd0;
                        end
                    endcase
                end
            end else begin
                // Edit mode.
                if (key_up_p) begin
                    if (curr_edit_value < edit_max_value(curr_page, curr_cursor))
                        curr_edit_value <= curr_edit_value + 1'b1;
                end else if (key_down_p) begin
                    if (curr_edit_value > 0)
                        curr_edit_value <= curr_edit_value - 1'b1;
                end else if (key_enter_p) begin
                    // Commit edits.
                    case (curr_page)
                        PAGE_SRC_CFG: begin
                            case (curr_cursor)
                                SRC_CFG_FREQ:  freq_idx[active_src_sel]  <= curr_edit_value[1:0];
                                SRC_CFG_TYPE:  type_idx[active_src_sel]  <= curr_edit_value[1:0];
                                SRC_CFG_PHASE: phase_idx[active_src_sel] <= curr_edit_value[1:0];
                            endcase
                        end

                        PAGE_TRIG: begin
                            case (curr_cursor)
                                TRIG_MODE_ITEM:  trig_mode_idx  <= curr_edit_value[2:0];
                                TRIG_EDGE_ITEM:  trig_edge_idx  <= curr_edit_value[2:0];
                                TRIG_LEVEL_ITEM: trig_level_idx <= curr_edit_value[2:0];
                                TRIG_SAMP_ITEM:  sample_div_idx <= curr_edit_value[2:0];
                            endcase
                        end

                        PAGE_DISP: begin
                            case (curr_cursor)
                                DISP_CH1_IN:    sel_ch1      <= curr_edit_value[2:0];
                                DISP_CH2_IN:    sel_ch2      <= curr_edit_value[2:0];
                                DISP_CH3_IN:    sel_ch3      <= curr_edit_value[2:0];
                                DISP_CH4_IN:    sel_ch4      <= curr_edit_value[2:0];
                                DISP_TRIG_IN:   sel_trig     <= curr_edit_value[2:0];
                                DISP_VIEW_SEL:  view_ch_sel  <= curr_edit_value[2:0];
                                DISP_STORE_SEL: flash_ch_sel <= curr_edit_value[1:0];
                            endcase
                        end
                    endcase

                    curr_edit_mode <= 1'b0;
                end else if (key_back_p) begin
                    // Cancel edit.
                    curr_edit_mode <= 1'b0;
                end
            end
        end
    end

  
    // Parameter mapping: config index -> actual output value.
    // f_out = freq_word / (2^32) * f_clk
    always @(*) begin
        case (freq_idx[0])
            2'd0: dds_freq_a = 32'd858993;//10kHz
            2'd1: dds_freq_a = 32'd1717987;//20kHz
            2'd2: dds_freq_a = 32'd2576980;//30kHz
            default: dds_freq_a = 32'd3435973;//40kHz
        endcase
        case (freq_idx[1])
            2'd0: dds_freq_b = 32'd858993;//10kHz
            2'd1: dds_freq_b = 32'd1717987;//20kHz
            2'd2: dds_freq_b = 32'd2576980;//30kHz
            default: dds_freq_b = 32'd3435973;//40kHz
        endcase
        case (freq_idx[2])
            2'd0: dds_freq_c = 32'd858993;//10kHz
            2'd1: dds_freq_c = 32'd1717987;//20kHz
            2'd2: dds_freq_c = 32'd2576980;//30kHz
            default: dds_freq_c = 32'd3435973;//40kHz
        endcase
        case (freq_idx[3])
            2'd0: dds_freq_d = 32'd858993;//10kHz
            2'd1: dds_freq_d = 32'd1717987;//20kHz
            2'd2: dds_freq_d = 32'd2576980;//30kHz
            default: dds_freq_d = 32'd3435973;//40kHz
        endcase
        case (freq_idx[4])
            2'd0: dds_freq_e = 32'd858993;//10kHz
            2'd1: dds_freq_e = 32'd1717987;//20kHz
            2'd2: dds_freq_e = 32'd2576980;//30kHz
            default: dds_freq_e = 32'd3435973;//40kHz
        endcase

        // Wave type index is directly forwarded.
        dds_type_a = type_idx[0];
        dds_type_b = type_idx[1];
        dds_type_c = type_idx[2];
        dds_type_d = type_idx[3];
        dds_type_e = type_idx[4];

        // Phase index mapping.
        case (phase_idx[0])
            2'd0: dds_phase_a = 8'd0;
            2'd1: dds_phase_a = 8'd64;
            2'd2: dds_phase_a = 8'd128;
            default: dds_phase_a = 8'd192;
        endcase
        case (phase_idx[1])
            2'd0: dds_phase_b = 8'd0;
            2'd1: dds_phase_b = 8'd64;
            2'd2: dds_phase_b = 8'd128;
            default: dds_phase_b = 8'd192;
        endcase
        case (phase_idx[2])
            2'd0: dds_phase_c = 8'd0;
            2'd1: dds_phase_c = 8'd64;
            2'd2: dds_phase_c = 8'd128;
            default: dds_phase_c = 8'd192;
        endcase
        case (phase_idx[3])
            2'd0: dds_phase_d = 8'd0;
            2'd1: dds_phase_d = 8'd64;
            2'd2: dds_phase_d = 8'd128;
            default: dds_phase_d = 8'd192;
        endcase
        case (phase_idx[4])
            2'd0: dds_phase_e = 8'd0;
            2'd1: dds_phase_e = 8'd64;
            2'd2: dds_phase_e = 8'd128;
            default: dds_phase_e = 8'd192;
        endcase

        // Trigger parameter mapping.
        trig_mode = trig_mode_idx[0];
        trig_edge = trig_edge_idx[0];

        case (trig_level_idx)
            3'd0: trig_level = 8'd25;
            3'd1: trig_level = 8'd75;
            3'd2: trig_level = 8'd128;
            3'd3: trig_level = 8'd180;
            default: trig_level = 8'd75;
        endcase

        case (sample_div_idx[1:0])
            2'd0: sample_div = 32'd1;//50MHz
            2'd1: sample_div = 32'd5;//10MHz
            2'd2: sample_div = 32'd12;//~4.17MHz, ~3 cycles/window at 20kHz
            default: sample_div = 32'd50;//1MHz
        endcase
    end

   
    // Export UI state to display logic.
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            ui_page           <= PAGE_MAIN;
            ui_cursor         <= 4'd0;
            ui_curr_edit_mode      <= 1'b0;
            ui_curr_edit_value     <= 4'd0;
            ui_active_src_sel <= 3'd0;
        end else begin
            ui_page           <= curr_page;
            ui_cursor         <= curr_cursor;
            ui_curr_edit_mode      <= curr_edit_mode;
            ui_curr_edit_value     <= curr_edit_value;
            ui_active_src_sel <= active_src_sel;
        end
    end

endmodule
