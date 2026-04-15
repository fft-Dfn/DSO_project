// -----------------------------------------------------------------------------
// hmi_controller
// -----------------------------------------------------------------------------
// Interaction state model:
// - Page/cursor navigation selects setting group and field.
// - Edit mode latches current value, applies step changes, and commits on confirmation.
// - Flash save/load requests are pulse-based and externally arbitrated by busy handshake.
//
// Key registers:
// - page/cursor/edit_mode: UI state machine core.
// - *_idx arrays: compact indices for per-source waveform parameters.
// - persist_save_req/persist_save_state: explicit commit channel to EEPROM persistence.
// -----------------------------------------------------------------------------

module hmi_controller (
    input  wire        clk_50m,
    input  wire        rst_n,

    input  wire        key_up_p,
    input  wire        key_down_p,
    input  wire        key_enter_p,
    input  wire        key_back_p,

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

    output reg         trig_mode,
    output reg         trig_edge,
    output reg  [7:0]  trig_level,
    output reg  [31:0] sample_div,

    output reg  [2:0]  sel_ch1,
    output reg  [2:0]  sel_ch2,
    output reg  [2:0]  sel_ch3,
    output reg  [2:0]  sel_ch4,
    output reg  [2:0]  sel_trig,

    output reg  [1:0]  flash_ch_sel,
    output reg         flash_write_req,
    output reg         flash_read_req,
    output reg         flash_cancel_req,
    input  wire        flash_txn_busy,

    output reg  [3:0]  ui_page,
    output reg  [3:0]  ui_cursor,
    output reg         ui_curr_edit_mode,
    output reg  [3:0]  ui_curr_edit_value,
    output reg  [2:0]  ui_active_src_sel,

    input  wire        persist_load_valid,
    input  wire [51:0] persist_load_state,
    output wire [51:0] persist_save_state,
    output reg         persist_save_req
);

    localparam PAGE_MAIN    = 4'd0;
    localparam PAGE_SRC     = 4'd1;
    localparam PAGE_SRC_CFG = 4'd2;
    localparam PAGE_TRIG    = 4'd3;
    localparam PAGE_DISP    = 4'd4;

    localparam MAIN_SIG_SRC = 4'd0;
    localparam MAIN_TRIG    = 4'd1;
    localparam MAIN_DISP    = 4'd2;
    localparam MAIN_STORE   = 4'd3;
    localparam MAIN_LOAD    = 4'd4;

    localparam SRC_CFG_FREQ  = 4'd0;
    localparam SRC_CFG_TYPE  = 4'd1;
    localparam SRC_CFG_PHASE = 4'd2;

    localparam TRIG_MODE_ITEM  = 4'd0;
    localparam TRIG_EDGE_ITEM  = 4'd1;
    localparam TRIG_LEVEL_ITEM = 4'd2;
    localparam TRIG_SAMP_ITEM  = 4'd3;

    localparam DISP_CH1_IN    = 4'd0;
    localparam DISP_CH2_IN    = 4'd1;
    localparam DISP_CH3_IN    = 4'd2;
    localparam DISP_CH4_IN    = 4'd3;
    localparam DISP_TRIG_IN   = 4'd4;
    localparam DISP_STORE_SEL = 4'd5;

    // Core UI state registers.
    reg [3:0] curr_page;
    reg [3:0] curr_cursor;
    reg       curr_edit_mode;
    reg [3:0] curr_edit_value;
    reg [2:0] active_src_sel;

    // Per-source compact configuration indices.
    reg [1:0] freq_idx [0:4];
    reg [1:0] type_idx [0:4];
    reg [1:0] phase_idx[0:4];

    // Trigger/display index form is used to keep UI edits bounded and deterministic.
    reg [2:0] trig_mode_idx;
    reg [2:0] trig_edge_idx;
    reg [2:0] trig_level_idx;
    reg [2:0] sample_div_idx;

    function [3:0] page_max_cursor;
        input [3:0] page;
        begin
            case (page)
                PAGE_MAIN:    page_max_cursor = 4'd4;
                PAGE_SRC:     page_max_cursor = 4'd4;
                PAGE_SRC_CFG: page_max_cursor = 4'd2;
                PAGE_TRIG:    page_max_cursor = 4'd3;
                PAGE_DISP:    page_max_cursor = 4'd5;
                default:      page_max_cursor = 4'd0;
            endcase
        end
    endfunction

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
                        DISP_STORE_SEL: edit_max_value = 4'd3;
                        default:        edit_max_value = 4'd0;
                    endcase
                end
                default: edit_max_value = 4'd0;
            endcase
        end
    endfunction

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
                        DISP_STORE_SEL: get_current_value = {2'b00, flash_ch_sel};
                        default:        get_current_value = 4'd0;
                    endcase
                end
            endcase
        end
    endfunction

    function [31:0] freq_word_from_idx;
        input [1:0] idx;
        begin
            case (idx)
                2'd0: freq_word_from_idx = 32'd858993;
                2'd1: freq_word_from_idx = 32'd1717987;
                2'd2: freq_word_from_idx = 32'd2576980;
                default: freq_word_from_idx = 32'd3435973;
            endcase
        end
    endfunction

    function [7:0] phase_from_idx;
        input [1:0] idx;
        begin
            case (idx)
                2'd0: phase_from_idx = 8'd0;
                2'd1: phase_from_idx = 8'd64;
                2'd2: phase_from_idx = 8'd128;
                default: phase_from_idx = 8'd192;
            endcase
        end
    endfunction

    function [7:0] trig_level_from_idx;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: trig_level_from_idx = 8'd25;
                3'd1: trig_level_from_idx = 8'd75;
                3'd2: trig_level_from_idx = 8'd128;
                3'd3: trig_level_from_idx = 8'd180;
                default: trig_level_from_idx = 8'd75;
            endcase
        end
    endfunction

    function [31:0] sample_div_from_idx;
        input [2:0] idx;
        begin
            case (idx[1:0])
                2'd0: sample_div_from_idx = 32'd1;
                2'd1: sample_div_from_idx = 32'd5;
                2'd2: sample_div_from_idx = 32'd12;
                default: sample_div_from_idx = 32'd50;
            endcase
        end
    endfunction

    assign persist_save_state = {
        sel_trig[2:0],
        sel_ch4[2:0],
        sel_ch3[2:0],
        sel_ch2[2:0],
        sel_ch1[2:0],
        sample_div_idx[1:0],
        trig_level_idx[2:0],
        trig_edge_idx[0],
        trig_mode_idx[0],
        phase_idx[4], phase_idx[3], phase_idx[2], phase_idx[1], phase_idx[0],
        type_idx[4],  type_idx[3],  type_idx[2],  type_idx[1],  type_idx[0],
        freq_idx[4],  freq_idx[3],  freq_idx[2],  freq_idx[1],  freq_idx[0]
    };

    // Navigation-mode behavior:
    // - UP/DOWN moves cursor within current page.
    // - ENTER either enters child page/edit mode or emits flash request pulse.
    // - BACK returns to parent page while preserving selected source context.
    task handle_nav_mode;
        begin
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
                                if (flash_txn_busy)
                                    flash_cancel_req <= 1'b1;
                                else
                                    flash_write_req <= 1'b1;
                            end
                            MAIN_LOAD: begin
                                if (flash_txn_busy)
                                    flash_cancel_req <= 1'b1;
                                else
                                    flash_read_req <= 1'b1;
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
        end
    endtask

    // Edit-mode behavior:
    // - UP/DOWN modifies temporary edit value within field-specific bounds.
    // - ENTER commits to backing indices/signals and emits persist_save_req when needed.
    // - BACK discards temporary edit and returns to navigation mode.
    task handle_edit_mode;
        begin
            if (key_up_p) begin
                if (curr_edit_value > 0)
                    curr_edit_value <= curr_edit_value - 1'b1;
            end else if (key_down_p) begin
                if (curr_edit_value < edit_max_value(curr_page, curr_cursor))
                    curr_edit_value <= curr_edit_value + 1'b1;
            end else if (key_enter_p) begin

                case (curr_page)
                    PAGE_SRC_CFG: begin
                        case (curr_cursor)
                            SRC_CFG_FREQ:  freq_idx[active_src_sel]  <= curr_edit_value[1:0];
                            SRC_CFG_TYPE:  type_idx[active_src_sel]  <= curr_edit_value[1:0];
                            SRC_CFG_PHASE: phase_idx[active_src_sel] <= curr_edit_value[1:0];
                        endcase
                        persist_save_req <= 1'b1;
                    end

                    PAGE_TRIG: begin
                        case (curr_cursor)
                            TRIG_MODE_ITEM:  trig_mode_idx  <= curr_edit_value[2:0];
                            TRIG_EDGE_ITEM:  trig_edge_idx  <= curr_edit_value[2:0];
                            TRIG_LEVEL_ITEM: trig_level_idx <= curr_edit_value[2:0];
                            TRIG_SAMP_ITEM:  sample_div_idx <= curr_edit_value[2:0];
                        endcase
                        persist_save_req <= 1'b1;
                    end

                    PAGE_DISP: begin
                        case (curr_cursor)
                            DISP_CH1_IN:    sel_ch1      <= curr_edit_value[2:0];
                            DISP_CH2_IN:    sel_ch2      <= curr_edit_value[2:0];
                            DISP_CH3_IN:    sel_ch3      <= curr_edit_value[2:0];
                            DISP_CH4_IN:    sel_ch4      <= curr_edit_value[2:0];
                            DISP_TRIG_IN:   sel_trig     <= curr_edit_value[2:0];
                            DISP_STORE_SEL: flash_ch_sel <= curr_edit_value[1:0];
                        endcase
                        if (curr_cursor <= DISP_TRIG_IN)
                            persist_save_req <= 1'b1;
                    end
                endcase

                curr_edit_mode <= 1'b0;
            end else if (key_back_p) begin

                curr_edit_mode <= 1'b0;
            end
        end
    endtask

    // Main UI control FSM register process.
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            curr_page      <= PAGE_MAIN;
            curr_cursor    <= 4'd0;
            curr_edit_mode <= 1'b0;
            curr_edit_value <= 4'd0;
            active_src_sel <= 3'd0;

            freq_idx[0]  <= 2'd1;
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

            freq_idx[4]  <= 2'd1;
            type_idx[4]  <= 2'd1;
            phase_idx[4] <= 2'd0;

            trig_mode_idx  <= 3'd1;
            trig_edge_idx  <= 3'd0;
            trig_level_idx <= 3'd2;
            sample_div_idx <= 3'd2;

            sel_ch1      <= 3'd0;
            sel_ch2      <= 3'd1;
            sel_ch3      <= 3'd2;
            sel_ch4      <= 3'd3;
            sel_trig     <= 3'd4;
            flash_ch_sel    <= 2'd0;
            flash_write_req <= 1'b0;
            flash_read_req  <= 1'b0;
            flash_cancel_req <= 1'b0;
            persist_save_req <= 1'b0;
        end else begin

            flash_write_req <= 1'b0;
            flash_read_req  <= 1'b0;
            flash_cancel_req <= 1'b0;
            persist_save_req <= 1'b0;

            // Persist restore has priority over key handling so boot-load can overwrite defaults.
            if (persist_load_valid) begin
                freq_idx[0] <= persist_load_state[1:0];
                freq_idx[1] <= persist_load_state[3:2];
                freq_idx[2] <= persist_load_state[5:4];
                freq_idx[3] <= persist_load_state[7:6];
                freq_idx[4] <= persist_load_state[9:8];

                type_idx[0] <= persist_load_state[11:10];
                type_idx[1] <= persist_load_state[13:12];
                type_idx[2] <= persist_load_state[15:14];
                type_idx[3] <= persist_load_state[17:16];
                type_idx[4] <= persist_load_state[19:18];

                phase_idx[0] <= persist_load_state[21:20];
                phase_idx[1] <= persist_load_state[23:22];
                phase_idx[2] <= persist_load_state[25:24];
                phase_idx[3] <= persist_load_state[27:26];
                phase_idx[4] <= persist_load_state[29:28];

                trig_mode_idx <= {2'b00, persist_load_state[30]};
                trig_edge_idx <= {2'b00, persist_load_state[31]};

                if (persist_load_state[34:32] <= 3'd4)
                    trig_level_idx <= persist_load_state[34:32];
                else
                    trig_level_idx <= 3'd2;

                sample_div_idx <= {1'b0, persist_load_state[36:35]};

                if (persist_load_state[39:37] <= 3'd4) sel_ch1 <= persist_load_state[39:37]; else sel_ch1 <= 3'd0;
                if (persist_load_state[42:40] <= 3'd4) sel_ch2 <= persist_load_state[42:40]; else sel_ch2 <= 3'd1;
                if (persist_load_state[45:43] <= 3'd4) sel_ch3 <= persist_load_state[45:43]; else sel_ch3 <= 3'd2;
                if (persist_load_state[48:46] <= 3'd4) sel_ch4 <= persist_load_state[48:46]; else sel_ch4 <= 3'd3;
                if (persist_load_state[51:49] <= 3'd4) sel_trig <= persist_load_state[51:49]; else sel_trig <= 3'd4;
            end else begin
                if (!curr_edit_mode)
                    handle_nav_mode;
                else
                    handle_edit_mode;
            end
        end
    end

    // Decode compact indices into runtime control words.
    always @(*) begin
        dds_freq_a = freq_word_from_idx(freq_idx[0]);
        dds_freq_b = freq_word_from_idx(freq_idx[1]);
        dds_freq_c = freq_word_from_idx(freq_idx[2]);
        dds_freq_d = freq_word_from_idx(freq_idx[3]);
        dds_freq_e = freq_word_from_idx(freq_idx[4]);

        dds_type_a = type_idx[0];
        dds_type_b = type_idx[1];
        dds_type_c = type_idx[2];
        dds_type_d = type_idx[3];
        dds_type_e = type_idx[4];

        dds_phase_a = phase_from_idx(phase_idx[0]);
        dds_phase_b = phase_from_idx(phase_idx[1]);
        dds_phase_c = phase_from_idx(phase_idx[2]);
        dds_phase_d = phase_from_idx(phase_idx[3]);
        dds_phase_e = phase_from_idx(phase_idx[4]);

        trig_mode = trig_mode_idx[0];
        trig_edge = trig_edge_idx[0];

        trig_level = trig_level_from_idx(trig_level_idx);
        sample_div = sample_div_from_idx(sample_div_idx);
    end

    // One-cycle delayed mirror exported to renderer to avoid combinational fanout.
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            ui_page           <= PAGE_MAIN;
            ui_cursor         <= 4'd0;
            ui_curr_edit_mode <= 1'b0;
            ui_curr_edit_value <= 4'd0;
            ui_active_src_sel <= 3'd0;
        end else begin
            ui_page           <= curr_page;
            ui_cursor         <= curr_cursor;
            ui_curr_edit_mode <= curr_edit_mode;
            ui_curr_edit_value <= curr_edit_value;
            ui_active_src_sel <= active_src_sel;
        end
    end

endmodule
