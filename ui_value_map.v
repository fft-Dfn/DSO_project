// -----------------------------------------------------------------------------
// ui_value_map
// -----------------------------------------------------------------------------
// Purpose:
//   Centralize value-to-text mapping for UI overlay.
//   Keeps mapping logic out of both HMI controller and VGA renderer.
// -----------------------------------------------------------------------------
module ui_value_map (
    input  wire        trig_mode,
    input  wire        trig_edge,
    input  wire [7:0]  trig_level,
    input  wire [31:0] sample_div,
    input  wire [2:0]  sel_trig,
    input  wire [2:0]  view_ch_sel,

    output reg  [31:0] mode_text,      // 4 chars: "auto"/"norm"
    output reg  [7:0]  edge_char,      // 'r' or 'f'
    output reg  [23:0] level_text,     // 3 chars (known decimal points or hex fallback)
    output reg  [31:0] div_text,       // 4 chars fixed lookup
    output reg  [7:0]  trig_src_char,  // 'a'..'e'
    output reg  [23:0] view_text       // 3 chars: "ch1".."ch4"/"fls"
);

    function [7:0] src_char;
        input [2:0] sel;
        begin
            case (sel)
                3'd0: src_char = 8'h61; // a
                3'd1: src_char = 8'h62; // b
                3'd2: src_char = 8'h63; // c
                3'd3: src_char = 8'h64; // d
                3'd4: src_char = 8'h65; // e
                default: src_char = 8'h3f; // ?
            endcase
        end
    endfunction

    function [7:0] hex_char;
        input [3:0] v;
        begin
            if (v < 4'd10)
                hex_char = 8'h30 + v;
            else
                hex_char = 8'h61 + (v - 4'd10);
        end
    endfunction

    always @(*) begin
        // mode
        if (trig_mode)
            mode_text = {8'h61, 8'h75, 8'h74, 8'h6f}; // auto
        else
            mode_text = {8'h6e, 8'h6f, 8'h72, 8'h6d}; // norm

        // edge
        edge_char = trig_edge ? 8'h66 : 8'h72; // f/r

        // level text:
        // common trigger levels keep decimal readability; fallback is xhh (hex).
        case (trig_level)
            8'd0:   level_text = {8'h30, 8'h30, 8'h30}; // 000
            8'd64:  level_text = {8'h30, 8'h36, 8'h34}; // 064
            8'd128: level_text = {8'h31, 8'h32, 8'h38}; // 128
            8'd192: level_text = {8'h31, 8'h39, 8'h32}; // 192
            8'd255: level_text = {8'h32, 8'h35, 8'h35}; // 255
            default:level_text = {8'h78, hex_char(trig_level[7:4]), hex_char(trig_level[3:0])}; // xhh
        endcase

        // divider text: fixed lookup, no runtime division.
        case (sample_div)
            32'd1:   div_text = {8'h31, 8'h20, 8'h20, 8'h20}; // "1   "
            32'd5:   div_text = {8'h35, 8'h20, 8'h20, 8'h20}; // "5   "
            32'd12:  div_text = {8'h31, 8'h32, 8'h20, 8'h20}; // "12  "
            32'd50:  div_text = {8'h35, 8'h30, 8'h20, 8'h20}; // "50  "
            default: div_text = {8'h63, 8'h75, 8'h73, 8'h74}; // "cust"
        endcase

        trig_src_char   = src_char(sel_trig);

        case (view_ch_sel)
            3'd0: view_text = {8'h63, 8'h68, 8'h31}; // ch1
            3'd1: view_text = {8'h63, 8'h68, 8'h32}; // ch2
            3'd2: view_text = {8'h63, 8'h68, 8'h33}; // ch3
            3'd3: view_text = {8'h63, 8'h68, 8'h34}; // ch4
            default: view_text = {8'h66, 8'h6c, 8'h73}; // fls
        endcase
    end

endmodule
