// -----------------------------------------------------------------------------
// ui_value_map
// -----------------------------------------------------------------------------
// Centralized mapping from runtime numeric values to UI character codes.
// Keeps display formatting logic out of controller and renderer modules.
// -----------------------------------------------------------------------------

module ui_value_map (
    input  wire        trig_mode,
    input  wire        trig_edge,
    input  wire [7:0]  trig_level,
    input  wire [31:0] sample_div,
    input  wire [2:0]  sel_trig,

    output reg  [31:0] mode_text,
    output reg  [7:0]  edge_char,
    output reg  [23:0] level_text,
    output reg  [31:0] div_text,
    output reg  [7:0]  trig_src_char
);

    function [7:0] src_char;
        input [2:0] sel;
        begin
            case (sel)
                3'd0: src_char = 8'h61;
                3'd1: src_char = 8'h62;
                3'd2: src_char = 8'h63;
                3'd3: src_char = 8'h64;
                3'd4: src_char = 8'h65;
                default: src_char = 8'h3f;
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

        if (trig_mode)
            mode_text = {8'h61, 8'h75, 8'h74, 8'h6f};
        else
            mode_text = {8'h6e, 8'h6f, 8'h72, 8'h6d};

        edge_char = trig_edge ? 8'h66 : 8'h72;

        case (trig_level)
            8'd0:   level_text = {8'h30, 8'h30, 8'h30};
            8'd64:  level_text = {8'h30, 8'h36, 8'h34};
            8'd128: level_text = {8'h31, 8'h32, 8'h38};
            8'd192: level_text = {8'h31, 8'h39, 8'h32};
            8'd255: level_text = {8'h32, 8'h35, 8'h35};
            default:level_text = {8'h78, hex_char(trig_level[7:4]), hex_char(trig_level[3:0])};
        endcase

        case (sample_div)
            32'd1:   div_text = {8'h31, 8'h20, 8'h20, 8'h20};
            32'd5:   div_text = {8'h35, 8'h20, 8'h20, 8'h20};
            32'd12:  div_text = {8'h31, 8'h32, 8'h20, 8'h20};
            32'd50:  div_text = {8'h35, 8'h30, 8'h20, 8'h20};
            default: div_text = {8'h63, 8'h75, 8'h73, 8'h74};
        endcase

        trig_src_char = src_char(sel_trig);
    end

endmodule
