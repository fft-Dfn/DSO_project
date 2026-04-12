// -----------------------------------------------------------------------------
// font8x12_rom
// -----------------------------------------------------------------------------
// Purpose:
//   ASCII 8x12 font ROM initialized from hex file.
//   Storage style follows sine LUT style (readmemh-initialized ROM).
// -----------------------------------------------------------------------------
module font8x12_rom (
    input  wire [7:0] char_code,
    input  wire [3:0] row_idx,   // 0..11
    output wire [7:0] row_bits
);

    localparam GLYPH_H = 12;
    localparam MEM_D   = 128 * GLYPH_H;

    (* rom_style = "block" *) reg [7:0] rom [0:MEM_D-1];
    wire [10:0] base_addr = (char_code[6:0] << 3) + (char_code[6:0] << 2); // *12
    wire [10:0] addr = base_addr + row_idx;

    initial begin
        $readmemh("font8x12.hex", rom);
    end

    assign row_bits = (row_idx < GLYPH_H) ? rom[addr] : 8'h00;

endmodule

