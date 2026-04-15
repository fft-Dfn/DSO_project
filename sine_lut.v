// -----------------------------------------------------------------------------
// sine_lut
// -----------------------------------------------------------------------------
// ROM lookup table for sine amplitude samples indexed by phase.
// -----------------------------------------------------------------------------

module sine_lut #(
    parameter ADDR_W = 8,
    parameter DATA_W = 8
)(
    input  wire                 clk,
    input  wire [ADDR_W-1:0]    addr,
    output reg  [DATA_W-1:0]    data
);

    (* rom_style = "block" *) reg [DATA_W-1:0] rom [0:(1<<ADDR_W)-1];

    initial begin
        $readmemh("sine_wave_256x8.hex", rom);
    end

    always @(posedge clk) begin
        data <= rom[addr];
    end

endmodule
