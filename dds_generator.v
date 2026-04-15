// -----------------------------------------------------------------------------
// dds_generator
// -----------------------------------------------------------------------------
// Synthesizes selectable waveform outputs from frequency word and phase offset.
// -----------------------------------------------------------------------------

module dds_generator #(
    parameter integer PHASE_W  = 32,
    parameter integer TABLE_AW = 8,
    parameter integer DATA_W   = 8
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire [PHASE_W-1:0]    freq_word,
    input  wire [TABLE_AW-1:0]   phase_offset,
    input  wire [1:0]            wave_type,

    output reg  [DATA_W-1:0]     wave_data
);

    localparam TRUNC_POS = PHASE_W - TABLE_AW;

    reg [PHASE_W-1:0] phase_acc;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) phase_acc <= {PHASE_W{1'b0}};
        else        phase_acc <= phase_acc + freq_word;
    end

    wire [TABLE_AW-1:0] curr_phase;
    assign curr_phase = phase_acc[PHASE_W-1 : TRUNC_POS] + phase_offset;

    wire [DATA_W-1:0] sin_out;
    sine_lut #(
        .ADDR_W(TABLE_AW),
        .DATA_W(DATA_W)
    ) u_sine_lut (
        .clk  (clk),
        .addr (curr_phase),
        .data (sin_out)
    );

    wire [DATA_W-1:0] sqr_out = curr_phase[TABLE_AW-1] ? {8'b1100_0000} : {8'b0100_0000};
    wire [DATA_W-1:0] tri_out = curr_phase[TABLE_AW-1] ?
                                (~curr_phase[TABLE_AW-2:0] << (DATA_W - (TABLE_AW-1))) :
                                ( curr_phase[TABLE_AW-2:0] << (DATA_W - (TABLE_AW-1)));
    wire [DATA_W-1:0] saw_out = curr_phase << (DATA_W - TABLE_AW);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) wave_data <= {DATA_W{1'b0}};
        else begin
            case (wave_type)
                2'b00:   wave_data <= sin_out;
                2'b01:   wave_data <= sqr_out;
                2'b10:   wave_data <= tri_out;
                2'b11:   wave_data <= saw_out;
                default: wave_data <= {DATA_W{1'b0}};
            endcase
        end
    end
endmodule
