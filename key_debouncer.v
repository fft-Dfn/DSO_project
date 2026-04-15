// -----------------------------------------------------------------------------
// key_debouncer
// -----------------------------------------------------------------------------
// Debounces mechanical key inputs by requiring a stable level for a configured
// number of clock cycles before updating the clean output state.
// -----------------------------------------------------------------------------

module key_debouncer #(
    parameter KEY_WIDTH = 5,

    parameter WAIT_TIME = 20'd1_000_000
)(
    input  wire                 clk_50m,

    input  wire [KEY_WIDTH-1:0] key_raw,

    output reg  [KEY_WIDTH-1:0] key_clean
);

    initial begin
        key_clean = {KEY_WIDTH{1'b1}};
    end

    reg [KEY_WIDTH-1:0] key_sync_0 = {KEY_WIDTH{1'b1}};
    reg [KEY_WIDTH-1:0] key_sync_1 = {KEY_WIDTH{1'b1}};

    always @(posedge clk_50m) begin
        key_sync_0 <= key_raw;
        key_sync_1 <= key_sync_0;
    end

    reg [19:0] cnt = 20'd0;

    wire state_changed = (key_sync_1 != key_clean);

    always @(posedge clk_50m) begin
        if (state_changed) begin

            cnt <= cnt + 1'b1;
            if (cnt == WAIT_TIME) begin

                key_clean <= key_sync_1;
                cnt <= 20'd0;
            end
        end else begin

            cnt <= 20'd0;
        end
    end

endmodule
