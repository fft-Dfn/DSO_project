// -----------------------------------------------------------------------------
// key_debouncer
// -----------------------------------------------------------------------------
// Purpose:
//   Debounce active-low mechanical keys.
//   Uses 2-FF synchronizer + stable-time counter filter.
// -----------------------------------------------------------------------------
module key_debouncer #(
    parameter KEY_WIDTH = 5,
    // At 50 MHz, 20 ms equals 1,000,000 cycles.
    parameter WAIT_TIME = 20'd1_000_000 
)(
    input  wire                 clk_50m,
    
    // Raw asynchronous key inputs.
    input  wire [KEY_WIDTH-1:0] key_raw,
    
    // Debounced, synchronized key outputs.
    output reg  [KEY_WIDTH-1:0] key_clean
);
    // Power-on initialization:
    // The reset key itself is debounced in this module, so all internal registers
    // should have explicit init values. Pull-up keys are idle-high.

    initial begin
        key_clean = {KEY_WIDTH{1'b1}};
    end


    // Two-stage synchronizer to reduce metastability risk.
    reg [KEY_WIDTH-1:0] key_sync_0 = {KEY_WIDTH{1'b1}};
    reg [KEY_WIDTH-1:0] key_sync_1 = {KEY_WIDTH{1'b1}};

    always @(posedge clk_50m) begin
        key_sync_0 <= key_raw;
        key_sync_1 <= key_sync_0;
    end

    // State-change detection and stable-time counting.
    reg [19:0] cnt = 20'd0;
    
    // Input differs from stable output while key is moving/bouncing.
    wire state_changed = (key_sync_1 != key_clean);

    always @(posedge clk_50m) begin
        if (state_changed) begin
            // Keep counting while candidate state remains different.
            cnt <= cnt + 1'b1;
            if (cnt == WAIT_TIME) begin
                // Stable for WAIT_TIME: accept new debounced state.
                key_clean <= key_sync_1;
                cnt <= 20'd0;
            end
        end else begin
            // Bounce or returned state: clear counter and restart.
            cnt <= 20'd0;
        end
    end

endmodule
