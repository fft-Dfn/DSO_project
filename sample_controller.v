// -----------------------------------------------------------------------------
// sample_controller
// -----------------------------------------------------------------------------
// Purpose:
//   Capture 4 selected channels into a circular RAM window with trigger support.
//
// Capture Flow:
//   S_IDLE    -> wait for rearm
//   S_PREFILL -> fill pre-trigger half window
//   S_ARMED   -> wait trigger edge (or auto-timeout force trigger)
//   S_POST    -> capture post-trigger half window, then pulse capture_done
//
// Key Signals (English):
//   - sample_tick: decimated sampling strobe from sample_div.
//   - edge_fired: configured trigger edge condition met.
//   - force_req: auto-mode timeout fallback trigger request.
//   - frame_start_addr: logical start of display window around trigger point.
// -----------------------------------------------------------------------------
module sample_controller #(
    parameter DATA_W       = 8,
    parameter ADDR_W       = 10,
    parameter AUTO_TIMEOUT = 32'd25_000_000 // half second at 50 MHz
)(
    input  wire              clk,
    input  wire              rst_n,

    input  wire [DATA_W-1:0] in_a,
    input  wire [DATA_W-1:0] in_b,
    input  wire [DATA_W-1:0] in_c,
    input  wire [DATA_W-1:0] in_d,
    input  wire [DATA_W-1:0] in_e,

    input  wire [2:0]        sel_trig,
    input  wire [2:0]        sel_ch1,
    input  wire [2:0]        sel_ch2,
    input  wire [2:0]        sel_ch3,
    input  wire [2:0]        sel_ch4,
    input  wire              trig_mode,
    input  wire              trig_edge,
    input  wire [DATA_W-1:0] trig_level,
    input  wire [31:0]       sample_div,

    input  wire              rearm,
    output reg               capture_done,
    output reg  [ADDR_W-1:0] frame_start_addr,
    output reg               ram_we,

    output reg  [ADDR_W-1:0] ram_waddr,
    output reg  [DATA_W-1:0] ram_wdata_ch1,
    output reg  [DATA_W-1:0] ram_wdata_ch2,
    output reg  [DATA_W-1:0] ram_wdata_ch3,
    output reg  [DATA_W-1:0] ram_wdata_ch4
);

    localparam DEPTH      = (1 << ADDR_W);
    localparam HALF_DEPTH = (DEPTH >> 1);
    localparam POST_COUNT = HALF_DEPTH - 1;

    localparam S_IDLE    = 2'd0;
    localparam S_PREFILL = 2'd1;
    localparam S_ARMED   = 2'd2;
    localparam S_POST    = 2'd3;

    function [DATA_W-1:0] mux_5to1;
        input [2:0]        sel;
        input [DATA_W-1:0] a, b, c, d, e;
        begin
            case (sel)
                3'd0: mux_5to1 = a;
                3'd1: mux_5to1 = b;
                3'd2: mux_5to1 = c;
                3'd3: mux_5to1 = d;
                3'd4: mux_5to1 = e;
                default: mux_5to1 = 8'd127;
            endcase
        end
    endfunction

    wire [DATA_W-1:0] trig_sig = mux_5to1(sel_trig, in_a, in_b, in_c, in_d, in_e);
    wire [DATA_W-1:0] ch1_sig  = mux_5to1(sel_ch1,  in_a, in_b, in_c, in_d, in_e);
    wire [DATA_W-1:0] ch2_sig  = mux_5to1(sel_ch2,  in_a, in_b, in_c, in_d, in_e);
    wire [DATA_W-1:0] ch3_sig  = mux_5to1(sel_ch3,  in_a, in_b, in_c, in_d, in_e);
    wire [DATA_W-1:0] ch4_sig  = mux_5to1(sel_ch4,  in_a, in_b, in_c, in_d, in_e);

    reg [31:0] div_cnt;
    reg        sample_tick;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt     <= 32'd0;
            sample_tick <= 1'b0;
        end else if (sample_div <= 32'd1) begin
            div_cnt     <= 32'd0;
            sample_tick <= 1'b1;
        end else if (div_cnt == sample_div - 1'b1) begin
            div_cnt     <= 32'd0;
            sample_tick <= 1'b1;
        end else begin
            div_cnt     <= div_cnt + 1'b1;
            sample_tick <= 1'b0;
        end
    end

    reg [DATA_W-1:0] trig_cur, trig_last;
    reg [DATA_W-1:0] ch1_cur, ch2_cur, ch3_cur, ch4_cur;
    reg              tick_d1;

    wire is_rising  = (trig_last < trig_level) && (trig_cur >= trig_level);
    wire is_falling = (trig_last > trig_level) && (trig_cur <= trig_level);
    wire edge_fired = tick_d1 && ((trig_edge == 1'b0) ? is_rising : is_falling);

    reg [31:0] auto_cnt;
    reg        force_req;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trig_cur    <= {DATA_W{1'b0}};
            trig_last   <= {DATA_W{1'b0}};
            ch1_cur     <= {DATA_W{1'b0}};
            ch2_cur     <= {DATA_W{1'b0}};
            ch3_cur     <= {DATA_W{1'b0}};
            ch4_cur     <= {DATA_W{1'b0}};
            tick_d1     <= 1'b0;
        end else begin
            tick_d1     <= sample_tick;
            if (sample_tick) begin
                trig_last <= trig_cur;
                trig_cur  <= trig_sig;
                ch1_cur   <= ch1_sig;
                ch2_cur   <= ch2_sig;
                ch3_cur   <= ch3_sig;
                ch4_cur   <= ch4_sig;
            end
        end
    end

    reg [1:0]        state;
    reg [ADDR_W-1:0] wr_ptr;
    reg [ADDR_W-1:0] trig_addr;
    reg [ADDR_W-1:0] post_cnt;
    reg [ADDR_W-1:0] prefill_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_IDLE;
            wr_ptr           <= {ADDR_W{1'b0}};
            trig_addr        <= {ADDR_W{1'b0}};
            post_cnt         <= {ADDR_W{1'b0}};
            prefill_cnt      <= {ADDR_W{1'b0}};
            auto_cnt         <= 32'd0;
            force_req        <= 1'b0;
            capture_done     <= 1'b0;
            frame_start_addr <= {ADDR_W{1'b0}};
            ram_we           <= 1'b0;
            ram_waddr        <= {ADDR_W{1'b0}};
            ram_wdata_ch1    <= {DATA_W{1'b0}};
            ram_wdata_ch2    <= {DATA_W{1'b0}};
            ram_wdata_ch3    <= {DATA_W{1'b0}};
            ram_wdata_ch4    <= {DATA_W{1'b0}};
        end else begin
            capture_done <= 1'b0;
            ram_we       <= 1'b0;

            case (state)
                S_IDLE: begin
                    wr_ptr      <= {ADDR_W{1'b0}};
                    prefill_cnt <= {ADDR_W{1'b0}};
                    post_cnt    <= {ADDR_W{1'b0}};
                    auto_cnt    <= 32'd0;
                    force_req   <= 1'b0;
                    if (rearm) begin
                        state <= S_PREFILL;
                    end
                end

                S_PREFILL: begin
                    if (tick_d1) begin
                        ram_we        <= 1'b1;
                        ram_waddr     <= wr_ptr;
                        ram_wdata_ch1 <= ch1_cur;
                        ram_wdata_ch2 <= ch2_cur;
                        ram_wdata_ch3 <= ch3_cur;
                        ram_wdata_ch4 <= ch4_cur;
                        wr_ptr        <= wr_ptr + 1'b1;
                        if (prefill_cnt == HALF_DEPTH - 1) begin
                            prefill_cnt <= {ADDR_W{1'b0}};
                            auto_cnt    <= 32'd0;
                            force_req   <= 1'b0;
                            state       <= S_ARMED;
                        end else begin
                            prefill_cnt <= prefill_cnt + 1'b1;
                        end
                    end
                end

                S_ARMED: begin
                    if (trig_mode == 1'b1) begin
                        if (!force_req) begin
                            if (auto_cnt < AUTO_TIMEOUT)
                                auto_cnt <= auto_cnt + 1'b1;
                            else
                                force_req <= 1'b1;
                        end
                    end else begin
                        auto_cnt <= 32'd0;
                        force_req <= 1'b0;
                    end

                    if (tick_d1) begin
                        ram_we        <= 1'b1;
                        ram_waddr     <= wr_ptr;
                        ram_wdata_ch1 <= ch1_cur;
                        ram_wdata_ch2 <= ch2_cur;
                        ram_wdata_ch3 <= ch3_cur;
                        ram_wdata_ch4 <= ch4_cur;

                        if (edge_fired || force_req) begin
                            trig_addr <= wr_ptr;
                            wr_ptr    <= wr_ptr + 1'b1;
                            post_cnt  <= {ADDR_W{1'b0}};
                            auto_cnt  <= 32'd0;
                            force_req <= 1'b0;
                            state     <= S_POST;
                        end else begin
                            wr_ptr <= wr_ptr + 1'b1;
                        end
                    end
                end

                S_POST: begin
                    if (tick_d1) begin
                        ram_we        <= 1'b1;
                        ram_waddr     <= wr_ptr;
                        ram_wdata_ch1 <= ch1_cur;
                        ram_wdata_ch2 <= ch2_cur;
                        ram_wdata_ch3 <= ch3_cur;
                        ram_wdata_ch4 <= ch4_cur;

                        if (post_cnt == POST_COUNT[ADDR_W-1:0]) begin
                            capture_done     <= 1'b1;
                            frame_start_addr <= trig_addr - HALF_DEPTH[ADDR_W-1:0];
                            wr_ptr           <= {ADDR_W{1'b0}};
                            post_cnt         <= {ADDR_W{1'b0}};
                            auto_cnt         <= 32'd0;
                            state            <= S_IDLE;
                        end else begin
                            wr_ptr   <= wr_ptr + 1'b1;
                            post_cnt <= post_cnt + 1'b1;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
