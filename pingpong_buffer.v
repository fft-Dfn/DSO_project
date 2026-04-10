module pingpong_buffer #(
    parameter DATA_W = 8,
    parameter ADDR_W = 10,
    parameter ACK_TIMEOUT_CYCLES = 22'd2_000_000
)(
    input  wire                  rst_n_write,
    input  wire                  rst_n_read,

    // write side
    input  wire                  clk_write,
    input  wire [ADDR_W-1:0]     waddr,
    input  wire [DATA_W-1:0]     wdata_ch1,
    input  wire [DATA_W-1:0]     wdata_ch2,
    input  wire [DATA_W-1:0]     wdata_ch3,
    input  wire [DATA_W-1:0]     wdata_ch4,

    // read side
    input  wire                  clk_read,
    input  wire [ADDR_W-1:0]     raddr,
    output reg  [DATA_W-1:0]     rdata_ch1,
    output reg  [DATA_W-1:0]     rdata_ch2,
    output reg  [DATA_W-1:0]     rdata_ch3,
    output reg  [DATA_W-1:0]     rdata_ch4,

    // control
    input  wire                  capture_done,
    input  wire                  we,
    output reg                   frame_valid,
    input  wire                  rd_frame_done,
    output reg                   overflow,
    input  wire [ADDR_W-1:0]     frame_start_addr,
    output reg  [ADDR_W-1:0]     active_frame_start_addr,
    output wire                  capture_ready
);

    wire [31:0] wdata_packed = {wdata_ch4, wdata_ch3, wdata_ch2, wdata_ch1};

    reg wr_bank_sel;
    reg wr_wait_ack;
    reg active_bank_rd;
    reg [21:0] ack_wait_cnt;

    // While waiting read-side commit ack, block new writes and stop next capture.
    wire we_eff = we && !wr_wait_ack;
    assign capture_ready = !wr_wait_ack;

    wire bank0_we_a = we_eff && (wr_bank_sel == 1'b0);
    wire bank1_we_a = we_eff && (wr_bank_sel == 1'b1);

    wire [31:0] bank0_rdata_b;
    wire [31:0] bank1_rdata_b;
    wire [31:0] bank0_rdata_a_unused;
    wire [31:0] bank1_rdata_a_unused;

    pingpong_bram_refact u_pingpong_bram_bank0 (
        .we_a    (bank0_we_a),
        .addr_a  (waddr),
        .wdata_a (wdata_packed),
        .rdata_a (bank0_rdata_a_unused),
        .rdata_b (bank0_rdata_b),
        .addr_b  (raddr),
        .wdata_b (32'd0),
        .clk_a   (clk_write),
        .clk_b   (clk_read)
    );

    pingpong_bram_refact u_pingpong_bram_bank1 (
        .we_a    (bank1_we_a),
        .addr_a  (waddr),
        .wdata_a (wdata_packed),
        .rdata_a (bank1_rdata_a_unused),
        .rdata_b (bank1_rdata_b),
        .addr_b  (raddr),
        .wdata_b (32'd0),
        .clk_a   (clk_write),
        .clk_b   (clk_read)
    );

    // read->write CDC: commit ack toggle
    reg commit_ack_tog_rd;
    reg commit_ack_sync1_wr, commit_ack_sync2_wr, commit_ack_sync2_d_wr;
    wire commit_ack_pulse_wr = commit_ack_sync2_wr ^ commit_ack_sync2_d_wr;
    always @(posedge clk_write or negedge rst_n_write) begin
        if (!rst_n_write) begin
            commit_ack_sync1_wr   <= 1'b0;
            commit_ack_sync2_wr   <= 1'b0;
            commit_ack_sync2_d_wr <= 1'b0;
        end else begin
            commit_ack_sync1_wr   <= commit_ack_tog_rd;
            commit_ack_sync2_wr   <= commit_ack_sync1_wr;
            commit_ack_sync2_d_wr <= commit_ack_sync2_wr;
        end
    end

    // write domain publish events
    reg frame_done_tog_bank0_wr, frame_done_tog_bank1_wr;
    reg [ADDR_W-1:0] frame_start_addr_bank0_wr, frame_start_addr_bank1_wr;

    always @(posedge clk_write or negedge rst_n_write) begin
        if (!rst_n_write) begin
            wr_bank_sel               <= 1'b0;
            wr_wait_ack               <= 1'b0;
            ack_wait_cnt              <= 22'd0;
            frame_done_tog_bank0_wr   <= 1'b0;
            frame_done_tog_bank1_wr   <= 1'b0;
            frame_start_addr_bank0_wr <= {ADDR_W{1'b0}};
            frame_start_addr_bank1_wr <= {ADDR_W{1'b0}};
            overflow                  <= 1'b0;
        end else begin
            overflow <= 1'b0;

            if (wr_wait_ack) begin
                if (commit_ack_pulse_wr || (ack_wait_cnt >= ACK_TIMEOUT_CYCLES)) begin
                    wr_bank_sel  <= ~wr_bank_sel;
                    wr_wait_ack  <= 1'b0;
                    ack_wait_cnt <= 22'd0;
                    if (ack_wait_cnt >= ACK_TIMEOUT_CYCLES)
                        overflow <= 1'b1;
                end else begin
                    ack_wait_cnt <= ack_wait_cnt + 1'b1;
                end
            end

            if (capture_done && !wr_wait_ack) begin
                if (wr_bank_sel == 1'b0) begin
                    frame_start_addr_bank0_wr <= frame_start_addr;
                    frame_done_tog_bank0_wr   <= ~frame_done_tog_bank0_wr;
                end else begin
                    frame_start_addr_bank1_wr <= frame_start_addr;
                    frame_done_tog_bank1_wr   <= ~frame_done_tog_bank1_wr;
                end
                ack_wait_cnt <= 22'd0;
                wr_wait_ack <= 1'b1;
            end

            // Indicates blocked writes/captures while waiting read commit.
            if (wr_wait_ack && (we || capture_done)) begin
                overflow <= 1'b1;
            end
        end
    end

    // write->read CDC
    reg done0_sync1_rd, done0_sync2_rd, done0_sync2_d_rd;
    reg done1_sync1_rd, done1_sync2_rd, done1_sync2_d_rd;
    wire done0_pulse_rd = done0_sync2_rd ^ done0_sync2_d_rd;
    wire done1_pulse_rd = done1_sync2_rd ^ done1_sync2_d_rd;

    reg [ADDR_W-1:0] frame_start_addr_bank0_rd_sync1, frame_start_addr_bank0_rd_sync2;
    reg [ADDR_W-1:0] frame_start_addr_bank1_rd_sync1, frame_start_addr_bank1_rd_sync2;
    always @(posedge clk_read or negedge rst_n_read) begin
        if (!rst_n_read) begin
            done0_sync1_rd <= 1'b0;
            done0_sync2_rd <= 1'b0;
            done0_sync2_d_rd <= 1'b0;
            done1_sync1_rd <= 1'b0;
            done1_sync2_rd <= 1'b0;
            done1_sync2_d_rd <= 1'b0;
            frame_start_addr_bank0_rd_sync1 <= {ADDR_W{1'b0}};
            frame_start_addr_bank0_rd_sync2 <= {ADDR_W{1'b0}};
            frame_start_addr_bank1_rd_sync1 <= {ADDR_W{1'b0}};
            frame_start_addr_bank1_rd_sync2 <= {ADDR_W{1'b0}};
        end else begin
            done0_sync1_rd   <= frame_done_tog_bank0_wr;
            done0_sync2_rd   <= done0_sync1_rd;
            done0_sync2_d_rd <= done0_sync2_rd;
            done1_sync1_rd   <= frame_done_tog_bank1_wr;
            done1_sync2_rd   <= done1_sync1_rd;
            done1_sync2_d_rd <= done1_sync2_rd;
            frame_start_addr_bank0_rd_sync1 <= frame_start_addr_bank0_wr;
            frame_start_addr_bank0_rd_sync2 <= frame_start_addr_bank0_rd_sync1;
            frame_start_addr_bank1_rd_sync1 <= frame_start_addr_bank1_wr;
            frame_start_addr_bank1_rd_sync2 <= frame_start_addr_bank1_rd_sync1;
        end
    end

    // read domain commit state
    reg pending_valid_rd;
    reg pending_bank_rd;
    reg [ADDR_W-1:0] pending_frame_start_addr_rd;

    always @(posedge clk_read or negedge rst_n_read) begin
        if (!rst_n_read) begin
            pending_valid_rd            <= 1'b0;
            pending_bank_rd             <= 1'b0;
            pending_frame_start_addr_rd <= {ADDR_W{1'b0}};
            active_bank_rd              <= 1'b0;
            frame_valid                 <= 1'b0;
            active_frame_start_addr     <= {ADDR_W{1'b0}};
            commit_ack_tog_rd           <= 1'b0;
        end else begin
            // Keep only the newest completed frame.
            if (done0_pulse_rd) begin
                pending_valid_rd            <= 1'b1;
                pending_bank_rd             <= 1'b0;
                pending_frame_start_addr_rd <= frame_start_addr_bank0_rd_sync2;
            end
            if (done1_pulse_rd) begin
                pending_valid_rd            <= 1'b1;
                pending_bank_rd             <= 1'b1;
                pending_frame_start_addr_rd <= frame_start_addr_bank1_rd_sync2;
            end

            // First frame commits immediately; subsequent commits on frame boundary.
            if (!frame_valid && pending_valid_rd) begin
                active_bank_rd          <= pending_bank_rd;
                active_frame_start_addr <= pending_frame_start_addr_rd;
                frame_valid             <= 1'b1;
                pending_valid_rd        <= 1'b0;
                commit_ack_tog_rd       <= ~commit_ack_tog_rd;
            end else if (rd_frame_done && pending_valid_rd) begin
                active_bank_rd          <= pending_bank_rd;
                active_frame_start_addr <= pending_frame_start_addr_rd;
                frame_valid             <= 1'b1;
                pending_valid_rd        <= 1'b0;
                commit_ack_tog_rd       <= ~commit_ack_tog_rd;
            end
        end
    end

    // read data mux
    always @(posedge clk_read or negedge rst_n_read) begin
        if (!rst_n_read) begin
            rdata_ch1 <= 8'd127;
            rdata_ch2 <= 8'd127;
            rdata_ch3 <= 8'd127;
            rdata_ch4 <= 8'd127;
        end else if (!frame_valid) begin
            rdata_ch1 <= 8'd127;
            rdata_ch2 <= 8'd127;
            rdata_ch3 <= 8'd127;
            rdata_ch4 <= 8'd127;
        end else if (active_bank_rd == 1'b0) begin
            rdata_ch1 <= bank0_rdata_b[7:0];
            rdata_ch2 <= bank0_rdata_b[15:8];
            rdata_ch3 <= bank0_rdata_b[23:16];
            rdata_ch4 <= bank0_rdata_b[31:24];
        end else begin
            rdata_ch1 <= bank1_rdata_b[7:0];
            rdata_ch2 <= bank1_rdata_b[15:8];
            rdata_ch3 <= bank1_rdata_b[23:16];
            rdata_ch4 <= bank1_rdata_b[31:24];
        end
    end

endmodule
