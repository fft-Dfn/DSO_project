// -----------------------------------------------------------------------------
// pingpong_buffer
// -----------------------------------------------------------------------------
// Purpose:
//   Two-bank frame buffer crossing write (50 MHz) and read (25 MHz) domains.
//   Guarantees:
//   1) Display never switches bank in middle of a frame.
//   2) Write side avoids writing active display bank.
//   3) New frame commit is atomic (bank locked during one capture burst).
//
// Key Signals (English):
//   - wr_bank_sel: next target bank when write side is idle.
//   - capture_bank_wr: bank locked for current capture burst.
//   - active_bank_rd: bank currently selected by VGA read side.
//   - pending_bank_rd: latest committed bank waiting next frame boundary.
//   - writing_active_bank: illegal condition detector (write targeting active bank).
// -----------------------------------------------------------------------------
module pingpong_buffer #(
    parameter DATA_W = 8,
    parameter ADDR_W = 10,
    parameter USE_INFERRED_BRAM = 1'b1
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
    output wire                  capture_ready,
    output wire [15:0]           dbg_bus,
    output wire [3:0]            dbg_read_tap
);

    wire [31:0] wdata_packed = {wdata_ch4, wdata_ch3, wdata_ch2, wdata_ch1};

    // -------------------------------------------------------------------------
    // Two-bank BRAM
    // -------------------------------------------------------------------------
    reg  wr_bank_sel; // 0: bank0, 1: bank1 (next capture target when idle)
    wire bank0_we_a;
    wire bank1_we_a;

    wire [31:0] bank0_rdata_b;
    wire [31:0] bank1_rdata_b;
    wire [31:0] bank0_rdata_a_unused;
    wire [31:0] bank1_rdata_a_unused;

    generate
        if (USE_INFERRED_BRAM) begin : g_inferred_bram
            reg [31:0] bank0_mem [0:(1<<ADDR_W)-1];
            reg [31:0] bank1_mem [0:(1<<ADDR_W)-1];
            reg [31:0] bank0_rdata_b_r;
            reg [31:0] bank1_rdata_b_r;

            assign bank0_rdata_a_unused = 32'd0;
            assign bank1_rdata_a_unused = 32'd0;
            assign bank0_rdata_b = bank0_rdata_b_r;
            assign bank1_rdata_b = bank1_rdata_b_r;

            always @(posedge clk_write) begin
                if (bank0_we_a)
                    bank0_mem[waddr] <= wdata_packed;
                if (bank1_we_a)
                    bank1_mem[waddr] <= wdata_packed;
            end

            // Model as synchronous read (1-cycle latency) to match stream prefetch pipeline.
            always @(posedge clk_read) begin
                bank0_rdata_b_r <= bank0_mem[raddr];
                bank1_rdata_b_r <= bank1_mem[raddr];
            end
        end else begin : g_ip_bram
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
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Write side: never write active display bank
    // -------------------------------------------------------------------------
    reg capture_active_wr;
    reg capture_bank_wr; // bank locked for current capture burst
    reg active_bank_rd;
    reg active_bank_sync1_wr, active_bank_sync2_wr;
    reg frame_valid_sync1_wr, frame_valid_sync2_wr;

    reg bank0_commit_tog_wr, bank1_commit_tog_wr;
    reg dbg_we_seen_wr, dbg_capdone_seen_wr, dbg_write_active_seen_wr, dbg_overrun_seen_wr;
    reg dbg_bank0_we_seen_wr, dbg_bank1_we_seen_wr;

    wire write_bank_sel = capture_active_wr ? capture_bank_wr : wr_bank_sel;
    wire safe_idle_bank_sel = frame_valid_sync2_wr ? ~active_bank_sync2_wr : wr_bank_sel;
    wire writing_active_bank = frame_valid_sync2_wr && (write_bank_sel == active_bank_sync2_wr);
    assign bank0_we_a = we && (write_bank_sel == 1'b0) && ~writing_active_bank;
    assign bank1_we_a = we && (write_bank_sel == 1'b1) && ~writing_active_bank;
    // Keep sample_controller rearm path robust:
    // ready only depends on whether a capture burst is currently in progress.
    // Bank safety is enforced by write-bank lock + write gating, not by stalling rearm.
    assign capture_ready = !capture_active_wr;

    always @(posedge clk_write or negedge rst_n_write) begin
        if (!rst_n_write) begin
            wr_bank_sel               <= 1'b1; // start from bank1 to avoid reset default active=bank0
            capture_active_wr         <= 1'b0;
            capture_bank_wr           <= 1'b1;
            active_bank_sync1_wr      <= 1'b0;
            active_bank_sync2_wr      <= 1'b0;
            frame_valid_sync1_wr      <= 1'b0;
            frame_valid_sync2_wr      <= 1'b0;
            bank0_commit_tog_wr       <= 1'b0;
            bank1_commit_tog_wr       <= 1'b0;
            dbg_we_seen_wr            <= 1'b0;
            dbg_capdone_seen_wr       <= 1'b0;
            dbg_write_active_seen_wr  <= 1'b0;
            dbg_overrun_seen_wr       <= 1'b0;
            dbg_bank0_we_seen_wr      <= 1'b0;
            dbg_bank1_we_seen_wr      <= 1'b0;
            overflow                  <= 1'b0;
        end else begin
            overflow <= 1'b0;
            if (we)
                dbg_we_seen_wr <= 1'b1;
            if (bank0_we_a)
                dbg_bank0_we_seen_wr <= 1'b1;
            if (bank1_we_a)
                dbg_bank1_we_seen_wr <= 1'b1;
            if (capture_done)
                dbg_capdone_seen_wr <= 1'b1;
            if (writing_active_bank)
                dbg_write_active_seen_wr <= 1'b1;

            // Sync read-side active bank and frame_valid.
            active_bank_sync1_wr <= active_bank_rd;
            active_bank_sync2_wr <= active_bank_sync1_wr;
            frame_valid_sync1_wr <= frame_valid;
            frame_valid_sync2_wr <= frame_valid_sync1_wr;

            // Idle steering: always hold next target at non-active bank.
            if (!capture_active_wr && frame_valid_sync2_wr)
                wr_bank_sel <= ~active_bank_sync2_wr;

            // Lock target bank on first write beat of a capture burst.
            if (capture_done) begin
                capture_active_wr <= 1'b0;
            end else if (!capture_active_wr && we) begin
                capture_active_wr <= 1'b1;
                capture_bank_wr   <= safe_idle_bank_sel;
            end

            if (capture_done) begin
                if (frame_valid_sync2_wr && (capture_bank_wr == active_bank_sync2_wr)) begin
                    // Overrun: a capture ended on active display bank, drop this frame.
                    overflow <= 1'b1;
                    dbg_overrun_seen_wr <= 1'b1;
                end else begin
                    if (capture_bank_wr == 1'b0) begin
                        bank0_commit_tog_wr       <= ~bank0_commit_tog_wr;
                    end else begin
                        bank1_commit_tog_wr       <= ~bank1_commit_tog_wr;
                    end
                end

                // Keep idle target steered by active bank; do not force blind toggle.
                if (frame_valid_sync2_wr)
                    wr_bank_sel <= ~active_bank_sync2_wr;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Read side: consume latest committed frame at frame boundary
    // -------------------------------------------------------------------------
    reg bank0_commit_sync1_rd, bank0_commit_sync2_rd, bank0_commit_sync2_d_rd;
    reg bank1_commit_sync1_rd, bank1_commit_sync2_rd, bank1_commit_sync2_d_rd;
    wire bank0_commit_pulse_rd = bank0_commit_sync2_rd ^ bank0_commit_sync2_d_rd;
    wire bank1_commit_pulse_rd = bank1_commit_sync2_rd ^ bank1_commit_sync2_d_rd;

    reg pending_valid_rd;
    reg pending_bank_rd;
    reg [ADDR_W-1:0] pending_start_addr_rd;
    reg dbg_rd_switch_seen, dbg_commit_seen_rd;

    always @(posedge clk_read or negedge rst_n_read) begin
        if (!rst_n_read) begin
            bank0_commit_sync1_rd         <= 1'b0;
            bank0_commit_sync2_rd         <= 1'b0;
            bank0_commit_sync2_d_rd       <= 1'b0;
            bank1_commit_sync1_rd         <= 1'b0;
            bank1_commit_sync2_rd         <= 1'b0;
            bank1_commit_sync2_d_rd       <= 1'b0;
            active_bank_rd                <= 1'b0;
            pending_valid_rd              <= 1'b0;
            pending_bank_rd               <= 1'b0;
            pending_start_addr_rd         <= {ADDR_W{1'b0}};
            frame_valid                   <= 1'b0;
            active_frame_start_addr       <= {ADDR_W{1'b0}};
            dbg_rd_switch_seen            <= 1'b0;
            dbg_commit_seen_rd            <= 1'b0;
        end else begin
            bank0_commit_sync1_rd   <= bank0_commit_tog_wr;
            bank0_commit_sync2_rd   <= bank0_commit_sync1_rd;
            bank0_commit_sync2_d_rd <= bank0_commit_sync2_rd;
            bank1_commit_sync1_rd   <= bank1_commit_tog_wr;
            bank1_commit_sync2_rd   <= bank1_commit_sync1_rd;
            bank1_commit_sync2_d_rd <= bank1_commit_sync2_rd;

            // Keep only the newest completed frame pending.
            if (bank0_commit_pulse_rd) begin
                pending_valid_rd      <= 1'b1;
                pending_bank_rd       <= 1'b0;
                pending_start_addr_rd <= {ADDR_W{1'b0}};
                dbg_commit_seen_rd    <= 1'b1;
            end
            if (bank1_commit_pulse_rd) begin
                pending_valid_rd      <= 1'b1;
                pending_bank_rd       <= 1'b1;
                pending_start_addr_rd <= {ADDR_W{1'b0}};
                dbg_commit_seen_rd    <= 1'b1;
            end

            if (!frame_valid && pending_valid_rd) begin
                // First frame starts displaying immediately.
                active_bank_rd          <= pending_bank_rd;
                active_frame_start_addr <= pending_start_addr_rd;
                pending_valid_rd        <= 1'b0;
                frame_valid             <= 1'b1;
                dbg_rd_switch_seen      <= 1'b1;
            end else if (rd_frame_done && pending_valid_rd) begin
                // Switch only at display frame boundary.
                active_bank_rd          <= pending_bank_rd;
                active_frame_start_addr <= pending_start_addr_rd;
                pending_valid_rd        <= 1'b0;
                frame_valid             <= 1'b1;
                dbg_rd_switch_seen      <= 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Read data mux
    // -------------------------------------------------------------------------
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

    // Read-path taps (25m read domain, direct visibility):
    // [0] bank0 raw read data has any non-zero bit
    // [1] bank1 raw read data has any non-zero bit
    // [2] currently selected active bank raw read data non-zero
    // [3] registered muxed rdata_ch* output non-zero
    wire bank0_raw_nz = |bank0_rdata_b;
    wire bank1_raw_nz = |bank1_rdata_b;
    wire active_raw_nz = active_bank_rd ? bank1_raw_nz : bank0_raw_nz;
    wire muxed_reg_nz = |{rdata_ch4, rdata_ch3, rdata_ch2, rdata_ch1};
    assign dbg_read_tap = {muxed_reg_nz, active_raw_nz, bank1_raw_nz, bank0_raw_nz};

    // [0] wr_bank_sel
    // [1] active_bank_sync2_wr
    // [2] writing_active_bank
    // [3] capture_active_wr
    // [4] bank0_commit_tog_wr
    // [5] bank1_commit_tog_wr
    // [6] active_bank_rd
    // [7] pending_valid_rd
    // [8] pending_bank_rd
    // [9] frame_valid
    // [10] we_seen_wr (sticky)
    // [11] capture_done_seen_wr (sticky)
    // [12] wrote_active_bank_seen_wr (sticky)
    // [13] overrun_seen_wr (sticky)
    // [14] bank0_we_a_seen_wr (sticky, real BRAM write hit)
    // [15] bank1_we_a_seen_wr (sticky, real BRAM write hit)
    assign dbg_bus = {
        dbg_bank1_we_seen_wr,
        dbg_bank0_we_seen_wr,
        dbg_overrun_seen_wr,
        dbg_write_active_seen_wr,
        dbg_capdone_seen_wr,
        dbg_we_seen_wr,
        frame_valid,
        pending_bank_rd,
        pending_valid_rd,
        active_bank_rd,
        bank1_commit_tog_wr,
        bank0_commit_tog_wr,
        capture_active_wr,
        writing_active_bank,
        active_bank_sync2_wr,
        wr_bank_sel
    };

endmodule
