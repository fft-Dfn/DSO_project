// -----------------------------------------------------------------------------
// pingpong_buffer
// -----------------------------------------------------------------------------
// Architecture:
// - Two-bank frame buffer with write domain at 50 MHz and read domain at 25 MHz.
// - Write side commits completed frames; read side only switches active bank at frame boundary.
//
// State/handshake intent:
// - capture_active_wr/capture_bank_wr lock one target bank during an ongoing capture burst.
// - pending_bank_rd stores newest committed bank until rd_frame_done allows atomic switch.
// - commit_inflight_wr blocks rearm until read side consumes the committed frame, preventing
//   pending-bank overwrite before display handoff.
// -----------------------------------------------------------------------------

module pingpong_buffer #(
    parameter DATA_W = 8,
    parameter ADDR_W = 10,
    parameter USE_INFERRED_BRAM = 1'b1
)(
    input  wire                  rst_n_write,
    input  wire                  rst_n_read,

    input  wire                  clk_write,
    input  wire [ADDR_W-1:0]     waddr,
    input  wire [DATA_W-1:0]     wdata_ch1,
    input  wire [DATA_W-1:0]     wdata_ch2,
    input  wire [DATA_W-1:0]     wdata_ch3,
    input  wire [DATA_W-1:0]     wdata_ch4,

    input  wire                  clk_read,
    input  wire [ADDR_W-1:0]     raddr,
    output reg  [DATA_W-1:0]     rdata_ch1,
    output reg  [DATA_W-1:0]     rdata_ch2,
    output reg  [DATA_W-1:0]     rdata_ch3,
    output reg  [DATA_W-1:0]     rdata_ch4,

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

    // Idle write-bank preference (used only when a capture burst is not locked yet).
    reg  wr_bank_sel;
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

    // Write-domain capture lock:
    // - capture_active_wr: a capture burst is in progress
    // - capture_bank_wr  : bank fixed for this burst from first write beat to capture_done
    reg capture_active_wr;
    reg capture_bank_wr;
    reg active_bank_rd;
    reg active_bank_sync1_wr, active_bank_sync2_wr;
    reg frame_valid_sync1_wr, frame_valid_sync2_wr;
    // A committed frame exists but has not been consumed by read domain yet.
    reg commit_inflight_wr;

    reg bank0_commit_tog_wr, bank1_commit_tog_wr;
    reg bank0_commit_req_wr, bank1_commit_req_wr;
    reg [ADDR_W-1:0] bank0_start_addr_wr, bank1_start_addr_wr;
    reg dbg_we_seen_wr, dbg_capdone_seen_wr, dbg_write_active_seen_wr, dbg_overrun_seen_wr;
    reg dbg_bank0_we_seen_wr, dbg_bank1_we_seen_wr;
    // Read->write acknowledgment toggle synchronizer.
    reg consume_tog_sync1_wr, consume_tog_sync2_wr, consume_tog_d_wr;
    reg consume_tog_rd;
    wire consume_pulse_wr = consume_tog_sync2_wr ^ consume_tog_d_wr;

    wire write_bank_sel = capture_active_wr ? capture_bank_wr : wr_bank_sel;
    wire safe_idle_bank_sel = frame_valid_sync2_wr ? ~active_bank_sync2_wr : wr_bank_sel;
    wire writing_active_bank = frame_valid_sync2_wr && (write_bank_sel == active_bank_sync2_wr);
    assign bank0_we_a = we && (write_bank_sel == 1'b0) && ~writing_active_bank;
    assign bank1_we_a = we && (write_bank_sel == 1'b1) && ~writing_active_bank;

    // Rearm is allowed only when:
    // 1) current capture burst is fully finished, and
    // 2) the last committed frame has already been consumed by display side.
    assign capture_ready = !capture_active_wr && !commit_inflight_wr;

    // Write-domain control FSM (capture lock + commit publication).
    always @(posedge clk_write or negedge rst_n_write) begin
        if (!rst_n_write) begin
            wr_bank_sel               <= 1'b1;
            capture_active_wr         <= 1'b0;
            capture_bank_wr           <= 1'b1;
            active_bank_sync1_wr      <= 1'b0;
            active_bank_sync2_wr      <= 1'b0;
            frame_valid_sync1_wr      <= 1'b0;
            frame_valid_sync2_wr      <= 1'b0;
            commit_inflight_wr        <= 1'b0;
            bank0_commit_tog_wr       <= 1'b0;
            bank1_commit_tog_wr       <= 1'b0;
            bank0_commit_req_wr       <= 1'b0;
            bank1_commit_req_wr       <= 1'b0;
            bank0_start_addr_wr       <= {ADDR_W{1'b0}};
            bank1_start_addr_wr       <= {ADDR_W{1'b0}};
            consume_tog_sync1_wr      <= 1'b0;
            consume_tog_sync2_wr      <= 1'b0;
            consume_tog_d_wr          <= 1'b0;
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

            active_bank_sync1_wr <= active_bank_rd;
            active_bank_sync2_wr <= active_bank_sync1_wr;
            frame_valid_sync1_wr <= frame_valid;
            frame_valid_sync2_wr <= frame_valid_sync1_wr;
            consume_tog_sync1_wr <= consume_tog_rd;
            consume_tog_sync2_wr <= consume_tog_sync1_wr;
            consume_tog_d_wr     <= consume_tog_sync2_wr;

            // Read domain toggles consume_tog_rd when it accepts pending frame.
            if (consume_pulse_wr)
                commit_inflight_wr <= 1'b0;

            if (bank0_commit_req_wr) begin
                bank0_commit_tog_wr <= ~bank0_commit_tog_wr;
                bank0_commit_req_wr <= 1'b0;
            end
            if (bank1_commit_req_wr) begin
                bank1_commit_tog_wr <= ~bank1_commit_tog_wr;
                bank1_commit_req_wr <= 1'b0;
            end

            if (!capture_active_wr && frame_valid_sync2_wr)
                wr_bank_sel <= ~active_bank_sync2_wr;

            if (capture_done) begin
                capture_active_wr <= 1'b0;
            end else if (!capture_active_wr && we) begin
                // Lock target bank on first write of a new capture burst.
                capture_active_wr <= 1'b1;
                capture_bank_wr   <= safe_idle_bank_sel;
            end

            if (capture_done) begin
                if (frame_valid_sync2_wr && (capture_bank_wr == active_bank_sync2_wr)) begin

                    overflow <= 1'b1;
                    dbg_overrun_seen_wr <= 1'b1;
                end else begin
                    // Publish a new frame commit into read domain via toggle pulse.
                    if (capture_bank_wr == 1'b0) begin
                        bank0_start_addr_wr       <= frame_start_addr;
                        bank0_commit_req_wr       <= 1'b1;
                    end else begin
                        bank1_start_addr_wr       <= frame_start_addr;
                        bank1_commit_req_wr       <= 1'b1;
                    end
                    commit_inflight_wr <= 1'b1;
                end

                if (frame_valid_sync2_wr)
                    wr_bank_sel <= ~active_bank_sync2_wr;
            end
        end
    end

    // Read-domain control FSM (pending queue + atomic frame-boundary bank switch).
    reg bank0_commit_sync1_rd, bank0_commit_sync2_rd, bank0_commit_sync2_d_rd;
    reg bank1_commit_sync1_rd, bank1_commit_sync2_rd, bank1_commit_sync2_d_rd;
    wire bank0_commit_pulse_rd = bank0_commit_sync2_rd ^ bank0_commit_sync2_d_rd;
    wire bank1_commit_pulse_rd = bank1_commit_sync2_rd ^ bank1_commit_sync2_d_rd;

    reg [ADDR_W-1:0] bank0_start_sync1_rd, bank0_start_sync2_rd;
    reg [ADDR_W-1:0] bank1_start_sync1_rd, bank1_start_sync2_rd;

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
            bank0_start_sync1_rd          <= {ADDR_W{1'b0}};
            bank0_start_sync2_rd          <= {ADDR_W{1'b0}};
            bank1_start_sync1_rd          <= {ADDR_W{1'b0}};
            bank1_start_sync2_rd          <= {ADDR_W{1'b0}};
            active_bank_rd                <= 1'b0;
            pending_valid_rd              <= 1'b0;
            pending_bank_rd               <= 1'b0;
            pending_start_addr_rd         <= {ADDR_W{1'b0}};
            frame_valid                   <= 1'b0;
            active_frame_start_addr       <= {ADDR_W{1'b0}};
            dbg_rd_switch_seen            <= 1'b0;
            dbg_commit_seen_rd            <= 1'b0;
            consume_tog_rd                <= 1'b0;
        end else begin
            bank0_commit_sync1_rd   <= bank0_commit_tog_wr;
            bank0_commit_sync2_rd   <= bank0_commit_sync1_rd;
            bank0_commit_sync2_d_rd <= bank0_commit_sync2_rd;
            bank1_commit_sync1_rd   <= bank1_commit_tog_wr;
            bank1_commit_sync2_rd   <= bank1_commit_sync1_rd;
            bank1_commit_sync2_d_rd <= bank1_commit_sync2_rd;
            bank0_start_sync1_rd    <= bank0_start_addr_wr;
            bank0_start_sync2_rd    <= bank0_start_sync1_rd;
            bank1_start_sync1_rd    <= bank1_start_addr_wr;
            bank1_start_sync2_rd    <= bank1_start_sync1_rd;

            if (bank0_commit_pulse_rd) begin
                pending_valid_rd      <= 1'b1;
                pending_bank_rd       <= 1'b0;
                pending_start_addr_rd <= bank0_start_sync2_rd;
                dbg_commit_seen_rd    <= 1'b1;
            end
            if (bank1_commit_pulse_rd) begin
                pending_valid_rd      <= 1'b1;
                pending_bank_rd       <= 1'b1;
                pending_start_addr_rd <= bank1_start_sync2_rd;
                dbg_commit_seen_rd    <= 1'b1;
            end

            if (!frame_valid && pending_valid_rd) begin
                // First valid frame after reset can switch immediately.
                active_bank_rd          <= pending_bank_rd;
                active_frame_start_addr <= pending_start_addr_rd;
                pending_valid_rd        <= 1'b0;
                frame_valid             <= 1'b1;
                dbg_rd_switch_seen      <= 1'b1;
                consume_tog_rd          <= ~consume_tog_rd;
            end else if (rd_frame_done && pending_valid_rd) begin
                // Normal path: only switch display source at frame boundary.
                active_bank_rd          <= pending_bank_rd;
                active_frame_start_addr <= pending_start_addr_rd;
                pending_valid_rd        <= 1'b0;
                frame_valid             <= 1'b1;
                dbg_rd_switch_seen      <= 1'b1;
                consume_tog_rd          <= ~consume_tog_rd;
            end
        end
    end

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

    wire bank0_raw_nz = |bank0_rdata_b;
    wire bank1_raw_nz = |bank1_rdata_b;
    wire active_raw_nz = active_bank_rd ? bank1_raw_nz : bank0_raw_nz;
    wire muxed_reg_nz = |{rdata_ch4, rdata_ch3, rdata_ch2, rdata_ch1};
    assign dbg_read_tap = {muxed_reg_nz, active_raw_nz, bank1_raw_nz, bank0_raw_nz};

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
