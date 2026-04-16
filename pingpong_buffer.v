// -----------------------------------------------------------------------------
// pingpong_buffer
// -----------------------------------------------------------------------------
// Architecture:
// - Two-bank frame buffer with write domain at 50 MHz and read domain at 25 MHz.
// - Write side captures completed frame descriptors into Efinity async FIFO.
// - Read side pops one descriptor only at frame boundary and switches bank atomically.
//
// State/handshake intent:
// - capture_active_wr/capture_bank_wr lock one target bank during a capture burst.
// - commit_inflight_wr blocks rearm until read side consumes one committed descriptor.
// - Descriptor FIFO format (2x16b per frame, merged to 1x32b at read):
//   word0(main)   = {2'b10, bank[0], frame_start_addr[9:0], 3'b101}
//   word1(shadow) = {2'b01, bank[0], frame_start_addr[9:0], 3'b010}
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

    // Write-domain capture lock.
    reg capture_active_wr;
    reg capture_bank_wr;

    // Read-side active bank sampled back into write domain.
    reg active_bank_rd;
    reg active_bank_sync1_wr, active_bank_sync2_wr;
    reg frame_valid_sync1_wr, frame_valid_sync2_wr;

    // One frame descriptor is committed but not yet consumed by read domain.
    reg commit_inflight_wr;

    reg dbg_we_seen_wr, dbg_capdone_seen_wr, dbg_write_active_seen_wr, dbg_overrun_seen_wr;
    reg dbg_bank0_we_seen_wr, dbg_bank1_we_seen_wr;
    reg dbg_fifo_full_seen_wr, dbg_fifo_overflow_seen_wr;

    reg dbg_rd_switch_seen, dbg_bad_desc_seen_rd, dbg_fifo_underflow_seen_rd;

    // Read->write consume acknowledgment toggle synchronizer.
    reg consume_tog_sync1_wr, consume_tog_sync2_wr, consume_tog_d_wr;
    reg consume_tog_rd;
    wire consume_pulse_wr = consume_tog_sync2_wr ^ consume_tog_d_wr;

    // Frame descriptor FIFO signals.
    reg         desc_fifo_wr_en;
    reg [15:0]  desc_fifo_wdata;
    reg         desc_fifo_rd_en;
    wire [31:0] desc_fifo_rdata;
    wire        desc_fifo_full;
    wire        desc_fifo_prog_full;
    wire        desc_fifo_empty;
    wire        desc_fifo_rst_busy;
    wire [9:0]  desc_fifo_wr_count;
    wire [8:0]  desc_fifo_rd_count;
    wire        desc_fifo_underflow;
    wire        desc_fifo_overflow;
    wire        desc_fifo_arst = ~(rst_n_write & rst_n_read);

    // Efinity FIFO IP: async clock, 16-bit write / 32-bit read (1:2).
    efx_fifo_0 u_frame_desc_fifo (
        .prog_full_o     (desc_fifo_prog_full),
        .full_o          (desc_fifo_full),
        .empty_o         (desc_fifo_empty),
        .wr_clk_i        (clk_write),
        .rd_clk_i        (clk_read),
        .wr_en_i         (desc_fifo_wr_en),
        .rd_en_i         (desc_fifo_rd_en),
        .wdata           (desc_fifo_wdata),
        .rst_busy        (desc_fifo_rst_busy),
        .rdata           (desc_fifo_rdata),
        .a_rst_i         (desc_fifo_arst),
        .wr_datacount_o  (desc_fifo_wr_count),
        .rd_datacount_o  (desc_fifo_rd_count),
        .underflow_o     (desc_fifo_underflow),
        .overflow_o      (desc_fifo_overflow)
    );

    localparam PUSH_IDLE = 2'd0;
    localparam PUSH_W0   = 2'd1;
    localparam PUSH_W1   = 2'd2;
    reg [1:0]  desc_push_state_wr;
    reg [15:0] desc_word0_wr;
    reg [15:0] desc_word1_wr;

    localparam [1:0] POP_WAIT_CYCLES = 2'd2;
    reg        pop_active_rd;
    reg [1:0]  pop_wait_rd;

    wire write_bank_sel = capture_active_wr ? capture_bank_wr : wr_bank_sel;
    wire safe_idle_bank_sel = frame_valid_sync2_wr ? ~active_bank_sync2_wr : wr_bank_sel;
    wire writing_active_bank = frame_valid_sync2_wr && (write_bank_sel == active_bank_sync2_wr);
    assign bank0_we_a = we && (write_bank_sel == 1'b0) && ~writing_active_bank;
    assign bank1_we_a = we && (write_bank_sel == 1'b1) && ~writing_active_bank;

    // Rearm only when capture burst ended and committed descriptor was consumed.
    assign capture_ready = !capture_active_wr &&
                           !commit_inflight_wr &&
                           (desc_push_state_wr == PUSH_IDLE);

    // Write-domain control FSM:
    // 1) lock capture bank
    // 2) on capture_done, queue frame descriptor into FIFO
    // 3) wait consume ack from read side
    always @(posedge clk_write or negedge rst_n_write) begin
        if (!rst_n_write) begin
            desc_fifo_wr_en            <= 1'b0;
            desc_fifo_wdata            <= 16'd0;

            wr_bank_sel               <= 1'b1;
            capture_active_wr         <= 1'b0;
            capture_bank_wr           <= 1'b1;
            active_bank_sync1_wr      <= 1'b0;
            active_bank_sync2_wr      <= 1'b0;
            frame_valid_sync1_wr      <= 1'b0;
            frame_valid_sync2_wr      <= 1'b0;
            commit_inflight_wr        <= 1'b0;
            consume_tog_sync1_wr      <= 1'b0;
            consume_tog_sync2_wr      <= 1'b0;
            consume_tog_d_wr          <= 1'b0;
            desc_push_state_wr        <= PUSH_IDLE;
            desc_word0_wr             <= 16'd0;
            desc_word1_wr             <= 16'd0;
            dbg_we_seen_wr            <= 1'b0;
            dbg_capdone_seen_wr       <= 1'b0;
            dbg_write_active_seen_wr  <= 1'b0;
            dbg_overrun_seen_wr       <= 1'b0;
            dbg_bank0_we_seen_wr      <= 1'b0;
            dbg_bank1_we_seen_wr      <= 1'b0;
            dbg_fifo_full_seen_wr     <= 1'b0;
            dbg_fifo_overflow_seen_wr <= 1'b0;
            overflow                  <= 1'b0;
        end else begin
            desc_fifo_wr_en <= 1'b0;
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
            if (desc_fifo_full)
                dbg_fifo_full_seen_wr <= 1'b1;
            if (desc_fifo_overflow)
                dbg_fifo_overflow_seen_wr <= 1'b1;

            active_bank_sync1_wr <= active_bank_rd;
            active_bank_sync2_wr <= active_bank_sync1_wr;
            frame_valid_sync1_wr <= frame_valid;
            frame_valid_sync2_wr <= frame_valid_sync1_wr;
            consume_tog_sync1_wr <= consume_tog_rd;
            consume_tog_sync2_wr <= consume_tog_sync1_wr;
            consume_tog_d_wr     <= consume_tog_sync2_wr;

            // Read side consumed one committed descriptor.
            if (consume_pulse_wr)
                commit_inflight_wr <= 1'b0;

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
                end else if (!commit_inflight_wr && (desc_push_state_wr == PUSH_IDLE)) begin
                    // Main + shadow descriptor words (16b each) to build one 32b read record.
                    desc_word0_wr      <= {2'b10, capture_bank_wr, frame_start_addr[9:0], 3'b101};
                    desc_word1_wr      <= {2'b01, capture_bank_wr, frame_start_addr[9:0], 3'b010};
                    desc_push_state_wr <= PUSH_W0;
                    commit_inflight_wr <= 1'b1;
                end else begin
                    // Should not happen if capture_ready gating is respected.
                    overflow <= 1'b1;
                    dbg_overrun_seen_wr <= 1'b1;
                end

                if (frame_valid_sync2_wr)
                    wr_bank_sel <= ~active_bank_sync2_wr;
            end

            case (desc_push_state_wr)
                PUSH_IDLE: begin
                end

                PUSH_W0: begin
                    // Use prog_full to guarantee there is room for 2x16b words.
                    if (!desc_fifo_prog_full && !desc_fifo_rst_busy) begin
                        desc_fifo_wr_en      <= 1'b1;
                        desc_fifo_wdata      <= desc_word0_wr;
                        desc_push_state_wr   <= PUSH_W1;
                    end
                end

                PUSH_W1: begin
                    if (!desc_fifo_full && !desc_fifo_rst_busy) begin
                        desc_fifo_wr_en      <= 1'b1;
                        desc_fifo_wdata      <= desc_word1_wr;
                        desc_push_state_wr   <= PUSH_IDLE;
                    end
                end

                default: begin
                    desc_push_state_wr <= PUSH_IDLE;
                end
            endcase
        end
    end

    // Read-domain control FSM:
    // Pop one frame descriptor from FIFO at frame boundary and switch atomically.
    always @(posedge clk_read or negedge rst_n_read) begin
        if (!rst_n_read) begin
            desc_fifo_rd_en               <= 1'b0;
            active_bank_rd                <= 1'b0;
            pop_active_rd                 <= 1'b0;
            pop_wait_rd                   <= 2'd0;
            frame_valid                   <= 1'b0;
            active_frame_start_addr       <= {ADDR_W{1'b0}};
            dbg_rd_switch_seen            <= 1'b0;
            dbg_bad_desc_seen_rd          <= 1'b0;
            dbg_fifo_underflow_seen_rd    <= 1'b0;
            consume_tog_rd                <= 1'b0;
        end else begin
            desc_fifo_rd_en <= 1'b0;

            if (desc_fifo_underflow)
                dbg_fifo_underflow_seen_rd <= 1'b1;

            if (pop_active_rd) begin
                if (pop_wait_rd != 2'd0) begin
                    pop_wait_rd <= pop_wait_rd - 1'b1;
                end else begin
                    // Descriptor order can be upper/lower or lower/upper depending internal endian mapping.
                    if ((desc_fifo_rdata[31:30] == 2'b10) &&
                        (desc_fifo_rdata[15:14] == 2'b01) &&
                        (desc_fifo_rdata[18:16] == 3'b101) &&
                        (desc_fifo_rdata[2:0]   == 3'b010) &&
                        (desc_fifo_rdata[29:19] == desc_fifo_rdata[13:3])) begin
                        active_bank_rd          <= desc_fifo_rdata[29];
                        active_frame_start_addr <= desc_fifo_rdata[28:19];
                        frame_valid             <= 1'b1;
                        dbg_rd_switch_seen      <= 1'b1;
                    end else if ((desc_fifo_rdata[15:14] == 2'b10) &&
                                 (desc_fifo_rdata[31:30] == 2'b01) &&
                                 (desc_fifo_rdata[2:0]   == 3'b101) &&
                                 (desc_fifo_rdata[18:16] == 3'b010) &&
                                 (desc_fifo_rdata[13:3]  == desc_fifo_rdata[29:19])) begin
                        active_bank_rd          <= desc_fifo_rdata[13];
                        active_frame_start_addr <= desc_fifo_rdata[12:3];
                        frame_valid             <= 1'b1;
                        dbg_rd_switch_seen      <= 1'b1;
                    end else begin
                        dbg_bad_desc_seen_rd    <= 1'b1;
                    end

                    // One descriptor popped from FIFO, release write-side rearm lock.
                    consume_tog_rd  <= ~consume_tog_rd;
                    pop_active_rd   <= 1'b0;
                end
            end else if ((!frame_valid || rd_frame_done) && !desc_fifo_empty && !desc_fifo_rst_busy) begin
                desc_fifo_rd_en <= 1'b1;
                pop_active_rd   <= 1'b1;
                pop_wait_rd     <= POP_WAIT_CYCLES;
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
        dbg_fifo_overflow_seen_wr,
        dbg_fifo_underflow_seen_rd,
        dbg_bad_desc_seen_rd,
        dbg_overrun_seen_wr,
        dbg_write_active_seen_wr,
        dbg_capdone_seen_wr,
        dbg_we_seen_wr,
        frame_valid,
        desc_fifo_empty,
        desc_fifo_full,
        active_bank_rd,
        commit_inflight_wr,
        capture_active_wr,
        writing_active_bank,
        active_bank_sync2_wr,
        wr_bank_sel
    };

endmodule
