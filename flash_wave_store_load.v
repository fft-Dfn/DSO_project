// -----------------------------------------------------------------------------
// flash_wave_store_load
// -----------------------------------------------------------------------------
// Transaction FSM intent:
// - STORE: snapshot one selected channel from fetch stream -> trigger SPI write.
// - LOAD : trigger SPI read -> populate view buffer -> expose flash_view_valid.
// - Supports cancel/timeout/error signaling without back-pressuring VGA display path.
// -----------------------------------------------------------------------------

module flash_wave_store_load #(
    parameter integer ADDR_W = 10,
    parameter integer DATA_W = 8,
    parameter [23:0] FLASH_DATA_BASE = 24'h300000,
    parameter [23:0] FLASH_CH_STRIDE = 24'h001000,
    parameter integer TXN_TIMEOUT_CYCLES = 300_000_000
)(
    input  wire                  clk_25m,
    input  wire                  rst_n,

    input  wire                  store_req,
    input  wire                  load_req,
    input  wire                  cancel_req,
    input  wire [1:0]            store_ch_sel,
    input  wire [1:0]            load_ch_sel,

    input  wire                  fetch_sample_valid,
    input  wire [ADDR_W-1:0]     fetch_sample_idx,
    input  wire [31:0]           fetch_sample_packed,
    input  wire                  fetch_frame_done,

    input  wire [ADDR_W-1:0]     flash_view_addr,
    output reg  [DATA_W-1:0]     flash_view_sample,
    output reg                   flash_view_valid,

    output wire                  flash_busy,
    output reg                   flash_req_ack_pulse,
    output reg                   flash_done_pulse,
    output reg                   flash_cancel_pulse,
    output reg                   flash_timeout_pulse,
    output reg                   flash_error,
    output reg  [1:0]            flash_last_op_ch,
    output reg                   flash_active_is_load,
    output reg  [23:0]           flash_jedec_id,
    output reg                   flash_jedec_valid,

    output wire                  flash_cs_n,
    output wire                  flash_sck,
    output wire                  flash_mosi,
    input  wire                  flash_miso
);

    localparam integer SAMPLE_CNT = (1 << ADDR_W);

    localparam [23:0] FLASH_BASE_CH0 = FLASH_DATA_BASE;
    localparam [23:0] FLASH_BASE_CH1 = FLASH_DATA_BASE + FLASH_CH_STRIDE;
    localparam [23:0] FLASH_BASE_CH2 = FLASH_DATA_BASE + FLASH_CH_STRIDE + FLASH_CH_STRIDE;
    localparam [23:0] FLASH_BASE_CH3 = FLASH_DATA_BASE + FLASH_CH_STRIDE + FLASH_CH_STRIDE + FLASH_CH_STRIDE;

    function [23:0] flash_base_from_ch;
        input [1:0] ch;
        begin
            case (ch)
                2'd0: flash_base_from_ch = FLASH_BASE_CH0;
                2'd1: flash_base_from_ch = FLASH_BASE_CH1;
                2'd2: flash_base_from_ch = FLASH_BASE_CH2;
                default: flash_base_from_ch = FLASH_BASE_CH3;
            endcase
        end
    endfunction

    function [7:0] pick_ch_sample;
        input [31:0] packed;
        input [1:0]  ch;
        begin
            case (ch)
                2'd0: pick_ch_sample = packed[7:0];
                2'd1: pick_ch_sample = packed[15:8];
                2'd2: pick_ch_sample = packed[23:16];
                default: pick_ch_sample = packed[31:24];
            endcase
        end
    endfunction

    // frame_buf     : staging buffer for STORE write source and LOAD destination.
    // flash_view_buf: stable readback buffer used by display flash-view path.
    (* ram_style = "block" *) reg [DATA_W-1:0] frame_buf      [0:SAMPLE_CNT-1];
    (* ram_style = "block" *) reg [DATA_W-1:0] flash_view_buf [0:SAMPLE_CNT-1];

    // STORE front-end capture control.
    reg                  snap_active;
    reg                  snapshot_ready;
    reg [1:0]            snap_ch_sel;

    // SPI transaction ownership.
    reg                  op_active;
    reg                  op_is_load;

    reg                  spi_wr_req;
    reg                  spi_rd_req;
    reg                  spi_id_req;
    reg [23:0]           spi_base_addr;
    reg                  spi_soft_reset;

    reg                  snap_wr_en;
    reg [ADDR_W-1:0]     snap_wr_addr;
    reg [DATA_W-1:0]     snap_wr_data;

    wire [ADDR_W-1:0]    spi_in_data_idx;
    reg  [DATA_W-1:0]    spi_in_data;
    wire [DATA_W-1:0]    spi_out_data;
    wire                 spi_out_data_we;
    wire [ADDR_W-1:0]    spi_out_data_idx;
    wire [2:0]           spi_status;
    wire                 spi_busy;
    wire                 spi_done_pulse;
    wire                 spi_error;
    wire [23:0]          spi_jedec_id;
    wire                 spi_jedec_valid;
    reg                  jedec_probe_pending;

    // Guards against stuck SPI operation.
    reg [31:0]           txn_watchdog_cnt;

    wire                 txn_active = snap_active || snapshot_ready || op_active || spi_busy;

    wire                 watchdog_active = op_active || spi_busy;
    wire                 timeout_hit = watchdog_active && (txn_watchdog_cnt >= (TXN_TIMEOUT_CYCLES - 1));
    wire                 spi_rst_n = rst_n & ~spi_soft_reset;

    wire                 frame_buf_we   = spi_out_data_we | snap_wr_en;
    wire [ADDR_W-1:0]    frame_buf_addr = spi_out_data_we ? spi_out_data_idx : snap_wr_addr;
    wire [DATA_W-1:0]    frame_buf_data = spi_out_data_we ? spi_out_data     : snap_wr_data;
    reg  [ADDR_W-1:0]    flash_view_addr_r;

    assign flash_busy = txn_active;

    // Memory datapath process.
    always @(posedge clk_25m) begin
        if (frame_buf_we)
            frame_buf[frame_buf_addr] <= frame_buf_data;

        if (spi_out_data_we)
            flash_view_buf[spi_out_data_idx] <= spi_out_data;

        flash_view_addr_r  <= flash_view_addr;
        flash_view_sample  <= flash_view_buf[flash_view_addr_r];

        spi_in_data <= frame_buf[spi_in_data_idx];
    end

    // Transaction control FSM:
    // - accepts store/load requests when fully idle
    // - executes cancel/timeout abort paths with highest priority
    // - sequences snapshot capture and SPI request generation
    always @(posedge clk_25m or negedge rst_n) begin
        if (!rst_n) begin
            snap_active          <= 1'b0;
            snapshot_ready       <= 1'b0;
            snap_ch_sel          <= 2'd0;
            op_active            <= 1'b0;
            op_is_load           <= 1'b0;
            spi_wr_req           <= 1'b0;
            spi_rd_req           <= 1'b0;
            spi_id_req           <= 1'b0;
            spi_base_addr        <= FLASH_BASE_CH0;
            spi_soft_reset       <= 1'b0;
            snap_wr_en           <= 1'b0;
            snap_wr_addr         <= {ADDR_W{1'b0}};
            snap_wr_data         <= {DATA_W{1'b0}};
            flash_req_ack_pulse  <= 1'b0;
            flash_done_pulse     <= 1'b0;
            flash_cancel_pulse   <= 1'b0;
            flash_timeout_pulse  <= 1'b0;
            flash_error          <= 1'b0;
            flash_view_valid     <= 1'b0;
            flash_last_op_ch     <= 2'd0;
            flash_active_is_load <= 1'b0;
            flash_jedec_id       <= 24'h000000;
            flash_jedec_valid    <= 1'b0;
            jedec_probe_pending  <= 1'b1;
            txn_watchdog_cnt     <= 32'd0;
        end else begin
            spi_wr_req          <= 1'b0;
            spi_rd_req          <= 1'b0;
            spi_id_req          <= 1'b0;
            spi_soft_reset      <= 1'b0;
            snap_wr_en          <= 1'b0;
            flash_req_ack_pulse <= 1'b0;
            flash_done_pulse    <= 1'b0;
            flash_cancel_pulse  <= 1'b0;
            flash_timeout_pulse <= 1'b0;

            if (!watchdog_active)
                txn_watchdog_cnt <= 32'd0;
            else if (!timeout_hit)
                txn_watchdog_cnt <= txn_watchdog_cnt + 1'b1;

            // Priority 1: explicit cancel from UI.
            if (cancel_req && txn_active) begin
                snap_active          <= 1'b0;
                snapshot_ready       <= 1'b0;
                op_active            <= 1'b0;
                op_is_load           <= 1'b0;
                spi_soft_reset       <= spi_busy;
                flash_cancel_pulse   <= 1'b1;
                flash_error          <= 1'b0;
                flash_active_is_load <= 1'b0;
            end else if (timeout_hit) begin
                // Priority 2: watchdog timeout abort.
                snap_active          <= 1'b0;
                snapshot_ready       <= 1'b0;
                op_active            <= 1'b0;
                op_is_load           <= 1'b0;
                spi_soft_reset       <= spi_busy;
                flash_timeout_pulse  <= 1'b1;
                flash_error          <= 1'b1;
                flash_active_is_load <= 1'b0;
                txn_watchdog_cnt     <= 32'd0;
            end else begin
                if (spi_jedec_valid) begin
                    flash_jedec_id    <= spi_jedec_id;
                    flash_jedec_valid <= 1'b1;
                end

                // Optional one-shot JEDEC ID probe after reset.
                if (!txn_active && jedec_probe_pending) begin
                    spi_id_req <= 1'b1;
                    jedec_probe_pending <= 1'b0;
                end else if (!txn_active) begin
                    if (store_req) begin
                        snap_active          <= 1'b1;
                        snapshot_ready       <= 1'b0;
                        snap_ch_sel          <= store_ch_sel;
                        flash_req_ack_pulse  <= 1'b1;
                        flash_active_is_load <= 1'b0;
                        flash_error          <= 1'b0;
                    end else if (load_req) begin
                        op_active            <= 1'b1;
                        op_is_load           <= 1'b1;
                        flash_active_is_load <= 1'b1;
                        flash_last_op_ch     <= load_ch_sel;
                        flash_view_valid     <= 1'b0;
                        spi_base_addr        <= flash_base_from_ch(load_ch_sel);
                        spi_rd_req           <= 1'b1;
                        flash_req_ack_pulse  <= 1'b1;
                        flash_error          <= 1'b0;
                    end
                end

                // STORE path: capture selected channel from fetched frame stream.
                if (snap_active && fetch_sample_valid) begin
                    snap_wr_en   <= 1'b1;
                    snap_wr_addr <= fetch_sample_idx;
                    snap_wr_data <= pick_ch_sample(fetch_sample_packed, snap_ch_sel);
                end

                if (snap_active && fetch_frame_done) begin
                    snap_active    <= 1'b0;
                    snapshot_ready <= 1'b1;
                end

                // Launch STORE SPI write once full snapshot is available.
                if (snapshot_ready && !op_active && !spi_busy) begin
                    op_active            <= 1'b1;
                    op_is_load           <= 1'b0;
                    flash_active_is_load <= 1'b0;
                    flash_last_op_ch     <= snap_ch_sel;
                    spi_base_addr        <= flash_base_from_ch(snap_ch_sel);
                    spi_wr_req           <= 1'b1;
                    flash_error          <= 1'b0;
                end

                // Common completion path for both LOAD and STORE.
                if (spi_done_pulse && op_active) begin
                    op_active        <= 1'b0;
                    flash_done_pulse <= 1'b1;
                    if (op_is_load)
                        flash_view_valid <= 1'b1;
                    else
                        snapshot_ready <= 1'b0;
                end

            end
        end
    end

    spi_flash_ctrl #(
        .ADDR_DEPTH(ADDR_W),
        .DATA_DEPTH(DATA_W)
    ) u_spi_flash_ctrl (
        .clk_25m         (clk_25m),
        .rst_n           (spi_rst_n),
        .flash_write_req (spi_wr_req),
        .flash_read_req  (spi_rd_req),
        .flash_id_req    (spi_id_req),
        .flash_base_addr (spi_base_addr),
        .in_data         (spi_in_data),
        .in_data_idx     (spi_in_data_idx),
        .out_data        (spi_out_data),
        .out_data_we     (spi_out_data_we),
        .out_data_idx    (spi_out_data_idx),
        .flash_status    (spi_status),
        .busy            (spi_busy),
        .done_pulse      (spi_done_pulse),
        .error           (spi_error),
        .jedec_id        (spi_jedec_id),
        .jedec_id_valid  (spi_jedec_valid),
        .flash_cs_n      (flash_cs_n),
        .flash_sck       (flash_sck),
        .flash_mosi      (flash_mosi),
        .flash_miso      (flash_miso)
    );

endmodule
