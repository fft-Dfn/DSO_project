// -----------------------------------------------------------------------------
// spi_flash_ctrl
// -----------------------------------------------------------------------------
// Command engine for SPI NOR flash transactions. It sequences command, address, data,
// busy polling, and status reporting for read/write/erase/ID operations.
// -----------------------------------------------------------------------------

module spi_flash_ctrl #(
    parameter integer ADDR_DEPTH      = 10,
    parameter integer DATA_DEPTH      = 8,
    parameter integer SPI_STEP_DIV    = 8,
    parameter integer WAIT_ERASE_CYCLES = 10_500_000,
    parameter integer WAIT_PP_CYCLES    = 100_000,
    parameter integer WAKE_WAIT_CYCLES  = 256
)(
    input  wire                    clk_25m,
    input  wire                    rst_n,

    input  wire                    flash_write_req,
    input  wire                    flash_read_req,
    input  wire                    flash_id_req,
    input  wire [23:0]             flash_base_addr,

    input  wire [DATA_DEPTH-1:0]   in_data,
    output wire [ADDR_DEPTH-1:0]   in_data_idx,
    output reg  [DATA_DEPTH-1:0]   out_data,
    output reg                     out_data_we,
    output reg  [ADDR_DEPTH-1:0]   out_data_idx,

    output reg  [2:0]              flash_status,
    output reg                     busy,
    output reg                     done_pulse,
    output reg                     error,
    output reg  [23:0]             jedec_id,
    output reg                     jedec_id_valid,

    output reg                     flash_cs_n,
    output reg                     flash_sck,
    output reg                     flash_mosi,
    input  wire                    flash_miso
);

    localparam integer FRAME_BYTES = (1 << ADDR_DEPTH);

    localparam [7:0] CMD_WREN  = 8'h06;
    localparam [7:0] CMD_RDSR1 = 8'h05;
    localparam [7:0] CMD_READ  = 8'h03;
    localparam [7:0] CMD_PP    = 8'h02;
    localparam [7:0] CMD_SE    = 8'h20;
    localparam [7:0] CMD_RDID  = 8'h9F;
    localparam [7:0] CMD_RDPD  = 8'hAB;

    // External status encoding reported to upper module.
    localparam [2:0] ST_IDLE_S   = 3'b000;
    localparam [2:0] ST_WRITE_S  = 3'b001;
    localparam [2:0] ST_READ_S   = 3'b010;
    localparam [2:0] ST_WAIT_S   = 3'b011;
    localparam [2:0] ST_DONE_S   = 3'b100;
    localparam [2:0] ST_ERR_S    = 3'b111;

    // Internal micro-states:
    // - WR group: WREN + sector erase + page program + status polling loop
    // - RD group: READ command + streaming data reception
    // - ID group: JEDEC ID read
    // - WAKE group: release from deep power-down before operation
    localparam [5:0]
        S_IDLE                 = 6'd0,

        S_WR_INIT              = 6'd1,
        S_WR_WREN_LOAD         = 6'd2,
        S_WR_WREN_SHIFT        = 6'd3,
        S_WR_WREN_END          = 6'd4,
        S_WR_SE_LOAD           = 6'd5,
        S_WR_SE_SHIFT_CMD      = 6'd6,
        S_WR_SE_SHIFT_A23      = 6'd7,
        S_WR_SE_SHIFT_A15      = 6'd8,
        S_WR_SE_SHIFT_A7       = 6'd9,
        S_WR_SE_END            = 6'd10,
        S_WR_PP_LOAD           = 6'd11,
        S_WR_PP_SHIFT_CMD      = 6'd12,
        S_WR_PP_SHIFT_A23      = 6'd13,
        S_WR_PP_SHIFT_A15      = 6'd14,
        S_WR_PP_SHIFT_A7       = 6'd15,
        S_WR_PP_REQDATA        = 6'd35,
        S_WR_PP_WAITDATA       = 6'd16,
        S_WR_PP_SHIFT_DATA     = 6'd17,
        S_WR_PP_END            = 6'd18,
        S_WR_WAIT_ERASE        = 6'd36,
        S_WR_WAIT_PP           = 6'd37,
        S_WR_POLL_LOAD         = 6'd19,
        S_WR_POLL_SHIFT_CMD    = 6'd20,
        S_WR_POLL_SHIFT_DATA   = 6'd21,
        S_WR_POLL_END          = 6'd22,
        S_WR_NEXT_PAGE         = 6'd23,
        S_WR_DONE              = 6'd24,

        S_RD_INIT              = 6'd25,
        S_RD_CMD_LOAD          = 6'd26,
        S_RD_SHIFT_CMD         = 6'd27,
        S_RD_SHIFT_A23         = 6'd28,
        S_RD_SHIFT_A15         = 6'd29,
        S_RD_SHIFT_A7          = 6'd30,
        S_RD_SHIFT_DATA        = 6'd31,
        S_RD_END               = 6'd32,
        S_RD_DONE              = 6'd33,
        S_CS_GAP               = 6'd34,

        S_ID_LOAD              = 6'd38,
        S_ID_SHIFT_CMD         = 6'd39,
        S_ID_SHIFT_B2          = 6'd40,
        S_ID_SHIFT_B1          = 6'd41,
        S_ID_SHIFT_B0          = 6'd42,
        S_ID_END               = 6'd43,

        S_WAKE_LOAD            = 6'd44,
        S_WAKE_SHIFT           = 6'd45,
        S_WAKE_END             = 6'd46,
        S_WAKE_WAIT            = 6'd47;

    // Main state and transfer bookkeeping registers.
    reg [5:0] state;
    reg [7:0] spi_step_cnt;
    wire is_shift_state;

    reg [7:0] tx_byte;
    reg [7:0] rx_byte;
    reg [2:0] bit_cnt;

    reg [23:0] cur_addr;
    reg [23:0] flash_base_addr_latched;
    reg [15:0] byte_cnt;
    reg [8:0]  page_byte_cnt;
    reg [7:0]  status_reg1;
    reg [23:0] wait_cnt;
    reg [23:0] wake_wait_cnt;
    reg [5:0]  wr_wren_next_state;
    reg [5:0]  poll_next_state;
    reg [5:0]  cs_gap_next_state;
    reg [5:0]  next_after_wake;

    reg [7:0]  wr_data_latch;
    reg [7:0]  rd_data_latch;

    assign in_data_idx = byte_cnt[ADDR_DEPTH-1:0];

    wire frame_last_byte = (byte_cnt == FRAME_BYTES-1);
    wire page_last_byte  = (page_byte_cnt == 9'd255);
    // States that consume SPI_STEP_DIV pacing and bit-level shift toggling.
    assign is_shift_state =
        (state == S_WR_WREN_SHIFT)      ||
        (state == S_WR_SE_SHIFT_CMD)    ||
        (state == S_WR_SE_SHIFT_A23)    ||
        (state == S_WR_SE_SHIFT_A15)    ||
        (state == S_WR_SE_SHIFT_A7)     ||
        (state == S_WR_PP_SHIFT_CMD)    ||
        (state == S_WR_PP_SHIFT_A23)    ||
        (state == S_WR_PP_SHIFT_A15)    ||
        (state == S_WR_PP_SHIFT_A7)     ||
        (state == S_WR_PP_SHIFT_DATA)   ||
        (state == S_WR_POLL_SHIFT_CMD)  ||
        (state == S_WR_POLL_SHIFT_DATA) ||
        (state == S_RD_SHIFT_CMD)       ||
        (state == S_RD_SHIFT_A23)       ||
        (state == S_RD_SHIFT_A15)       ||
        (state == S_RD_SHIFT_A7)        ||
        (state == S_RD_SHIFT_DATA)      ||
        (state == S_ID_SHIFT_CMD)       ||
        (state == S_ID_SHIFT_B2)        ||
        (state == S_ID_SHIFT_B1)        ||
        (state == S_ID_SHIFT_B0)        ||
        (state == S_WAKE_SHIFT);

    // Loads one TX byte and initializes bit-shift datapath.
    task spi_shift_start;
        input [7:0] din;
        begin
            tx_byte  <= din;
            rx_byte  <= 8'h00;
            bit_cnt  <= 3'd7;

            flash_mosi <= din[7];
            flash_sck <= 1'b0;
        end
    endtask

    // Main command engine FSM.
    // Single always block keeps command sequencing explicit and hardware-friendly.
    always @(posedge clk_25m or negedge rst_n) begin
        if (!rst_n) begin
            state                    <= S_IDLE;

            flash_cs_n               <= 1'b1;
            flash_sck                <= 1'b0;
            flash_mosi               <= 1'b0;

            out_data                 <= {DATA_DEPTH{1'b0}};
            out_data_we              <= 1'b0;
            out_data_idx             <= {ADDR_DEPTH{1'b0}};
            flash_status             <= ST_IDLE_S;
            busy                     <= 1'b0;
            done_pulse               <= 1'b0;
            error                    <= 1'b0;
            jedec_id                 <= 24'h000000;
            jedec_id_valid           <= 1'b0;

            tx_byte                  <= 8'h00;
            rx_byte                  <= 8'h00;
            bit_cnt                  <= 3'd7;
            spi_step_cnt             <= 8'd0;

            cur_addr                 <= 24'd0;
            flash_base_addr_latched  <= 24'd0;
            byte_cnt                 <= 16'd0;
            page_byte_cnt            <= 9'd0;
            status_reg1              <= 8'h00;
            wait_cnt                 <= 24'd0;
            wake_wait_cnt            <= 24'd0;
            wr_wren_next_state       <= S_WR_SE_LOAD;
            poll_next_state          <= S_IDLE;
            cs_gap_next_state        <= S_IDLE;
            next_after_wake          <= S_IDLE;

            wr_data_latch            <= 8'h00;
            rd_data_latch            <= 8'h00;
        end else begin

            out_data_we <= 1'b0;
            done_pulse  <= 1'b0;
            jedec_id_valid <= 1'b0;
            busy        <= (state != S_IDLE);
            if (is_shift_state) begin
                if (spi_step_cnt == (SPI_STEP_DIV - 1))
                    spi_step_cnt <= 8'd0;
                else
                    spi_step_cnt <= spi_step_cnt + 1'b1;
            end else begin
                spi_step_cnt <= 8'd0;
            end

            if (is_shift_state && (spi_step_cnt != (SPI_STEP_DIV - 1))) begin

            end else case (state)

                S_IDLE: begin
                    flash_cs_n   <= 1'b1;
                    flash_sck    <= 1'b0;
                    flash_mosi   <= 1'b0;
                    flash_status  <= ST_IDLE_S;
                    busy          <= 1'b0;

                    // Request arbitration priority: write > read > id.
                    if (flash_write_req) begin
                        flash_base_addr_latched <= flash_base_addr;
                        error       <= 1'b0;
                        flash_status <= ST_WRITE_S;
                        next_after_wake <= S_WR_INIT;
                        state       <= S_WAKE_LOAD;
                    end else if (flash_read_req) begin
                        flash_base_addr_latched <= flash_base_addr;
                        error       <= 1'b0;
                        flash_status <= ST_READ_S;
                        next_after_wake <= S_RD_INIT;
                        state       <= S_WAKE_LOAD;
                    end else if (flash_id_req) begin
                        error       <= 1'b0;
                        flash_status <= ST_READ_S;
                        next_after_wake <= S_ID_LOAD;
                        state       <= S_WAKE_LOAD;
                    end
                end

                S_WAKE_LOAD: begin
                    flash_cs_n <= 1'b0;
                    spi_shift_start(CMD_RDPD);
                    state <= S_WAKE_SHIFT;
                end

                S_WAKE_SHIFT: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            state <= S_WAKE_END;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WAKE_END: begin
                    flash_cs_n <= 1'b1;
                    flash_sck  <= 1'b0;
                    wake_wait_cnt <= WAKE_WAIT_CYCLES;
                    state <= S_WAKE_WAIT;
                end

                S_WAKE_WAIT: begin
                    if (wake_wait_cnt == 24'd0) begin
                        state <= next_after_wake;
                    end else begin
                        wake_wait_cnt <= wake_wait_cnt - 1'b1;
                    end
                end

                S_WR_INIT: begin
                    byte_cnt                 <= 16'd0;
                    page_byte_cnt            <= 9'd0;
                    cur_addr                 <= flash_base_addr_latched;
                    wr_wren_next_state       <= S_WR_SE_LOAD;
                    state <= S_WR_WREN_LOAD;
                end

                S_WR_WREN_LOAD: begin
                    flash_cs_n <= 1'b0;
                    spi_shift_start(CMD_WREN);
                    state <= S_WR_WREN_SHIFT;
                end

                S_WR_WREN_SHIFT: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            state <= S_WR_WREN_END;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_WREN_END: begin
                    flash_cs_n <= 1'b1;
                    flash_sck  <= 1'b0;
                    cs_gap_next_state <= wr_wren_next_state;
                    state      <= S_CS_GAP;
                end

                S_WR_SE_LOAD: begin
                    flash_cs_n <= 1'b0;
                    spi_shift_start(CMD_SE);
                    state <= S_WR_SE_SHIFT_CMD;
                end

                S_WR_SE_SHIFT_CMD: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(cur_addr[23:16]);
                            state <= S_WR_SE_SHIFT_A23;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_SE_SHIFT_A23: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(cur_addr[15:8]);
                            state <= S_WR_SE_SHIFT_A15;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_SE_SHIFT_A15: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(cur_addr[7:0]);
                            state <= S_WR_SE_SHIFT_A7;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_SE_SHIFT_A7: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            state <= S_WR_SE_END;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_SE_END: begin
                    flash_cs_n <= 1'b1;
                    flash_sck  <= 1'b0;
                    flash_status <= ST_WAIT_S;
                    wait_cnt <= WAIT_ERASE_CYCLES;
                    state <= S_WR_WAIT_ERASE;
                end

                S_WR_WAIT_ERASE: begin
                    if (wait_cnt == 24'd0) begin
                        flash_status <= ST_WRITE_S;
                        wr_wren_next_state <= S_WR_PP_LOAD;
                        state <= S_WR_WREN_LOAD;
                    end else begin
                        wait_cnt <= wait_cnt - 1'b1;
                    end
                end

                S_WR_PP_LOAD: begin
                    flash_cs_n      <= 1'b0;
                    page_byte_cnt   <= 9'd0;
                    spi_shift_start(CMD_PP);
                    state <= S_WR_PP_SHIFT_CMD;
                end

                S_WR_PP_SHIFT_CMD: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(cur_addr[23:16]);
                            state <= S_WR_PP_SHIFT_A23;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_PP_SHIFT_A23: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(cur_addr[15:8]);
                            state <= S_WR_PP_SHIFT_A15;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_PP_SHIFT_A15: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(cur_addr[7:0]);
                            state <= S_WR_PP_SHIFT_A7;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_PP_SHIFT_A7: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            state <= S_WR_PP_REQDATA;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_PP_REQDATA: begin
                    state <= S_WR_PP_WAITDATA;
                end

                S_WR_PP_WAITDATA: begin
                    wr_data_latch <= in_data;
                    spi_shift_start(in_data);
                    state <= S_WR_PP_SHIFT_DATA;
                end

                S_WR_PP_SHIFT_DATA: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            byte_cnt      <= byte_cnt + 1'b1;
                            page_byte_cnt <= page_byte_cnt + 1'b1;

                            if (frame_last_byte || page_last_byte) begin
                                state <= S_WR_PP_END;
                            end else begin
                                state <= S_WR_PP_REQDATA;
                            end
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_PP_END: begin
                    flash_cs_n  <= 1'b1;
                    flash_sck   <= 1'b0;
                    flash_status <= ST_WAIT_S;
                    wait_cnt <= WAIT_PP_CYCLES;
                    state    <= S_WR_WAIT_PP;
                end

                S_WR_WAIT_PP: begin
                    if (wait_cnt == 24'd0) begin
                        flash_status <= ST_WRITE_S;
                        state <= S_WR_NEXT_PAGE;
                    end else begin
                        wait_cnt <= wait_cnt - 1'b1;
                    end
                end

                S_WR_POLL_LOAD: begin
                    flash_cs_n <= 1'b0;
                    spi_shift_start(CMD_RDSR1);
                    state <= S_WR_POLL_SHIFT_CMD;
                end

                S_WR_POLL_SHIFT_CMD: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(8'h00);
                            state <= S_WR_POLL_SHIFT_DATA;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_POLL_SHIFT_DATA: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        rx_byte[bit_cnt] <= flash_miso;
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            status_reg1 <= {rx_byte[7:1], flash_miso};
                            state <= S_WR_POLL_END;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_POLL_END: begin
                    flash_cs_n <= 1'b1;
                    flash_sck  <= 1'b0;

                    if (status_reg1[0]) begin
                        cs_gap_next_state <= S_WR_POLL_LOAD;
                        state <= S_CS_GAP;
                    end else begin
                        cs_gap_next_state <= poll_next_state;
                        state <= S_CS_GAP;
                    end
                end

                S_CS_GAP: begin
                    flash_cs_n <= 1'b1;
                    flash_sck  <= 1'b0;
                    state      <= cs_gap_next_state;
                end

                S_WR_NEXT_PAGE: begin
                    if (byte_cnt >= FRAME_BYTES) begin
                        flash_status <= ST_DONE_S;
                        state       <= S_WR_DONE;
                    end else begin
                        cur_addr    <= flash_base_addr_latched + byte_cnt;
                        flash_status <= ST_WRITE_S;
                        wr_wren_next_state <= S_WR_PP_LOAD;
                        state       <= S_WR_WREN_LOAD;
                    end
                end

                S_WR_DONE: begin
                    flash_status <= ST_IDLE_S;
                    done_pulse   <= 1'b1;
                    state <= S_IDLE;
                end

                S_RD_INIT: begin
                    cur_addr                  <= flash_base_addr_latched;
                    byte_cnt                  <= 16'd0;
                    state                     <= S_RD_CMD_LOAD;
                end

                S_RD_CMD_LOAD: begin
                    flash_cs_n <= 1'b0;
                    spi_shift_start(CMD_READ);
                    state <= S_RD_SHIFT_CMD;
                end

                S_RD_SHIFT_CMD: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(cur_addr[23:16]);
                            state <= S_RD_SHIFT_A23;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_RD_SHIFT_A23: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(cur_addr[15:8]);
                            state <= S_RD_SHIFT_A15;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_RD_SHIFT_A15: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(cur_addr[7:0]);
                            state <= S_RD_SHIFT_A7;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_RD_SHIFT_A7: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(8'h00);
                            state <= S_RD_SHIFT_DATA;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_RD_SHIFT_DATA: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck  <= 1'b1;
                    end else begin
                        rx_byte[bit_cnt] <= flash_miso;
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            rd_data_latch <= {rx_byte[7:1], flash_miso};
                            out_data      <= {rx_byte[7:1], flash_miso};
                            out_data_we   <= 1'b1;
                            out_data_idx  <= byte_cnt[ADDR_DEPTH-1:0];
                            byte_cnt      <= byte_cnt + 1'b1;

                            if (frame_last_byte) begin
                                state <= S_RD_END;
                            end else begin
                                cur_addr <= cur_addr + 1'b1;
                                spi_shift_start(8'h00);
                            end
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_RD_END: begin
                    flash_cs_n  <= 1'b1;
                    flash_sck   <= 1'b0;
                    flash_status <= ST_DONE_S;
                    state       <= S_RD_DONE;
                end

                S_RD_DONE: begin
                    flash_status <= ST_IDLE_S;
                    done_pulse   <= 1'b1;
                    state <= S_IDLE;
                end

                S_ID_LOAD: begin
                    flash_cs_n <= 1'b0;
                    spi_shift_start(CMD_RDID);
                    state <= S_ID_SHIFT_CMD;
                end

                S_ID_SHIFT_CMD: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(8'h00);
                            state <= S_ID_SHIFT_B2;
                        end else begin
                            flash_mosi <= tx_byte[bit_cnt - 1'b1];
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_ID_SHIFT_B2: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck <= 1'b1;
                    end else begin
                        rx_byte[bit_cnt] <= flash_miso;
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            jedec_id[23:16] <= {rx_byte[7:1], flash_miso};
                            spi_shift_start(8'h00);
                            state <= S_ID_SHIFT_B1;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_ID_SHIFT_B1: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck <= 1'b1;
                    end else begin
                        rx_byte[bit_cnt] <= flash_miso;
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            jedec_id[15:8] <= {rx_byte[7:1], flash_miso};
                            spi_shift_start(8'h00);
                            state <= S_ID_SHIFT_B0;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_ID_SHIFT_B0: begin
                    if (flash_sck == 1'b0) begin
                        flash_sck <= 1'b1;
                    end else begin
                        rx_byte[bit_cnt] <= flash_miso;
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            jedec_id[7:0] <= {rx_byte[7:1], flash_miso};
                            state <= S_ID_END;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_ID_END: begin
                    flash_cs_n <= 1'b1;
                    flash_sck  <= 1'b0;
                    flash_status <= ST_IDLE_S;
                    jedec_id_valid <= 1'b1;
                    state <= S_IDLE;
                end

                default: begin
                    state       <= S_IDLE;
                    flash_status <= ST_ERR_S;
                    error       <= 1'b1;
                end
            endcase
        end
    end

endmodule
