module spi_flash_ctrl #(
    parameter integer ADDR_DEPTH      = 10,
    parameter integer DATA_DEPTH      = 8,
    parameter [23:0] FLASH_BASE_ADDR  = 24'h380000
)(
    input  wire                    clk_25m,
    input  wire                    rst_n,

    // 用户请求
    input  wire                    flash_write_req,
    output reg                     in_flash_temp_buffer_req,

    input  wire                    flash_read_req,
    output reg                     out_flash_temp_buffer_req,

    input  wire [DATA_DEPTH-1:0]   in_data,
    output reg  [DATA_DEPTH-1:0]   out_data,

    output reg  [2:0]              flash_status,

    // SPI 物理接口
    output reg                     flash_cs_n,
    output reg                     flash_sck,
    output reg                     flash_mosi,
    input  wire                    flash_miso
);

    localparam integer FRAME_BYTES = (1 << ADDR_DEPTH);

    // W25Q32JV standard SPI commands

    localparam [7:0] CMD_WREN  = 8'h06; // Write Enable
    localparam [7:0] CMD_RDSR1 = 8'h05; // Read Status Register-1
    localparam [7:0] CMD_READ  = 8'h03; // Read Data
    localparam [7:0] CMD_PP    = 8'h02; // Page Program

  
    // flash_status 定义
    // 000: idle
    // 001: write flow
    // 010: read flow
    // 011: wait busy clear
    // 100: done pulse-like state
    // 111: error/reserved
 
    localparam [2:0] ST_IDLE_S   = 3'b000;
    localparam [2:0] ST_WRITE_S  = 3'b001;
    localparam [2:0] ST_READ_S   = 3'b010;
    localparam [2:0] ST_WAIT_S   = 3'b011;
    localparam [2:0] ST_DONE_S   = 3'b100;
    localparam [2:0] ST_ERR_S    = 3'b111;

    // 顶层状态机
    localparam [5:0]
        S_IDLE                 = 6'd0,

        // 写流程
        S_WR_BUF_REQ           = 6'd1,
        S_WR_BUF_WAIT          = 6'd2,
        S_WR_WREN_LOAD         = 6'd3,
        S_WR_WREN_SHIFT        = 6'd4,
        S_WR_WREN_END          = 6'd5,

        S_WR_PP_LOAD           = 6'd6,
        S_WR_PP_SHIFT_CMD      = 6'd7,
        S_WR_PP_SHIFT_A23      = 6'd8,
        S_WR_PP_SHIFT_A15      = 6'd9,
        S_WR_PP_SHIFT_A7       = 6'd10,
        S_WR_PP_SHIFT_DATA     = 6'd11,
        S_WR_PP_END            = 6'd12,

        S_WR_POLL_LOAD         = 6'd13,
        S_WR_POLL_SHIFT_CMD    = 6'd14,
        S_WR_POLL_SHIFT_DATA   = 6'd15,
        S_WR_POLL_END          = 6'd16,
        S_WR_NEXT_PAGE         = 6'd17,
        S_WR_DONE              = 6'd18,

        // 读流程
        S_RD_BUF_REQ           = 6'd19,
        S_RD_CMD_LOAD          = 6'd20,
        S_RD_SHIFT_CMD         = 6'd21,
        S_RD_SHIFT_A23         = 6'd22,
        S_RD_SHIFT_A15         = 6'd23,
        S_RD_SHIFT_A7          = 6'd24,
        S_RD_SHIFT_DATA        = 6'd25,
        S_RD_END               = 6'd26,
        S_RD_DONE              = 6'd27;

    reg [5:0] state;

    // SPI bit engine (Mode 0: CPOL=0, CPHA=0)
    // 约定：
    // 1) state切到某个 SHIFT 状态时，先把 tx_byte 准备好
    // 2) bit_cnt 从 7 递减到 0
    // 3) 每个系统拍翻转一次 flash_sck
    //    - sck=0 -> 输出 MOSI
    //    - sck=1 -> 采样 MISO, 并在最后一位后退出
    reg [7:0] tx_byte;
    reg [7:0] rx_byte;
    reg [2:0] bit_cnt;

    reg [23:0] cur_addr;
    reg [15:0] byte_cnt;
    reg [8:0]  page_byte_cnt;   // 最多计 256
    reg [7:0]  status_reg1;

    reg [7:0]  wr_data_latch;
    reg [7:0]  rd_data_latch;

    // 读写页内/整帧控制
    wire frame_last_byte = (byte_cnt == FRAME_BYTES-1);
    wire page_last_byte  = (page_byte_cnt == 9'd255);

    // SPI shift 公共过程通过状态机直接写，不额外拆子模块
    task spi_shift_start;
        input [7:0] din;
        begin
            tx_byte  <= din;
            rx_byte  <= 8'h00;
            bit_cnt  <= 3'd7;
            flash_sck <= 1'b0;
        end
    endtask

    // 主状态机
    always @(posedge clk_25m or negedge rst_n) begin
        if (!rst_n) begin
            state                    <= S_IDLE;

            flash_cs_n               <= 1'b1;
            flash_sck                <= 1'b0;
            flash_mosi               <= 1'b0;

            in_flash_temp_buffer_req <= 1'b0;
            out_flash_temp_buffer_req<= 1'b0;
            out_data                 <= {DATA_DEPTH{1'b0}};
            flash_status              <= ST_IDLE_S;

            tx_byte                  <= 8'h00;
            rx_byte                  <= 8'h00;
            bit_cnt                  <= 3'd7;

            cur_addr                 <= FLASH_BASE_ADDR;
            byte_cnt                 <= 16'd0;
            page_byte_cnt            <= 9'd0;
            status_reg1              <= 8'h00;

            wr_data_latch            <= 8'h00;
            rd_data_latch            <= 8'h00;
        end else begin
            // 默认单拍信号拉低
            in_flash_temp_buffer_req  <= 1'b0;
            out_flash_temp_buffer_req <= 1'b0;

            case (state)
          
                // IDLE
                S_IDLE: begin
                    flash_cs_n   <= 1'b1;
                    flash_sck    <= 1'b0;
                    flash_mosi   <= 1'b0;
                    cur_addr     <= FLASH_BASE_ADDR;
                    byte_cnt     <= 16'd0;
                    page_byte_cnt<= 9'd0;
                    flash_status  <= ST_IDLE_S;

                    if (flash_write_req) begin
                        flash_status <= ST_WRITE_S;
                        state       <= S_WR_BUF_REQ;
                    end else if (flash_read_req) begin
                        flash_status <= ST_READ_S;
                        state       <= S_RD_BUF_REQ;
                    end
                end

                // 写流程：先向 temp_buffer_in 发读请求
                S_WR_BUF_REQ: begin
                    in_flash_temp_buffer_req <= 1'b1;
                    byte_cnt                 <= 16'd0;
                    page_byte_cnt            <= 9'd0;
                    cur_addr                 <= FLASH_BASE_ADDR;
                    state                    <= S_WR_BUF_WAIT;
                end

                // 给 temp_buffer 一个拍子开始吐数据
                S_WR_BUF_WAIT: begin
                    state <= S_WR_WREN_LOAD;
                end

                //  Write Enable
                S_WR_WREN_LOAD: begin
                    flash_cs_n <= 1'b0;
                    spi_shift_start(CMD_WREN);
                    state <= S_WR_WREN_SHIFT;
                end

                S_WR_WREN_SHIFT: begin
                    if (flash_sck == 1'b0) begin
                        flash_mosi <= tx_byte[bit_cnt];
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            state <= S_WR_WREN_END;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_WREN_END: begin
                    flash_cs_n <= 1'b1;
                    flash_sck  <= 1'b0;
                    state      <= S_WR_PP_LOAD;
                end

                // -Page Program 命令 + 24位地址
                S_WR_PP_LOAD: begin
                    flash_cs_n      <= 1'b0;
                    page_byte_cnt   <= 9'd0;
                    spi_shift_start(CMD_PP);
                    state <= S_WR_PP_SHIFT_CMD;
                end

                S_WR_PP_SHIFT_CMD: begin
                    if (flash_sck == 1'b0) begin
                        flash_mosi <= tx_byte[bit_cnt];
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(cur_addr[23:16]);
                            state <= S_WR_PP_SHIFT_A23;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_PP_SHIFT_A23: begin
                    if (flash_sck == 1'b0) begin
                        flash_mosi <= tx_byte[bit_cnt];
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(cur_addr[15:8]);
                            state <= S_WR_PP_SHIFT_A15;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_PP_SHIFT_A15: begin
                    if (flash_sck == 1'b0) begin
                        flash_mosi <= tx_byte[bit_cnt];
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(cur_addr[7:0]);
                            state <= S_WR_PP_SHIFT_A7;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_PP_SHIFT_A7: begin
                    if (flash_sck == 1'b0) begin
                        flash_mosi <= tx_byte[bit_cnt];
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            wr_data_latch <= in_data;
                            spi_shift_start(in_data);
                            state <= S_WR_PP_SHIFT_DATA;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                // 连续写整页，最多256B
                S_WR_PP_SHIFT_DATA: begin
                    if (flash_sck == 1'b0) begin
                        flash_mosi <= tx_byte[bit_cnt];
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            byte_cnt      <= byte_cnt + 1'b1;
                            page_byte_cnt <= page_byte_cnt + 1'b1;

                            if (frame_last_byte || page_last_byte) begin
                                state <= S_WR_PP_END;
                            end else begin
                                wr_data_latch <= in_data;
                                spi_shift_start(in_data);
                            end
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_PP_END: begin
                    flash_cs_n  <= 1'b1;
                    flash_sck   <= 1'b0;
                    flash_status <= ST_WAIT_S;
                    state       <= S_WR_POLL_LOAD;
                end

                //轮询 BUSY 位
                S_WR_POLL_LOAD: begin
                    flash_cs_n <= 1'b0;
                    spi_shift_start(CMD_RDSR1);
                    state <= S_WR_POLL_SHIFT_CMD;
                end

                S_WR_POLL_SHIFT_CMD: begin
                    if (flash_sck == 1'b0) begin
                        flash_mosi <= tx_byte[bit_cnt];
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(8'h00);
                            state <= S_WR_POLL_SHIFT_DATA;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_WR_POLL_SHIFT_DATA: begin
                    if (flash_sck == 1'b0) begin
                        flash_mosi <= 1'b0;
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

                    // BUSY = status_reg1[0]
                    if ({rx_byte[7:1], flash_miso}[0]) begin
                        state <= S_WR_POLL_LOAD;
                    end else begin
                        state <= S_WR_NEXT_PAGE;
                    end
                end

                S_WR_NEXT_PAGE: begin
                    if (byte_cnt >= FRAME_BYTES) begin
                        flash_status <= ST_DONE_S;
                        state       <= S_WR_DONE;
                    end else begin
                        cur_addr    <= FLASH_BASE_ADDR + byte_cnt;
                        flash_status <= ST_WRITE_S;
                        state       <= S_WR_WREN_LOAD;
                    end
                end

                S_WR_DONE: begin
                    flash_status <= ST_IDLE_S;
                    state <= S_IDLE;
                end

                // 读流程：先通知 temp_buffer_out 准备接收
                S_RD_BUF_REQ: begin
                    out_flash_temp_buffer_req <= 1'b1;
                    cur_addr                  <= FLASH_BASE_ADDR;
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
                        flash_mosi <= tx_byte[bit_cnt];
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(cur_addr[23:16]);
                            state <= S_RD_SHIFT_A23;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_RD_SHIFT_A23: begin
                    if (flash_sck == 1'b0) begin
                        flash_mosi <= tx_byte[bit_cnt];
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(cur_addr[15:8]);
                            state <= S_RD_SHIFT_A15;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_RD_SHIFT_A15: begin
                    if (flash_sck == 1'b0) begin
                        flash_mosi <= tx_byte[bit_cnt];
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(cur_addr[7:0]);
                            state <= S_RD_SHIFT_A7;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_RD_SHIFT_A7: begin
                    if (flash_sck == 1'b0) begin
                        flash_mosi <= tx_byte[bit_cnt];
                        flash_sck  <= 1'b1;
                    end else begin
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            spi_shift_start(8'h00);
                            state <= S_RD_SHIFT_DATA;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                // 连续读取整帧
                S_RD_SHIFT_DATA: begin
                    if (flash_sck == 1'b0) begin
                        flash_mosi <= 1'b0;
                        flash_sck  <= 1'b1;
                    end else begin
                        rx_byte[bit_cnt] <= flash_miso;
                        flash_sck <= 1'b0;
                        if (bit_cnt == 0) begin
                            rd_data_latch <= {rx_byte[7:1], flash_miso};
                            out_data      <= {rx_byte[7:1], flash_miso};
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
                    state <= S_IDLE;
                end

                default: begin
                    state       <= S_IDLE;
                    flash_status <= ST_ERR_S;
                end
            endcase
        end
    end

endmodule