module VGA_top #(
    parameter DATA_W = 8,
    parameter ADDR_W = 10,
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    input  wire                  clk_25m,
    input  wire                  rst_n,

    // 来自 pingpong_buffer
    input  wire                  frame_valid,
    input  wire [ADDR_W-1:0]     active_frame_start_addr,
    output reg  [ADDR_W-1:0]     raddr,
    input  wire [DATA_W-1:0]     rdata_ch1,
    input  wire [DATA_W-1:0]     rdata_ch2,
    input  wire [DATA_W-1:0]     rdata_ch3,
    input  wire [DATA_W-1:0]     rdata_ch4,
    input  wire [7:0]            dbg_status,
    output reg                   rd_frame_done,

    // VGA输出
    output wire                  hsync,
    output wire                  vsync,
    output wire [15:0]           rgb565

    //来自HMI
    //input  wire           ui_page,
    //input  wire           ui_cursor,
    //input  wire           ui_curr_edit_mode,
    //input  wire           ui_curr_edit_valu,
    //input  wire           ui_active_src_sel,
    //input  wire           view_ch_sel,
);

    wire         de_timing;
    wire [10:0]  pix_x_timing;
    wire [9:0]   pix_y_timing;
    wire         frame_start;

    reg          de_d1;
    reg [10:0]   pix_x_d1;
    reg [9:0]    pix_y_d1;
    reg          sample_valid_d1;

    reg [ADDR_W-1:0] frame_base_addr_latched;

    // 一帧结束标志：最后一个有效像素
    wire last_active_pixel;
    assign last_active_pixel = de_d1 &&
                               (pix_x_d1 == H_ACTIVE - 1) &&
                               (pix_y_d1 == V_ACTIVE - 1);

    vga_timing u_vga_timing (
        .clk_pix    (clk_25m),
        .rst_n      (rst_n),
        .hsync      (hsync),
        .vsync      (vsync),
        .de         (de_timing),
        .pix_x      (pix_x_timing),
        .pix_y      (pix_y_timing),
        .frame_start(frame_start)
    );

    // 帧开始时锁存本帧使用的起始地址
    // 这样一整帧显示过程中地址基准不乱跳
    always @(posedge clk_25m or negedge rst_n) begin
        if (!rst_n) begin
            frame_base_addr_latched <= {ADDR_W{1'b0}};
        end else if (frame_start) begin
            frame_base_addr_latched <= active_frame_start_addr;
        end
    end

    // 发读地址 + 像素坐标打一拍，适配BRAM同步读
    always @(posedge clk_25m or negedge rst_n) begin
        if (!rst_n) begin
            raddr           <= {ADDR_W{1'b0}};
            de_d1           <= 1'b0;
            pix_x_d1        <= 11'd0;
            pix_y_d1        <= 10'd0;
            sample_valid_d1 <= 1'b0;
            rd_frame_done   <= 1'b0;
        end else begin
            rd_frame_done <= 1'b0;

            // 默认打一拍对齐，读地址和读数据不是同一拍
            de_d1           <= de_timing;
            pix_x_d1        <= pix_x_timing;
            pix_y_d1        <= pix_y_timing;
            sample_valid_d1 <= de_timing && frame_valid;

            if (de_timing && frame_valid) begin
                raddr <= frame_base_addr_latched + pix_x_timing[ADDR_W-1:0];
            end else begin
                raddr <= {ADDR_W{1'b0}};
            end

            if (last_active_pixel) begin
                rd_frame_done <= 1'b1;
            end
        end
    end

    waveform_renderer #(
        .H_ACTIVE(H_ACTIVE),
        .V_ACTIVE(V_ACTIVE)
    ) u_waveform_renderer (
        .clk_pix    (clk_25m),
        .de         (de_d1),
        .pix_x      (pix_x_d1),
        .pix_y      (pix_y_d1),
        .sample_valid(sample_valid_d1),
        .sample_ch1 (rdata_ch1),
        .sample_ch2 (rdata_ch2),
        .sample_ch3 (rdata_ch3),
        .sample_ch4 (rdata_ch4),
        .debug_status(dbg_status),
        .rgb565     (rgb565)
    );

endmodule
