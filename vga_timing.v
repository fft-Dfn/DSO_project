module vga_timing #(
    parameter H_ACTIVE = 640,
    parameter H_FP     = 16,
    parameter H_SYNC   = 96,
    parameter H_BP     = 48,

    parameter V_ACTIVE = 480,
    parameter V_FP     = 10,
    parameter V_SYNC   = 2,
    parameter V_BP     = 33
)(
    input  wire        clk_pix,
    input  wire        rst_n,

    output reg         hsync,
    output reg         vsync,
    output reg         de,          // display enable
    output reg [10:0]  pix_x,
    output reg [9:0]   pix_y,
    output reg         frame_start  // 每帧开始脉冲
);

    localparam H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;//800
    localparam V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;//525

    reg [10:0] h_cnt;
    reg [9:0]  v_cnt;

    always @(posedge clk_pix or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt       <= 11'd0;
            v_cnt       <= 10'd0;
            hsync       <= 1'b1;
            vsync       <= 1'b1;
            de          <= 1'b0;
            pix_x       <= 11'd0;
            pix_y       <= 10'd0;
            frame_start <= 1'b0;
        end else begin
            frame_start <= 1'b0;

            if (h_cnt == H_TOTAL - 1) begin
                h_cnt <= 11'd0;
                if (v_cnt == V_TOTAL - 1) begin
                    v_cnt       <= 10'd0;
                    frame_start <= 1'b1;
                end else begin
                    v_cnt <= v_cnt + 10'd1;
                end
            end else begin
                h_cnt <= h_cnt + 11'd1;
            end

            // VGA负极性同步
            hsync <= ~((h_cnt >= H_ACTIVE + H_FP) &&
                       (h_cnt <  H_ACTIVE + H_FP + H_SYNC));

            vsync <= ~((v_cnt >= V_ACTIVE + V_FP) &&
                       (v_cnt <  V_ACTIVE + V_FP + V_SYNC));

            de <= (h_cnt < H_ACTIVE) && (v_cnt < V_ACTIVE);

            if ((h_cnt < H_ACTIVE) && (v_cnt < V_ACTIVE)) begin
                pix_x <= h_cnt;
                pix_y <= v_cnt;//比v_cnt慢一个周期，跟de,vsync同步
            end else begin
                pix_x <= 11'd0;
                pix_y <= 10'd0;
            end
        end
    end

endmodule