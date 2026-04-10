module key_debouncer #(
    parameter KEY_WIDTH = 5,
    // 50MHz 时钟下，20ms 的计数值为 1,000,000
    parameter WAIT_TIME = 20'd1_000_000 
)(
    input  wire                 clk_50m,
    
    // 原始机械按键输入
    input  wire [KEY_WIDTH-1:0] key_raw,
    
    // 消抖后的干净输出
    output reg  [KEY_WIDTH-1:0] key_clean
);
    // 上电初始化 (Power-on Initialization)
    // 由于复位信号本身在此处被消抖，模块内部所有寄存器必须显式赋初值
    // 按键通常为上拉电阻，初始状态应为全 1 (高电平)

    initial begin
        key_clean = {KEY_WIDTH{1'b1}};
    end


    // 两级触发器同步 (消除亚稳态)
    reg [KEY_WIDTH-1:0] key_sync_0 = {KEY_WIDTH{1'b1}};
    reg [KEY_WIDTH-1:0] key_sync_1 = {KEY_WIDTH{1'b1}};

    always @(posedge clk_50m) begin
        key_sync_0 <= key_raw;
        key_sync_1 <= key_sync_0;
    end

    // 3. 状态翻转检测与延时计数
    reg [19:0] cnt = 20'd0;
    
    // 当同步后的输入与当前稳定输出不一致时，说明按键正在动作或正在抖动
    wire state_changed = (key_sync_1 != key_clean);

    always @(posedge clk_50m) begin
        if (state_changed) begin
            // 状态发生变化，计数器开始累加
            cnt <= cnt + 1'b1;
            if (cnt == WAIT_TIME) begin
                // 维持了 20ms 未再发生变化，确认状态稳定，更新输出
                key_clean <= key_sync_1;
                cnt <= 20'd0;
            end
        end else begin
            // 一旦在 20ms 内又发生翻转（抖动），计数器立刻清零重新计算
            cnt <= 20'd0;
        end
    end

endmodule