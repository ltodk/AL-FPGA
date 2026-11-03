// =====================================================================
// uart_tx: UART 发送器（8N1，无奇偶校验，1 停止位）
//   第5讲 Step2 可观测性——串口日志底层
//   波特率 = CLK_FREQ_HZ / BAUD_RATE（分频计满值 DIV - 1）
//   默认 100MHz / 115200 = 868.06 -> 取 868（误差约 0.007%）
// =====================================================================
module uart_tx #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BAUD_RATE   = 115200
)(
    input  wire       clk,
    input  wire       rst,          // 高有效复位
    input  wire [7:0] tx_data,      // 待发送字节
    input  wire       tx_start,     // 单拍启动脉冲（busy 期间被忽略）
    output reg        tx_busy,      // 正在发送
    output reg        uart_tx       // 串行输出（空闲为高）
);

localparam integer DIV = CLK_FREQ_HZ / BAUD_RATE;   // 每 bit 时钟周期数

reg [15:0] baud_cnt;      // 分频计数
reg [3:0]  bit_idx;       // 当前发送 bit 位置（0=起始位,1..8=数据,9=停止位）
reg [8:0]  shift_reg;     // 移位寄存器：{停止位, data[7:0]}，起始位单独发送
reg        sending;       // 发送进行中

always @(posedge clk or posedge rst) begin
    if (rst) begin
        baud_cnt <= 16'd0;
        bit_idx  <= 4'd0;
        shift_reg<= 9'd0;
        sending  <= 1'b0;
        tx_busy  <= 1'b0;
        uart_tx  <= 1'b1;
    end else begin
        // 启动：空闲时锁存数据并进入发送
        if (tx_start && !sending) begin
            shift_reg <= {1'b1, tx_data};   // 停止位=1，数据位
            bit_idx   <= 4'd0;
            sending   <= 1'b1;
            tx_busy   <= 1'b1;
            baud_cnt  <= 16'd0;
            uart_tx   <= 1'b0;              // 立即输出起始位
        end else if (sending) begin
            if (baud_cnt >= DIV - 1) begin
                baud_cnt <= 16'd0;
                if (bit_idx == 4'd9) begin
                    // 已发完停止位，结束
                    sending <= 1'b0;
                    tx_busy <= 1'b0;
                    uart_tx <= 1'b1;
                end else begin
                    // 依次输出 数据位/停止位
                    uart_tx <= shift_reg[0];
                    shift_reg <= {1'b0, shift_reg[8:1]};
                    bit_idx  <= bit_idx + 4'd1;
                end
            end else begin
                baud_cnt <= baud_cnt + 16'd1;
            end
        end
    end
end

endmodule
