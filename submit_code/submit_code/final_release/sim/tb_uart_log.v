// tb_uart_log: 验证 uart_logger 在 sd_init_done 上升时发出 "SD OK\r\n"
`timescale 1ns/1ps
module tb_uart_log;
reg clk = 0;
reg rst = 1;
reg sd_init_done = 0;
always #5 clk = ~clk;   // 100MHz

wire uart_tx;

uart_logger #(
    .CLK_FREQ_HZ(100_000_000),
    .BAUD_RATE(115200)
) u_log (
    .clk(clk), .rst(rst),
    .sd_init_done(sd_init_done),
    .img_idx(2'd0), .img_total(3'd4),
    .load_error(1'b0), .bmp_err_code(8'd0),
    .uart_tx(uart_tx)
);

// 串行接收解码（16x 过采样简化：每位中间采样）
localparam integer BIT_NS = 8680;  // 1e9/115200 ≈ 8681ns
integer i;
reg [7:0] rx_byte;
initial begin
    rst = 1;
    repeat (10) @(posedge clk);
    rst = 0;
    #1000;
    // 触发事件
    sd_init_done = 1;
    // 等起始位
    @(negedge uart_tx);
    #(BIT_NS/2);
    for (i = 0; i < 32; i = i + 1) begin
        // 读 8 数据位
        for (integer b = 0; b < 8; b = b + 1) begin
            #(BIT_NS);
            rx_byte[b] = uart_tx;
        end
        #(BIT_NS/2);   // 停止位中点，回到字节对齐
        if (rx_byte >= 8'h20 && rx_byte < 8'h7f)
            $display("RX: '%c' (%02x)", rx_byte, rx_byte);
        else
            $display("RX: %02x", rx_byte);
        // 找下一起始位（带超时：线空闲则退出）
        fork
            begin: wait_start
                @(negedge uart_tx);
            end
            begin
                #(BIT_NS*3);
                if (uart_tx) begin
                    $display("--- line idle, done ---");
                    $finish;
                end
            end
        join_any
        disable fork;
        #(BIT_NS/2);
    end
    $finish;
end
endmodule
