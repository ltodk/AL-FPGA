// tb_uart_err8: 验证 uart_logger 输出 8 位错误码 "ERR:0x31"~"ERR:0x38"
`timescale 1ns/1ps
module tb_uart_err8;
reg clk = 0;
reg rst = 1;
reg sd_init_done = 0;
reg load_error = 0;
reg [7:0] bmp_err_code = 8'd0;
always #5 clk = ~clk;   // 100MHz

wire uart_tx;

uart_logger #(
    .CLK_FREQ_HZ(100_000_000),
    .BAUD_RATE(115200)
) u_log (
    .clk(clk), .rst(rst),
    .sd_init_done(sd_init_done),
    .img_idx(2'd0), .img_total(3'd4),
    .load_error(load_error), .bmp_err_code(bmp_err_code),
    .uart_tx(uart_tx)
);

// 串行接收：采集一串字节直到线路空闲
localparam integer BIT_NS = 8680;  // 1e9/115200
reg [7:0] rx_buf [0:31];
integer rx_cnt = 0;
integer i;

// 接收一个字节（起始位 + 8 数据位 + 停止位）
task rx_byte_task;
    output [7:0] byte_out;
    begin
        for (integer b = 0; b < 8; b = b + 1) begin
            #(BIT_NS);
            byte_out[b] = uart_tx;
        end
        #(BIT_NS/2);   // 停止位中点
    end
endtask

initial begin
    rst = 1;
    repeat (10) @(posedge clk);
    rst = 0;
    #1000;

    // 触发错误事件：bmp_err_code = 0x31 (ERR_SIGNATURE)
    bmp_err_code = 8'h31;
    load_error = 1;
    @(posedge clk);
    load_error = 0;

    // 等起始位
    @(negedge uart_tx);
    #(BIT_NS/2);

    // 采集字节直到空闲
    rx_cnt = 0;
    while (rx_cnt < 32) begin
        rx_byte_task(rx_buf[rx_cnt]);
        rx_cnt = rx_cnt + 1;
        // 等下一字节起始位，超时则说明消息结束
        fork
            begin @(negedge uart_tx); end
            begin
                #(BIT_NS*3);
                if (uart_tx) disable recv_done;
            end
        join_any
        disable fork;
        if (uart_tx && rx_cnt >= 1) begin
            // 线路空闲，检查是否已收到完整消息
            disable recv_done;
        end
        #(BIT_NS/2);
    end
    begin : recv_done
    end

    // 打印收到的消息
    $write("RX bytes: ");
    for (i = 0; i < rx_cnt; i = i + 1) begin
        if (rx_buf[i] >= 8'h20 && rx_buf[i] < 8'h7f)
            $write("%c", rx_buf[i]);
        else
            $write("[%02x]", rx_buf[i]);
    end
    $write("\n");

    // 校验：期望 "ERR:0x31"
    // E=0x45 R=0x52 R=0x52 :=0x3A 0=0x30 x=0x78 3=0x33 1=0x31
    if (rx_cnt >= 8 &&
        rx_buf[0] == "E" && rx_buf[1] == "R" && rx_buf[2] == "R" &&
        rx_buf[3] == ":" && rx_buf[4] == "0" && rx_buf[5] == "x" &&
        rx_buf[6] == "3" && rx_buf[7] == "1")
        $display("PASS: ERR:0x31 输出正确");
    else
        $display("FAIL: 期望 ERR:0x31, 实际见上");

    $finish;
end
endmodule
