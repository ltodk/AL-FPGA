`timescale 1ns/1ps
module tb_uart_edges;
reg clk = 0;
reg rst = 1;
reg sd_init_done = 0;
always #5 clk = ~clk;
wire uart_tx;
uart_logger u_log (
    .clk(clk), .rst(rst),
    .sd_init_done(sd_init_done),
    .img_idx(2'd0), .img_total(3'd4),
    .load_error(1'b0), .bmp_err_code(3'd0),
    .uart_tx(uart_tx)
);
// 记录 uart_tx 每次跳变的时间与字符索引
reg prev;
integer edges;
always @(posedge clk) begin
    prev <= uart_tx;
    if (prev !== uart_tx)
        $display("edge t=%0tps val=%b char_idx=%0d fsm=%0d tx_start=%b", $time, uart_tx, u_log.char_idx, u_log.fsm, u_log.tx_start);
end
initial begin
    rst = 1;
    repeat (10) @(posedge clk);
    rst = 0;
    #1000;
    sd_init_done = 1;
    #300000000;
    $finish;
end
endmodule
