// tb_uart_diag_once: 验证 fat_dbg_pulse / scan_diag_pulse 边沿检测只触发一次（不刷屏）
// 关键回归点：uart_logger 此前漏更新 fat_dbg_pulse_d / scan_diag_pulse_d，
// 导致 pulse 保持高电平时 fat_dbg_rise/scan_diag_rise 恒为 1 → 消息无限重发刷屏。
`timescale 1ns/1ps
module tb_uart_diag_once;
reg clk = 0;
reg rst = 1;
reg sd_init_done = 0;
reg fat_dbg_pulse = 0;
reg fat_dbg_ok = 0;
reg [31:0] fat_dbg_val = 0;
reg scan_diag_pulse = 0;
reg scan_diag_zero = 0;
reg [31:0] scan_diag_val = 0;
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
    .ld3s_pulse(1'b0), .ld3s_src(1'b0),
    .fat_dbg_pulse(fat_dbg_pulse),
    .fat_dbg_ok(fat_dbg_ok),
    .fat_dbg_val(fat_dbg_val),
    .scan_diag_pulse(scan_diag_pulse),
    .scan_diag_zero(scan_diag_zero),
    .scan_diag_val(scan_diag_val),
    .uart_tx(uart_tx)
);

localparam integer BIT_NS = 8680;  // 1e9/115200
integer k;
reg [7:0] rx_byte;
integer msg_count;

initial begin
    msg_count = 0;
    rst = 1;
    repeat (10) @(posedge clk);
    rst = 0;
    #1000;

    // 后台持续采样器：从此刻起，只要 uart_tx 有下降沿就收字节，遇 '\n' 计数
    fork
        begin : bg_sampler
            forever begin
                @(negedge uart_tx);
                #(BIT_NS/2);
                for (k = 0; k < 8; k = k + 1) begin
                    #(BIT_NS);
                    rx_byte[k] = uart_tx;
                end
                #(BIT_NS/2);
                if (rx_byte == 8'h0A) begin
                    msg_count = msg_count + 1;
                    $display("  [msg %0d] 收到一条完整消息 (last_byte=0x%02x)", msg_count, rx_byte);
                end
            end
        end
        begin : main
            // ===== 场景1：fat_dbg_pulse 拉高并保持 =====
            $display("== 场景1: fat_dbg_pulse 保持高电平 200 拍 ==");
            fat_dbg_ok = 1; fat_dbg_val = 32'h00004000;
            fat_dbg_pulse = 1;
            repeat (200) @(posedge clk);
            fat_dbg_pulse = 0;
            // 等足够长时间（一条消息约1.2ms），观察是否重复发送
            #(2000000);   // 2ms

            // ===== 场景2：scan_diag_pulse 拉高并保持 =====
            $display("== 场景2: scan_diag_pulse 保持高电平 200 拍 ==");
            scan_diag_zero = 0; scan_diag_val = 32'h00005040;
            scan_diag_pulse = 1;
            repeat (200) @(posedge clk);
            scan_diag_pulse = 0;
            #(2000000);   // 2ms

            // 关采样器，出结果
            disable bg_sampler;
            #1000;
            $display("==== 总消息数 = %0d (预期=2：DS 一条 + S1 一条) ====", msg_count);
            if (msg_count == 2) begin
                $display("PASS: 诊断消息各只发一次，无刷屏");
            end else begin
                $display("FAIL: 消息数 = %0d，存在刷屏/漏发", msg_count);
            end
            $finish;
        end
    join
end

endmodule
