// =============================================================================
// tb_key_ack.v —— 矩阵键盘"应急警报人工解除键"解码验证
//
//   被测模块：key_matrix_4x4（在 top_tf_hdmi_audio.v 末尾）
//   验证点：
//     ① key_pressed[13]（丝印 KEY9，行 3 列 1）按下 -> o_key_ack_n 拉低
//     ② 同一时刻其它输出（next/auto/prev/spare/param）不受影响
//     ③ 松开后 o_key_ack_n 回到 1
//     ④ 按 key_pressed[0]（丝印 KEY12）时 o_key_ack_n 必须保持 1（不能误触发）
//     ⑤ 按 key_pressed[3]（丝印 KEY15）时只有 o_key_spare_n 动、ack 不动
//
//   键盘模型：模块把当前扫描行拉低（key_row 低有效），tb 在"该行被拉低"时
//   把对应列的 key_col 位拉低来模拟按下；列读到低 = 按下。
//   扫描节拍 SCAN_DIV=5000 @50MHz = 100us/行，16 键一轮 400us，所以每个动作
//   要给定足够长的保持时间（这里每次按 3ms，足够扫过 7 轮）。
// =============================================================================
`timescale 1ns/1ps

module tb_key_ack;

reg  clk = 0;
reg  rst = 1;
always #10 clk = ~clk;          // 50 MHz

wire [3:0] key_row;
reg  [3:0] key_col = 4'hF;

wire o_next, o_auto, o_prev, o_spare, o_ack, o_panic;
wire [3:0] o_param;

key_matrix_4x4 u_dut (
    .clk          (clk),
    .rst          (rst),
    .key_row      (key_row),
    .key_col      (key_col),
    .o_key_next_n (o_next),
    .o_key_auto_n (o_auto),
    .o_key_prev_n (o_prev),
    .o_key_spare_n(o_spare),
    .o_key_ack_n  (o_ack),
    .o_key_panic_n(o_panic),
    .o_key_param_n(o_param)
);

// ---- 键盘模型：hold_idx >= 0 时，扫描到对应行就把对应列拉低 ----
//   ★ 丝印 -> key_pressed 的实测规律（2026-10-31 仿真 + 板上三条独立证据）：
//        bit = (丝印编号 + 4) mod 16
//     证据① KEY0..3 → bits 4..7（亮度/对比度，板上实测）
//     证据② KEY8    → bit 12   （旧文档记的"写后静默期演示键"掩码）
//     证据③ KEY12..15 → bits 0..3（丝印 KEY13 显示 ERR:F8 = bit3 的 mask 命中）
//     本次仿真复现了这条规律：只驱动矩阵 row/col，DUT 解出的 bit 恰好比"矩阵序号"大 4。
//   所以 press(k) 的参数是**丝印编号**：内部先换算成矩阵行列再驱动。
integer hold_idx = -1;          // -1 = 没有键按下
integer hold_row, hold_col;
integer key_bit;

function integer silk2bit(input integer k);
    silk2bit = (k + 4) % 16;
endfunction

always @(*) begin
    key_col = 4'hF;
    if (hold_idx >= 0) begin
        // 该行被拉低时（key_row[hold_row] == 0）才把列拉低
        if (key_row[hold_row] == 1'b0)
            key_col = ~(4'b0001 << hold_col);
    end
end

// 按丝印编号按键：K = 0..15
//   本 tb 在**矩阵层**模拟：丝印第 K/4 行、第 K%4 列。
//   DUT 采样有一拍行延迟（列在上一行末尾被锁存），因此解出的 bit = (K+4) mod 16，
//   这与板上实测完全一致（KEY0..3→4..7、KEY8→12、KEY12..15→0..3）。此偏差是良性的，
//   面板走线与文档掩码都是按实测标定的，**不要"修"它**。
task press(input integer K, input integer ms);
    begin
        hold_idx = K;
        hold_row = K / 4;
        hold_col = K % 4;
        $display("  [press] silkscreen KEY%0d -> drive matrix row %0d col %0d -> expect bit %0d",
                 K, hold_row, hold_col, silk2bit(K));
        repeat (ms * 50000) @(posedge clk);   // 1ms @50MHz = 50000 拍
    end
endtask

task unpress(input integer ms);
    begin
        hold_idx = -1;
        repeat (ms * 50000) @(posedge clk);
    end
endtask

integer errors = 0;

// 关键观测：DUT 内部解码出来的 16 位键值（哪个 bit 置 1 = 哪一颗键被认到）
always @(u_dut.key_pressed)
    $display("  [%0t] key_pressed = %b", $time, u_dut.key_pressed);

task chk(input cond, input [255:0] msg);
    begin
        if (!cond) begin
            errors = errors + 1;
            $display("  FAIL: %0s", msg);
        end
    end
endtask

initial begin
    $display("=== tb_key_ack start ===");
    repeat (20) @(posedge clk);
    rst = 0;
    unpress(1);

    // ① 基态：所有输出都是高（未按）
    chk(o_ack === 1'b1, "idle: ack should be 1");
    $display("  idle    : next=%b auto=%b prev=%b spare=%b ack=%b param=%b",
             o_next, o_auto, o_prev, o_spare, o_ack, o_param);

    // ② 按丝印 KEY9 -> bit13 -> 只有 ack 变低
    press(9, 3);
    $display("  KEY9    : next=%b auto=%b prev=%b spare=%b ack=%b param=%b",
             o_next, o_auto, o_prev, o_spare, o_ack, o_param);
    chk(o_ack   === 1'b0, "KEY9: ack should be 0");
    chk(o_next  === 1'b1, "KEY9: next must stay 1");
    chk(o_auto  === 1'b1, "KEY9: auto must stay 1");
    chk(o_prev  === 1'b1, "KEY9: prev must stay 1");
    chk(o_spare === 1'b1, "KEY9: spare must stay 1");
    chk(o_param === 4'hF, "KEY9: param must stay F");

    // ③ 松开
    unpress(2);
    chk(o_ack === 1'b1, "release: ack should be 1");

    // ④ 按丝印 KEY12 -> bit0 -> next 变低、ack 不许动
    press(12, 3);
    $display("  KEY12   : ack=%b next=%b", o_ack, o_next);
    chk(o_ack  === 1'b1, "KEY12: ack must stay 1");
    chk(o_next === 1'b0, "KEY12: next should be 0");
    unpress(2);

    // ⑤ 按丝印 KEY15 -> bit3 -> 只有 spare 动
    press(15, 3);
    $display("  KEY15   : ack=%b spare=%b", o_ack, o_spare);
    chk(o_ack   === 1'b1, "KEY15: ack must stay 1");
    chk(o_spare === 1'b0, "KEY15: spare should be 0");
    unpress(2);

    // ⑥ 再按一次 KEY9，确认可重复触发
    press(9, 3);
    chk(o_ack === 1'b0, "KEY9 again: ack should be 0");
    unpress(2);

    // ⑦ 按丝印 KEY10 -> bit14 -> 只有 panic 变低（应急插播键）
    press(10, 3);
    $display("  KEY10   : ack=%b panic=%b next=%b spare=%b", o_ack, o_panic, o_next, o_spare);
    chk(o_panic === 1'b0, "KEY10: panic should be 0");
    chk(o_ack   === 1'b1, "KEY10: ack must stay 1");
    chk(o_next  === 1'b1, "KEY10: next must stay 1");
    chk(o_spare === 1'b1, "KEY10: spare must stay 1");
    unpress(2);

    // ⑧ 按丝印 KEY11 -> bit15 -> 必须是"不响应"（2026-10-31 起 KEY11 空闲，
    //    插播只留 KEY10 一颗键；这条断言就是把"KEY11 已摘掉"钉死）
    press(11, 3);
    $display("  KEY11   : panic=%b (expect 1 = idle)", o_panic);
    chk(o_panic === 1'b1, "KEY11 must be idle now (panic stays 1)");
    unpress(2);
    chk(o_panic === 1'b1, "release: panic should be 1");

    if (errors == 0) $display("RESULT: PASS");
    else             $display("RESULT: FAIL (errors=%0d)", errors);
    $finish;
end

endmodule
