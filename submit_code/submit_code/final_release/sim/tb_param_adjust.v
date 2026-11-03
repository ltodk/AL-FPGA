`timescale 1ns / 1ps
// =====================================================================
// tb_param_adjust —— 亮度/对比度按键控制器仿真（第 14 讲）
//
//   为了仿真速度，把 CLKS_PER_MS 压到 10（真实是 50000），
//   于是 1ms = 10 拍：20ms 消抖 = 200 拍，40ms 连发延迟 = 400 拍，
//   连发间隔 8ms = 80 拍，显示窗口 100ms = 1000 拍。
//   时序比例与真实参数完全一致，只是整体加速 —— 逻辑验证等价。
//
//   T1  按下即步进一次（短按响应，不等连发延迟）
//   T2  消抖：抖动 < 20ms 不触发
//   T3  连续短按到 0 后饱和，不回绕
//   T4  长按连发：按住期间按 80 拍节奏持续步进
//   T5  show_en 在调节后拉高，窗口结束自动回落
// =====================================================================
module tb_param_adjust;

    reg clk = 1'b0;
    always #10 clk = ~clk;          // 50MHz

    reg rst_n = 1'b0;
    reg kb_i = 1'b1, kb_d = 1'b1, kc_i = 1'b1, kc_d = 1'b1;

    wire [7:0] br, ct;
    wire       show_en, any_pulse;

    param_adjust_ctrl #(
        .CLKS_PER_MS     (10),
        .DEBOUNCE_MS     (20),
        .REPEAT_DELAY_MS (40),
        .REPEAT_RATE_MS  (8),
        .SHOW_MS         (100),
        .STEP            (16)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .key_b_inc_n  (kb_i),
        .key_b_dec_n  (kb_d),
        .key_c_inc_n  (kc_i),
        .key_c_dec_n  (kc_d),
        .brightness   (br),
        .contrast     (ct),
        .show_en      (show_en),
        .any_pulse    (any_pulse)
    );

    integer n_pass = 0, n_fail = 0;

    task check8;
        input [8*40-1:0] name;
        input [7:0] got;
        input [7:0] exp;
        begin
            if (got === exp) begin
                n_pass = n_pass + 1;
                $display("  [ OK ] %0s = %0d", name, got);
            end else begin
                n_fail = n_fail + 1;
                $display("  [FAIL] %0s = %0d  期望 %0d", name, got, exp);
            end
        end
    endtask

    // 按下 hold 拍、松开、再等消抖恢复
    task press;
        input integer hold_cyc;
        begin
            kb_d = 1'b0;
            repeat (hold_cyc) @(posedge clk);
            kb_d = 1'b1;
            repeat (260) @(posedge clk);     // 等松开消抖完 + 余量
        end
    endtask

    initial begin
        $display("\n===== 第14讲 参数调节控制器仿真 =====");

        rst_n = 1'b0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // ---------------- 初值 ----------------
        $display("\n[T0] 上电初值必须是旁路档（B=255 C=128）");
        check8("brightness", br, 8'd255);
        check8("contrast  ", ct, 8'd128);
        if (show_en === 1'b0) begin
            n_pass = n_pass + 1;
            $display("  [ OK ] show_en 初值 = 0（不显示参数）");
        end else begin
            n_fail = n_fail + 1;
            $display("  [FAIL] show_en 初值 = %b，应为 0", show_en);
        end

        // ---------------- T1：短按一次 ----------------
        $display("\n[T1] 短按亮度- 一次 → 255-16 = 239");
        press(300);
        check8("brightness", br, 8'd239);
        if (show_en === 1'b1) begin
            n_pass = n_pass + 1;
            $display("  [ OK ] 调节后 show_en=1（OSD 回显）");
        end else begin
            n_fail = n_fail + 1;
            $display("  [FAIL] 调节后 show_en 未拉高");
        end

        // ---------------- T2：抖动不触发 ----------------
        $display("\n[T2] 抖动（按住 100 拍 < 20ms 消抖）不应触发");
        kb_d = 1'b0; repeat (100) @(posedge clk);
        kb_d = 1'b1; repeat (300) @(posedge clk);
        check8("brightness", br, 8'd239);

        // ---------------- T3：连续按到 0 并饱和 ----------------
        $display("\n[T3] 连续短按 16 次 → 饱和到 0，不回绕");
        begin : t3
            integer i;
            for (i = 0; i < 16; i = i + 1) press(300);
        end
        check8("brightness", br, 8'd0);
        $display("     再按一次，确认不会回绕到 255");
        press(300);
        check8("brightness", br, 8'd0);

        // ---------------- T4：长按连发 ----------------
        $display("\n[T4] 长按亮度+ 800 拍 → 首次 1 次 + 连发约 5 次");
        kb_i = 1'b0;
        repeat (900) @(posedge clk);
        kb_i = 1'b1;
        repeat (260) @(posedge clk);
        $display("     按住 900 拍后 brightness = %0d", br);
        // 首次 1 次 + (900-400)/80 ≈ 6 次 = 7 次 → 0 + 7*16 = 112（±16 容差）
        if (br >= 8'd96 && br <= 8'd128) begin
            n_pass = n_pass + 1;
            $display("  [ OK ] 连发生效（%0d 落在 96~128 区间）", br);
        end else begin
            n_fail = n_fail + 1;
            $display("  [FAIL] 连发次数异常，brightness=%0d（应 96~128）", br);
        end

        // ---------------- T5：对比度独立 ----------------
        $display("\n[T5] 对比度键独立生效，不影响亮度");
        begin : t5
            reg [7:0] br_before;
            br_before = br;
            kc_i = 1'b0; repeat (300) @(posedge clk);
            kc_i = 1'b1; repeat (260) @(posedge clk);
            check8("contrast  ", ct, 8'd144);
            if (br === br_before) begin
                n_pass = n_pass + 1;
                $display("  [ OK ] 亮度未被影响 = %0d", br);
            end else begin
                n_fail = n_fail + 1;
                $display("  [FAIL] 亮度被误改 %0d -> %0d", br_before, br);
            end
        end

        // ---------------- T6：显示窗口自动关闭 ----------------
        $display("\n[T6] 最后一次调节后 1000 拍，show_en 应回落");
        repeat (1200) @(posedge clk);
        if (show_en === 1'b0) begin
            n_pass = n_pass + 1;
            $display("  [ OK ] show_en 已自动关闭（3 秒窗口到期）");
        end else begin
            n_fail = n_fail + 1;
            $display("  [FAIL] show_en 仍为 %b，显示窗口没关", show_en);
        end

        $display("\n========================================");
        $display(" 通过 %0d 项，失败 %0d 项", n_pass, n_fail);
        if (n_fail == 0) $display(" RESULT: PASS");
        else             $display(" RESULT: FAIL");
        $display("========================================\n");
        $finish;
    end

    initial begin
        #3000000;
        $display(" TIMEOUT");
        $finish;
    end

endmodule
