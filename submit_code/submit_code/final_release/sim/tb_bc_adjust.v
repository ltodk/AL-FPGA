`timescale 1ns / 1ps
// =====================================================================
// tb_bc_adjust —— 亮度/对比度模块仿真（第 14 讲）
//
//   验证重点（对应讲义 3.2.3 上板自检清单）：
//     T1  brightness=255 / contrast=128 → **逐位旁路**（回归基准）
//     T2  亮度 128 → 约一半
//     T3  亮度 0   → 几乎全黑（1/256，不是纯 0 —— 这是设计取舍，见注释）
//     T4  contrast=0 → 全屏趋向中灰 128
//     T5  contrast=255 → 拉伸约 2 倍（亮更亮、暗更暗）
//     T6  饱和截断：超 255 截到 255，负值截到 0，**不能回绕**
//     T7  算术右移 >>>7 对负数的处理（用逻辑右移会得到大正数，画面崩）
//     T8  de 延迟 2 拍对齐：out_de/out_x 与像素同步
//     T9  帧边界锁存：改了参数但没来 vs 上升沿 → 生效值不变
//
//   期望值算法与 RTL **独立实现**：TB 用数学 floor 除法，
//   RTL 用算术右移，两者在负数上必须一致（>>> 就是 floor）。
// =====================================================================
module tb_bc_adjust;

    reg clk = 1'b0;
    always #20 clk = ~clk;          // 25MHz，周期 40ns

    reg        rst_n = 1'b0;
    reg        de    = 1'b0;
    reg        vs    = 1'b0;
    reg [9:0]  vx    = 10'd0;
    reg [23:0] rgb   = 24'h000000;
    reg [7:0]  br    = 8'd255;
    reg [7:0]  ct    = 8'd128;

    wire        out_de, out_vs;
    wire [9:0]  out_x;
    wire [23:0] rgb_out;
    wire [7:0]  beff, ceff;

    brightness_contrast_adjust dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .de           (de),
        .vs           (vs),
        .video_x      (vx),
        .rgb_in       (rgb),
        .brightness   (br),
        .contrast     (ct),
        .out_de       (out_de),
        .out_vs       (out_vs),
        .out_x        (out_x),
        .rgb_out      (rgb_out),
        .bright_eff   (beff),
        .contrast_eff (ceff)
    );

    // ---------------- 独立参考模型 ----------------
    // 数学 floor 除法（对应 RTL 的 >>> 算术右移）
    function integer fdiv128;
        input integer v;
        begin
            if (v >= 0) fdiv128 = v / 128;
            else        fdiv128 = -((-v + 127) / 128);
        end
    endfunction

    function [7:0] sat8;
        input integer v;
        begin
            if (v < 0)      sat8 = 8'd0;
            else if (v >255) sat8 = 8'd255;
            else            sat8 = v[7:0];
        end
    endfunction

    // 单通道参考：先亮度后对比度
    function [7:0] ref_chan;
        input [7:0] x;
        input [7:0] b;
        input [7:0] c;
        integer y, z;
        begin
            y = (x * (b + 1)) / 256;      // 亮度（这里 y 一定是非负）
            z = fdiv128((y - 128) * c) + 128;
            ref_chan = sat8(z);
        end
    endfunction

    function [23:0] ref_rgb;
        input [23:0] x;
        input [7:0]  b;
        input [7:0]  c;
        begin
            ref_rgb = {ref_chan(x[23:16], b, c),
                       ref_chan(x[15:8],  b, c),
                       ref_chan(x[7:0],   b, c)};
        end
    endfunction

    // ---------------- 统计 ----------------
    integer n_pass = 0, n_fail = 0;

    task check;
        input [8*48-1:0] name;
        input [23:0] got;
        input [23:0] exp;
        begin
            if (got === exp) begin
                n_pass = n_pass + 1;
                $display("  [ OK ] %0s  得到 %06h (=期望)", name, got);
            end else begin
                n_fail = n_fail + 1;
                $display("  [FAIL] %0s  得到 %06h  期望 %06h", name, got, exp);
            end
        end
    endtask

    // ---------------- 参数设置（含 vs 帧边界锁存）----------------
    task set_param;
        input [7:0] b;
        input [7:0] c;
        begin
            br = b;  ct = c;
            repeat (4) @(posedge clk);   // 等 2FF 同步稳定
            vs = 1'b1; @(posedge clk);   // vs 上升沿 → 帧边界锁存
            vs = 1'b0; @(posedge clk);
            repeat (2) @(posedge clk);
        end
    endtask

    // ---------------- 单像素测量 ----------------
    task measure;
        input  [23:0] din;
        output [23:0] dout;
        begin
            de = 1'b1;  rgb = din;
            repeat (4) @(posedge clk);   // 2 拍流水 + 稳定
            dout = rgb_out;
            de = 1'b0;
            @(posedge clk);
        end
    endtask

    reg [23:0] got;

    // ---------------- 主流程 ----------------
    initial begin
        $display("\n===== 第14讲 亮度/对比度模块仿真 =====");

        rst_n = 1'b0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // ---------------- T1：严格旁路 ----------------
        $display("\n[T1] brightness=255, contrast=128 → 必须逐位旁路");
        set_param(8'd255, 8'd128);
        $display("     生效值: B=%0d C=%0d", beff, ceff);
        begin : t1
            reg [23:0] pat [0:5];
            integer k, bad;
            pat[0] = 24'h000000;  pat[1] = 24'hFFFFFF;
            pat[2] = 24'hFF8000;  pat[3] = 24'h123456;
            pat[4] = 24'h00FF7F;  pat[5] = 24'hABCDEF;
            bad = 0;
            for (k = 0; k < 6; k = k + 1) begin
                measure(pat[k], got);
                if (got !== pat[k]) begin
                    n_fail = n_fail + 1;
                    bad = bad + 1;
                    $display("  [FAIL] 旁路 %06h -> %06h", pat[k], got);
                end else begin
                    n_pass = n_pass + 1;
                end
            end
            if (bad == 0)
                $display("  [ OK ] 6 组图案旁路全部逐位一致");
            else
                $display("  [FAIL] 6 组中有 %0d 组旁路不一致", bad);
        end

        // ---------------- T2：亮度 128 ----------------
        $display("\n[T2] brightness=128 → 约一半亮度");
        set_param(8'd128, 8'd128);
        measure(24'hFFFFFF, got);
        check("亮128: FF -> 约 80", got, ref_rgb(24'hFFFFFF, 8'd128, 8'd128));
        $display("     实测 %02h（期望 255*129/256 = %0d）", got[7:0], (255*129)/256);

        // ---------------- T3：亮度 0 ----------------
        $display("\n[T3] brightness=0 → 接近全黑（1/256，非纯 0）");
        set_param(8'd0, 8'd128);
        measure(24'hFFFFFF, got);
        check("亮0: FF -> 约 00", got, ref_rgb(24'hFFFFFF, 8'd0, 8'd128));
        $display("     实测 %02h（255*1/256 = 0，设计上就是 0）", got[7:0]);

        // ---------------- T4：对比度 0 ----------------
        $display("\n[T4] contrast=0 → 所有像素趋向中灰 128");
        set_param(8'd255, 8'd0);
        measure(24'hFFFFFF, got);
        check("对比0: FF -> 80", got, 24'h808080);
        measure(24'h000000, got);
        check("对比0: 00 -> 80", got, 24'h808080);
        measure(24'hFF0000, got);
        check("对比0: FF0000 -> 808080", got, 24'h808080);

        // ---------------- T5：对比度 255 ----------------
        $display("\n[T5] contrast=255 → 拉伸约 2 倍");
        set_param(8'd255, 8'd255);
        measure(24'hC8C8C8, got);        // 200
        check("对比255: C8(200) 提亮", got, ref_rgb(24'hC8C8C8, 8'd255, 8'd255));
        $display("     200 -> %0d （原 200，向上拉伸）", got[7:0]);
        measure(24'h323232, got);        // 50
        check("对比255: 32(50) 压暗", got, ref_rgb(24'h323232, 8'd255, 8'd255));
        $display("     50  -> %0d （原 50，向下压缩）", got[7:0]);

        // ---------------- T6：饱和截断 ----------------
        $display("\n[T6] 饱和截断：超上限截 255、负值截 0，绝不能回绕");
        set_param(8'd255, 8'd255);
        measure(24'hFFFFFF, got);
        check("饱和: FF 不溢出（不是 7D/回绕）", got, 24'hFFFFFF);
        measure(24'h000000, got);
        check("饱和: 00 不为负（不是 81/回绕）", got, 24'h000000);

        // ---------------- T7：负数算术右移 ----------------
        $display("\n[T7] 算术右移 >>>7：负数必须 floor，不能变正");
        // y=100, c=255 -> (100-128)*255 = -7140, floor(-7140/128) = -56, +128 = 72
        set_param(8'd255, 8'd255);
        measure(24'h646464, got);        // 100
        check("负数右移: 64(100) -> 48(72)", got, 24'h484848);
        $display("     100 -> %0d （(100-128)*255>>>7 = -56, +128 = 72）", got[7:0]);

        // ---------------- T9：帧边界锁存 ----------------
        $display("\n[T9] 帧边界锁存：只改输入、不给 vs 沿 → 生效值不变");
        set_param(8'd255, 8'd128);       // 回到基线
        br = 8'd64;  ct = 8'd200;        // 只改输入，不发 vs
        repeat (6) @(posedge clk);
        $display("     输入 B=%0d C=%0d，生效 B=%0d C=%0d",
                 br, ct, beff, ceff);
        if (beff === 8'd255 && ceff === 8'd128) begin
            n_pass = n_pass + 1;
            $display("  [ OK ] 未到帧边界，生效值保持 255/128");
        end else begin
            n_fail = n_fail + 1;
            $display("  [FAIL] 参数在帧中间就生效了（会出半帧分界线）");
        end
        // 补一个 vs 沿，确认能生效
        vs = 1'b1; @(posedge clk); vs = 1'b0; @(posedge clk);
        repeat (2) @(posedge clk);
        $display("     来 vs 沿后：生效 B=%0d C=%0d", beff, ceff);
        if (beff === 8'd64 && ceff === 8'd200) begin
            n_pass = n_pass + 1;
            $display("  [ OK ] 帧边界到达后新参数生效");
        end else begin
            n_fail = n_fail + 1;
            $display("  [FAIL] 帧边界后参数未更新");
        end

        // ---------------- T8：延迟对齐 ----------------
        $display("\n[T8] de/vs/x 与像素同步延迟 2 拍");
        set_param(8'd255, 8'd128);
        de = 1'b1;  vx = 10'd123;  rgb = 24'h808080;
        repeat (1) @(posedge clk);
        // 第 1 拍：out_de 仍为 0（还没传过来）
        @(posedge clk);
        // 第 2 拍：out_de 应为 1，out_x 应为 123
        @(posedge clk);
        if (out_de === 1'b1 && out_x === 10'd123) begin
            n_pass = n_pass + 1;
            $display("  [ OK ] out_de 有效时 out_x=%0d（与像素同拍）", out_x);
        end else begin
            n_fail = n_fail + 1;
            $display("  [FAIL] out_de=%b out_x=%0d（应为 1 / 123）", out_de, out_x);
        end
        de = 1'b0;
        repeat (2) @(posedge clk);

        // ---------------- 汇总 ----------------
        $display("\n========================================");
        $display(" 通过 %0d 项，失败 %0d 项", n_pass, n_fail);
        if (n_fail == 0) $display(" RESULT: PASS");
        else             $display(" RESULT: FAIL");
        $display("========================================\n");
        $finish;
    end

    // 超时保护
    initial begin
        #2000000;
        $display(" TIMEOUT");
        $finish;
    end

endmodule
