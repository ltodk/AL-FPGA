`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_fade_apply —— fade_alpha_apply（淡到灰 / fade through grey）验证
//
//   理论式：y = (x * g + FADE_GRAY * (256 - g)) >> 8 ,  g = alpha + 1
//
// 核心断言：
//   A1 alpha=255 逐位旁路（转场结束后画面与原图完全一致，回归基线不能变）
//   A2 alpha=0 输出灰阶而不是纯黑（这是本次改动的目的，必须守住）
//   A3 全 alpha 扫描 × 3 通道与理论值逐点一致
//   A4 x=GRAY 是不动点：任何 alpha 下输出恒为 GRAY（公式自检）
//   A5 alpha 递增时输出向 x 单调收敛（亮图不减、暗图不增，无来回跳）
//   A6 alpha=0 时画面自然去色（三通道趋于相等）
//   A7 最糊档整屏灰度跨度很小 → 换 bank 发生在这一帧，新图藏得住
//   A8 三通道互不串扰
// -----------------------------------------------------------------------------
module tb_fade_apply;

    // 与顶层 FADE_GRAY_CFG / MIN_ALPHA_CFG 保持一致
    localparam integer FADE_GRAY   = 128;
    localparam integer MIN_ALPHA   = 16;
    localparam integer FADE_FRAMES = 8;
    localparam integer STEP        = (256 - MIN_ALPHA) / FADE_FRAMES;
    localparam [7:0]   GRAY_C      = FADE_GRAY;

    reg  [7:0]  alpha;
    reg  [23:0] rgb_in;
    wire [23:0] rgb_out;

    fade_alpha_apply #(
        .FADE_GRAY (FADE_GRAY)
    ) dut (
        .alpha   (alpha),
        .rgb_in  (rgb_in),
        .rgb_out (rgb_out)
    );

    integer n_pass = 0;
    integer n_fail = 0;

    task check;
        input cond;
        input [8*110-1:0] msg;
        begin
            if (cond) begin
                n_pass = n_pass + 1;
                $display("  [ OK ] %0s", msg);
            end else begin
                n_fail = n_fail + 1;
                $display("  [FAIL] %0s", msg);
            end
        end
    endtask

    // 期望值：floor((x*(a+1) + GRAY*(255-a)) / 256)
    function [7:0] expect_ch;
        input [7:0] x;
        input [7:0] a;
        reg [8:0]  g, ig;
        reg [16:0] p, q;
        reg [17:0] s;
        begin
            g  = {1'b0, a} + 9'd1;
            ig = 9'd256 - g;
            p  = {9'd0, x} * g;
            q  = {9'd0, GRAY_C} * ig;
            s  = {1'b0, p} + {1'b0, q};
            expect_ch = s[15:8];
        end
    endfunction

    integer i, k;
    reg [23:0] pat [0:7];
    reg [23:0] exp;
    integer    mism;
    integer    lo, hi;
    integer    prev;

    initial begin
        $display("");
        $display("========== 第10讲 淡到灰（fade through grey）验证 ==========");
        $display("  FADE_GRAY=%0d  MIN_ALPHA=%0d  STEP=%0d", FADE_GRAY, MIN_ALPHA, STEP);

        pat[0] = 24'h000000; pat[1] = 24'hFFFFFF;
        pat[2] = 24'hFF8000; pat[3] = 24'h123456;
        pat[4] = 24'h00FF7F; pat[5] = 24'hABCDEF;
        pat[6] = 24'h7F7F7F; pat[7] = 24'h010203;

        // ---------------- A1：alpha=255 逐位精确 ----------------
        $display("\n[A1] alpha=255 必须逐位等于输入");
        alpha = 8'd255;
        mism  = 0;
        for (k = 0; k < 8; k = k + 1) begin
            rgb_in = pat[k]; #1;
            if (rgb_out !== pat[k]) begin
                mism = mism + 1;
                $display("  [FAIL] %06h -> %06h", pat[k], rgb_out);
            end
        end
        check(mism == 0, "8 组图案 alpha=255 全部逐位一致");

        // ---------------- A2：alpha=0 输出灰阶 ----------------
        $display("\n[A2] alpha=0 输出灰阶（不是纯黑）");
        alpha = 8'd0;
        mism  = 0;
        for (k = 0; k < 8; k = k + 1) begin
            rgb_in = pat[k]; #1;
            exp = {expect_ch(pat[k][23:16], 8'd0),
                   expect_ch(pat[k][15:8],  8'd0),
                   expect_ch(pat[k][7:0],   8'd0)};
            if (rgb_out !== exp) begin
                mism = mism + 1;
                $display("  [FAIL] %06h -> %06h  期望 %06h", pat[k], rgb_out, exp);
            end
        end
        check(mism == 0, "alpha=0 时 8 组图案全部等于理论灰度");

        // 每个通道都必须贴着 FADE_GRAY（floor 误差 <=1）
        alpha = 8'd0;
        mism  = 0;
        for (k = 0; k < 8; k = k + 1) begin
            rgb_in = pat[k]; #1;
            if (rgb_out[23:16] < FADE_GRAY-1 || rgb_out[23:16] > FADE_GRAY)
                mism = mism + 1;
            if (rgb_out[15:8]  < FADE_GRAY-1 || rgb_out[15:8]  > FADE_GRAY)
                mism = mism + 1;
            if (rgb_out[7:0]   < FADE_GRAY-1 || rgb_out[7:0]   > FADE_GRAY)
                mism = mism + 1;
        end
        check(mism == 0, "alpha=0 时三通道全部落在 FADE_GRAY 上（误差<=1）");

        // ---------------- A3：全 alpha 扫描 ----------------
        $display("\n[A3] 遍历 alpha=0..255 × 3 通道逐点比对");
        mism = 0;
        for (i = 0; i < 256; i = i + 1) begin
            alpha  = i[7:0];
            rgb_in = 24'hFF8040;
            #1;
            exp = {expect_ch(8'hFF, i[7:0]),
                   expect_ch(8'h80, i[7:0]),
                   expect_ch(8'h40, i[7:0])};
            if (rgb_out !== exp) begin
                if (mism < 5)
                    $display("  [FAIL] alpha=%0d  得 %06h  期望 %06h", i, rgb_out, exp);
                mism = mism + 1;
            end
        end
        check(mism == 0, "256 档 x 3 通道全部等于理论加权平均");

        // ---------------- A4：x=GRAY 是不动点 ----------------
        $display("\n[A4] x=FADE_GRAY 是不动点（任何 alpha 都输出 FADE_GRAY）");
        mism = 0;
        for (i = 0; i < 256; i = i + 1) begin
            alpha  = i[7:0];
            rgb_in = {GRAY_C, GRAY_C, GRAY_C};
            #1;
            if (rgb_out !== {GRAY_C, GRAY_C, GRAY_C}) mism = mism + 1;
        end
        check(mism == 0, "输入恰为灰阶时 256 档 alpha 输出全部不变");

        // ---------------- A5：单调收敛 ----------------
        $display("\n[A5] alpha 递增时输出向原值单调收敛");
        // 亮图（255 > GRAY）：必须单调不减
        begin : chk_up
            integer bad;
            bad = 0; prev = 0;
            for (i = 0; i < 256; i = i + 1) begin
                alpha = i[7:0]; rgb_in = 24'hFFFFFF; #1;
                if (i > 0 && (rgb_out[23:16] < prev)) begin
                    bad = bad + 1;
                    if (bad <= 3)
                        $display("       回退：alpha=%0d %0d->%0d", i, prev, rgb_out[23:16]);
                end
                prev = rgb_out[23:16];
            end
            check(bad == 0, "亮图 alpha 递增时输出单调不减");
        end
        // 暗图（0 < GRAY）：必须单调不增
        begin : chk_dn
            integer bad;
            bad = 0; prev = 255;
            for (i = 0; i < 256; i = i + 1) begin
                alpha = i[7:0]; rgb_in = 24'h000000; #1;
                if (i > 0 && (rgb_out[23:16] > prev)) begin
                    bad = bad + 1;
                    if (bad <= 3)
                        $display("       回升：alpha=%0d %0d->%0d", i, prev, rgb_out[23:16]);
                end
                prev = rgb_out[23:16];
            end
            check(bad == 0, "暗图 alpha 递增时输出单调不增");
        end

        // ---------------- A6：最糊档去色 ----------------
        $display("\n[A6] alpha=0 时画面自然去色（三通道趋于相等）");
        alpha = 8'd0; rgb_in = 24'hFF0000; #1;
        lo = rgb_out[23:16];
        hi = rgb_out[23:16];
        if (rgb_out[15:8] < lo) lo = rgb_out[15:8];
        if (rgb_out[15:8] > hi) hi = rgb_out[15:8];
        if (rgb_out[7:0]  < lo) lo = rgb_out[7:0];
        if (rgb_out[7:0]  > hi) hi = rgb_out[7:0];
        $display("     饱和红 %06h -> %06h（跨度 %0d）", 24'hFF0000, rgb_out, hi-lo);
        check((hi - lo) <= 2, "纯红在 alpha=0 时三通道跨度<=2（已去色）");

        // ---------------- A7：最糊档整屏灰度跨度 ----------------
        $display("\n[A7] 最糊档（alpha=MIN_ALPHA）整屏灰度跨度");
        alpha = MIN_ALPHA[7:0];
        lo = 255; hi = 0;
        for (k = 0; k < 8; k = k + 1) begin
            rgb_in = pat[k]; #1;
            if (rgb_out[23:16] < lo) lo = rgb_out[23:16];
            if (rgb_out[23:16] > hi) hi = rgb_out[23:16];
        end
        $display("     alpha=%0d 时最暗 %0d / 最亮 %0d，跨度 %0d",
                 MIN_ALPHA, lo, hi, hi-lo);
        check((hi - lo) <= 32, "最糊档整屏灰度跨度<=32（近乎纯灰，换图不可见）");

        // ---------------- A8：通道独立 ----------------
        $display("\n[A8] 三通道互不串扰");
        alpha  = 8'd128;
        rgb_in = 24'hFF0000; #1;
        check(rgb_out[23:16] === expect_ch(8'hFF, 8'd128) &&
              rgb_out[15:8]  === rgb_out[7:0],
              "只有 R 有输入时 G/B 相同且与 R 不同");

        alpha  = 8'd200;
        rgb_in = 24'h00FF00; #1;
        check(rgb_out[15:8] === expect_ch(8'hFF, 8'd200) &&
              rgb_out[23:16] === rgb_out[7:0],
              "只有 G 有输入时 R/B 相同且与 G 不同");

        // 关键点打印
        $display("\n     关键点（白 255 / 黑 0 / 中灰 128）：");
        alpha = 8'd255; rgb_in = 24'hFFFFFF; #1;
        $display("     alpha=255  255 -> %0d  (逐位原图)", rgb_out[23:16]);
        alpha = MIN_ALPHA[7:0]; rgb_in = 24'hFFFFFF; #1;
        $display("     alpha=%0d   255 -> %0d  (最糊档亮部)", MIN_ALPHA, rgb_out[23:16]);
        alpha = MIN_ALPHA[7:0]; rgb_in = 24'h000000; #1;
        $display("     alpha=%0d     0 -> %0d  (最糊档暗部)", MIN_ALPHA, rgb_out[23:16]);
        alpha = 8'd0; rgb_in = 24'hFFFFFF; #1;
        $display("     alpha=0    255 -> %0d  (完全糊成灰)", rgb_out[23:16]);

        $display("");
        $display("------------------------------------------");
        if (n_fail == 0) $display("RESULT: PASS   (%0d 项检查全部通过)", n_pass);
        else             $display("RESULT: FAIL   (通过 %0d, 失败 %0d)", n_pass, n_fail);
        $display("------------------------------------------");
        $finish;
    end

endmodule
