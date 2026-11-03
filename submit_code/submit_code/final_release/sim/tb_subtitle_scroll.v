`timescale 1ns/1ps
// =====================================================================
// tb_subtitle_scroll.v : 第 9 讲滚动字幕层验证
//
// 缩小规模（H_ACTIVE=64, BAND_Y=4）以便快速跑完多个帧。
// 验证项：
//   T1 帧边界滚动：scroll_pos 每帧 +SCROLL_STEP，帧内保持恒定（无撕裂）
//   T2 从右滚入：初始帧屏幕无文字，滚大后文字出现
//   T3 Alpha 半透明底带：底带区域 rgb_out = rgb_in>>1（每通道）
//   T4 白字渲染：文字像素输出 24'hFFFFFF
//   T5 多条循环：滚满一个周期后 text_idx 自动切换到下一条
//   T6 关闭使能：enable=0 时完全透传
// =====================================================================

module tb_subtitle_scroll;

    localparam integer H_ACTIVE = 64;
    localparam integer BAND_Y   = 4;
    localparam integer BAND_H   = 31;     // 3 倍放大后底带高 = 23 + 2*4
    localparam integer SCROLL_STEP = 1;   // 与 DUT 默认一致
    localparam integer ROWS     = 40;     // 每帧有效行数（需覆盖 BAND_Y~BAND_Y+BAND_H）
    localparam integer CLK_HALF = 20;     // 25MHz -> 40ns

    reg         clk = 0;
    reg         rst = 1;
    reg         de  = 0;
    reg         vs  = 1'b1;
    reg  [9:0]  video_x = 0;
    reg  [23:0] video_in = 24'h808080;
    reg         enable = 1'b1;

    wire [23:0] video_out;
    wire        overlay_active;

    always #CLK_HALF clk = ~clk;

    subtitle_scroll_overlay #(
        .H_ACTIVE    (H_ACTIVE),
        .BAND_Y      (BAND_Y),
        .SCROLL_STEP (SCROLL_STEP)
    ) dut (
        .clk            (clk),
        .rst            (rst),
        .de             (de),
        .vs             (vs),
        .video_x        (video_x),
        .video_in       (video_in),
        .enable         (enable),
        .video_out      (video_out),
        .overlay_active (overlay_active)
    );

    integer errors = 0;
    integer frames = 0;
    integer row;

    // ---------------- 全局像素采样器 ----------------
    reg     sampling = 1'b0;
    integer g_white   = 0;   // 白字像素数
    integer g_band_ok = 0;   // 底带像素正确数
    integer g_band_bad= 0;   // 底带像素异常数
    integer g_min_x   = 9999;// 白字最左 x（用于验证向左滚动）

    // 监视器：de=1（行有效）期间 scroll_pos 若发生变化即为撕裂
    reg [12:0] sp_prev = 13'd0;
    integer    pos_change_in_de = 0;

    always @(posedge clk) begin
        if (sampling && de) begin
            if (video_out === 24'hFFFFFF) begin
                g_white = g_white + 1;
                if (video_x < g_min_x) g_min_x = video_x;
            end else if (dut.line_y >= BAND_Y[9:0] &&
                         dut.line_y <  BAND_Y[9:0] + BAND_H[9:0]) begin
                if (video_out === {1'b0, video_in[23:17],
                                   1'b0, video_in[15:9],
                                   1'b0, video_in[7:1]})
                    g_band_ok = g_band_ok + 1;
                else
                    g_band_bad = g_band_bad + 1;
            end
        end
        // 撕裂检测：有效行内滚动位置不得变化
        if (de && (dut.scroll_pos !== sp_prev))
            pos_change_in_de = pos_change_in_de + 1;
        sp_prev <= dut.scroll_pos;
    end

    task check;
        input         cond;
        input [255:0] name;
        begin
            if (!cond) begin
                errors = errors + 1;
                $display("  [FAIL] %0s", name);
            end else begin
                $display("  [ OK ] %0s", name);
            end
        end
    endtask

    // ---- 跑一帧：vs 脉冲 + ROWS 行（每行 H_ACTIVE 拍 de=1）----
    task run_frame;
        begin
            @(posedge clk); vs <= 1'b0;          // 场同步脉冲（低有效）
            @(posedge clk);
            @(posedge clk); vs <= 1'b1;
            @(posedge clk);

            for (row = 0; row < ROWS; row = row + 1) begin
                de <= 1'b0;
                @(posedge clk); @(posedge clk);   // 行消隐
                de <= 1'b1;
                for (video_x = 0; video_x < H_ACTIVE; video_x = video_x + 1)
                    @(posedge clk);
                de <= 1'b0;
                video_x <= 0;
                @(posedge clk);
            end
            frames = frames + 1;
        end
    endtask

    integer i;
    integer sp0, sp1, tx0;
    integer white_at_start;

    initial begin
        $display("=========================================================");
        $display(" 第9讲 滚动字幕层验证 (H_ACTIVE=%0d, BAND_Y=%0d)", H_ACTIVE, BAND_Y);
        $display("=========================================================");

        repeat (5) @(posedge clk);
        rst <= 1'b0;
        repeat (3) @(posedge clk);

        // ---------- T1：帧边界滚动，帧内恒定 ----------
        $display("\n[T1] 帧边界滚动（每帧 +%0d，帧内不变）", SCROLL_STEP);
        run_frame;
        sp0 = dut.scroll_pos;
        run_frame;
        sp1 = dut.scroll_pos;
        check(sp1 === (sp0 + SCROLL_STEP), "帧边界 scroll_pos 增加 SCROLL_STEP");
        check(pos_change_in_de == 0, "行有效期间 scroll_pos 从未变化（无撕裂）");
        $display("     frame=%0d scroll_pos=%0d text_idx=%0d",
                 frames, dut.scroll_pos, dut.text_idx);

        // ---------- T2：从右滚入 + 滚动方向 ----------
        $display("\n[T2] 从右滚入 + 向左滚动");
        // 复位后第一帧：scroll_pos 很小，文本只有最左侧几列露出
        rst <= 1'b1;
        @(posedge clk); @(posedge clk);
        rst <= 1'b0;
        @(posedge clk);
        g_white = 0; g_min_x = 9999; sampling = 1'b1;
        run_frame;                         // 本帧 scroll_pos 从 0 自增到 2
        sampling = 1'b0;
        white_at_start = g_white;
        sp0 = g_min_x;
        $display("     起始帧：scroll_pos=%0d 白字像素=%0d 最左x=%0d",
                 dut.scroll_pos, white_at_start, sp0);
        check(white_at_start < 60, "起始帧只有字符最左几列露出（从屏外进入）");
        check(sp0 >= H_ACTIVE - 8, "起始帧白字紧贴屏幕右边缘");

        // 滚动 20 帧后，白字应明显左移
        for (i = 0; i < 20; i = i + 1) run_frame;
        g_white = 0; g_min_x = 9999; sampling = 1'b1;
        run_frame;
        sampling = 1'b0;
        sp1 = g_min_x;
        $display("     滚动后：scroll_pos=%0d 白字像素=%0d 最左x=%0d",
                 dut.scroll_pos, g_white, sp1);
        check(sp1 < sp0, "文字向左滚动（最左白字 x 减小）");
        check(g_white > white_at_start, "文字露出面积随滚动增大");

        // ---------- T3/T4：底带半透明 + 白字 ----------
        $display("\n[T3][T4] Alpha 半透明底带 + 白字渲染");
        // 先滚到文本完整进入屏幕
        while (dut.scroll_pos < H_ACTIVE + 40) run_frame;
        g_white = 0; g_band_ok = 0; g_band_bad = 0; sampling = 1'b1;
        run_frame;
        sampling = 1'b0;
        $display("     白字像素=%0d, 底带正确=%0d, 底带异常=%0d",
                 g_white, g_band_ok, g_band_bad);
        check(g_white > 0, "滚动到可见区后有白字像素输出");
        check(g_band_ok > 0 && g_band_bad == 0,
              "底带像素 = rgb_in>>1（Alpha 半透明，无异常）");

        // ---------- T5：多条循环 ----------
        $display("\n[T5] 多条字幕自动循环");
        tx0 = dut.text_idx;
        // SCROLL_STEP=1 时 period = text_width + H_ACTIVE = 352 + 64 = 416 帧
        // （3 倍放大后文本更宽，比 2 倍时需要更多帧才能滚完一条）
        begin : t5_loop
            for (i = 0; i < 600; i = i + 1) begin
                run_frame;
                if (dut.text_idx !== tx0[1:0]) begin
                    $display("     第 %0d 帧 text_idx 由 %0d 切到 %0d（周期回绕）",
                             i, tx0, dut.text_idx);
                    disable t5_loop;
                end
            end
        end
        check(dut.text_idx !== tx0[1:0], "滚满一周期后自动切下一条字幕");

        // ---------- T6：enable=0 完全透传 ----------
        $display("\n[T6] enable=0 时完全透传");
        enable <= 1'b0;
        @(posedge clk); @(posedge clk);
        de <= 1'b1; video_in <= 24'h123456; video_x <= 10'd10;
        @(posedge clk); @(posedge clk);
        check(video_out === 24'h123456, "enable=0 时 video_out == video_in");
        check(overlay_active === 1'b0, "enable=0 时 overlay_active == 0");

        // ---------- 汇总 ----------
        $display("\n=========================================================");
        if (errors == 0)
            $display(" RESULT: PASS  （全部检查通过，共 %0d 帧）", frames);
        else
            $display(" RESULT: FAIL  （%0d 项失败）", errors);
        $display("=========================================================");
        $finish;
    end

    initial begin
        #200_000_000;
        $display("RESULT: FAIL  （TIMEOUT）");
        $finish;
    end

endmodule
