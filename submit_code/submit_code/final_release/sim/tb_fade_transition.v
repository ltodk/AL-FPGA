`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_fade_transition —— media_fade_transition 转场状态机验证
//
// 覆盖：
//   T1 复位初值（alpha=255 / IDLE / bank 索引=0）
//   T2 空闲期 alpha 恒 255，且帧内恒定（无撕裂）
//   T3 切换请求 → 淡出，alpha 按 STEP 逐帧递减，且只降不升
//   T4 压到最暗档的那一帧 bank 索引才切换（不能提前切、不能切晚）
//   T5 最暗档保持 HOLD_FRAMES 帧
//   T6 淡入 alpha 逐帧递增回到 255，最终回 IDLE
//   T7 enable=0 时索引直接跟随、不启动转场
//   T8 转场途中再次切图 → 最终追到最新值，不卡死
// -----------------------------------------------------------------------------
module tb_fade_transition;

    // 与顶层 FADE_FRAMES_CFG / MIN_ALPHA_CFG 保持一致（8 帧 + 最糊档保留 6.25%）
    localparam integer FADE_FRAMES = 8;
    localparam integer MIN_ALPHA   = 16;
    localparam integer HOLD_FRAMES = 1;
    localparam integer STEP        = (256 - MIN_ALPHA) / FADE_FRAMES;

    reg        clk = 1'b0;
    reg        rst = 1'b1;
    reg        vs  = 1'b1;
    reg [1:0]  disp_buf_idx = 2'd0;
    reg        enable = 1'b0;

    wire [1:0] idx_q;
    wire [7:0] alpha;
    wire       busy;

    media_fade_transition #(
        .FADE_FRAMES (FADE_FRAMES),
        .MIN_ALPHA   (MIN_ALPHA),
        .HOLD_FRAMES (HOLD_FRAMES)
    ) dut (
        .clk            (clk),
        .rst            (rst),
        .vs             (vs),
        .disp_buf_idx   (disp_buf_idx),
        .enable         (enable),
        .disp_buf_idx_q (idx_q),
        .alpha          (alpha),
        .fade_busy      (busy)
    );

    always #20 clk = ~clk;      // 25MHz

    integer n_pass = 0;
    integer n_fail = 0;

    task check;
        input cond;
        input [8*96-1:0] msg;
        begin
            if (cond) begin
                n_pass = n_pass + 1;
                $display("  [ OK ] %0s", msg);
            end else begin
                n_fail = n_fail + 1;
                $display("  [FAIL] %0s  (alpha=%0d state=%0d idx_q=%0d)",
                         msg, alpha, dut.state, idx_q);
            end
        end
    endtask

    // ---------------- 帧驱动：一次 vs 脉冲 = 一帧 ----------------
    // vs 拉低 1 拍 -> 产生 frame_start（DUT 在下一个时钟沿动作）
    task frame;
        begin
            @(negedge clk); vs = 1'b1;
            repeat (2) @(negedge clk);
            vs = 1'b0;
            @(negedge clk);
            vs = 1'b1;
            repeat (4) @(negedge clk);
        end
    endtask

    // ---------------- 帧内 alpha 恒定检查 ----------------
    // 跑一帧，采集每个时钟的 alpha，统计不同取值个数
    task frame_check_stable;
        output integer distinct;
        integer k, first_val, cnt;
        reg [7:0] last;
        begin
            distinct = 0;
            last = 8'hxx;
            @(negedge clk); vs = 1'b1;
            repeat (2) @(negedge clk);
            vs = 1'b0;
            for (k = 0; k < 14; k = k + 1) begin
                @(negedge clk);
                if (k == 1) vs = 1'b1;      // 下降沿只保持 1 拍
                if (alpha !== last) begin
                    distinct = distinct + 1;
                    last = alpha;
                end
            end
            repeat (2) @(negedge clk);
        end
    endtask

    integer distinct;
    integer i;
    integer n_out;          // 淡出帧计数
    integer n_in;           // 淡入帧计数
    reg [7:0] prev_a;
    reg [7:0] a_seq [0:127];      // 每帧 alpha
    integer   seq_len;
    reg [1:0] i_seq [0:127];      // 每帧 送给 reader 的 bank
    reg [1:0] s_seq [0:127];      // 每帧 状态
    integer   t_floor;            // 首次到达最暗档(MIN_ALPHA)的帧序号
    integer   t_rise;             // 首次离开最暗档的帧序号
    integer   a_min;              // 全程观察到的最暗 alpha
    integer   idx_early_err;
    integer   dn_bad, up_bad;

    initial begin
        $display("");
        $display("========== 第10讲 转场状态机验证 ==========");
        $display("FADE_FRAMES=%0d  STEP=%0d  MIN_ALPHA=%0d  HOLD_FRAMES=%0d",
                 FADE_FRAMES, STEP, MIN_ALPHA, HOLD_FRAMES);

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (2) @(posedge clk);

        // ---------------- T1：复位初值 ----------------
        $display("\n[T1] 复位初值");
        check(alpha === 8'd255, "复位后 alpha=255（原图全亮）");
        check(dut.state === 2'd0, "复位后 state=IDLE");
        check(idx_q === 2'd0, "复位后 bank 索引=0");

        // ---------------- T2：enable=0 不介入 ----------------
        $display("\n[T2] enable=0：无图时不介入转场");
        disp_buf_idx = 2'd2;
        frame; frame;
        check(alpha === 8'd255, "enable=0 时 alpha 保持 255");
        check(idx_q === 2'd2, "enable=0 时 bank 索引直接跟随上游");
        check(busy === 1'b0, "enable=0 时不进转场");

        // 复位回 0，进入正常显示
        disp_buf_idx = 2'd0;
        rst = 1'b1; @(posedge clk); @(posedge clk); rst = 1'b0;
        @(posedge clk);
        enable = 1'b1;
        frame; frame;

        // ---------------- T3：空闲期 alpha 恒定 ----------------
        $display("\n[T3] 空闲期 alpha 恒 255，且帧内不跳变");
        prev_a = alpha;
        for (i = 0; i < 4; i = i + 1) begin
            frame_check_stable(distinct);
            if (distinct != 1) begin
                n_fail = n_fail + 1;
                $display("  [FAIL] 第 %0d 帧 alpha 出现了 %0d 个不同值（撕裂风险）",
                         i, distinct);
            end
        end
        if (distinct == 1) begin
            n_pass = n_pass + 1;
            $display("  [ OK ] 4 帧内 alpha 每帧只有 1 个取值（无撕裂）");
        end
        check(alpha === 8'd255, "空闲 4 帧后 alpha 仍为 255");

        // ---------------- T4：切换请求 -> 淡出 ----------------
        $display("\n[T4] 切换请求（bank 0 -> 1）→ 淡出到最暗档");
        disp_buf_idx = 2'd1;
        seq_len      = 0;

        for (i = 0; i < 90; i = i + 1) begin
            frame;
            a_seq[seq_len] = alpha;
            i_seq[seq_len] = idx_q;
            s_seq[seq_len] = dut.state;
            if (prev_a !== alpha) begin
                $display("     f%02d  alpha %3d -> %3d   state=%0d idx_q=%0d",
                         i, prev_a, alpha, dut.state, idx_q);
                prev_a = alpha;
            end
            seq_len = seq_len + 1;
            if (dut.state === 2'd0 && i > 20) begin
                $display("     转场完成于第 %0d 帧", i);
                i = 999;
            end
        end

        // ---- 逐帧序列分析 ----
        // 找出：首次到达最暗档的帧 t_floor、首次离开最暗档的帧 t_rise、全程最暗值 a_min
        // ★ 加了 MIN_ALPHA 之后，转场不再压到 0，所以"全黑"要改成"到达最暗档"来判。
        begin : analyze
            integer k;
            t_floor = -1;
            t_rise  = -1;
            a_min   = 255;
            idx_early_err = 0;
            dn_bad  = 0;
            up_bad  = 0;
            for (k = 0; k < seq_len; k = k + 1) begin
                if (a_seq[k] < a_min) a_min = a_seq[k];
                if (a_seq[k] === MIN_ALPHA[7:0] && t_floor < 0)
                    t_floor = k;
            end
            for (k = 1; k < seq_len; k = k + 1) begin
                // 淡出段：t_floor 之前必须单调不增
                if (t_floor >= 0 && k <= t_floor && a_seq[k] > a_seq[k-1])
                    dn_bad = dn_bad + 1;
                // 淡入段：t_rise 之后必须单调不减
                if (t_rise >= 0 && a_seq[k] < a_seq[k-1])
                    up_bad = up_bad + 1;
                if (a_seq[k] > MIN_ALPHA && a_seq[k-1] === MIN_ALPHA[7:0] && t_rise < 0)
                    t_rise = k;
            end
            // 关键约束：还没压到最暗档之前，bank 绝不能提前切走
            for (k = 0; k < seq_len; k = k + 1) begin
                if (a_seq[k] > MIN_ALPHA && k < t_floor && i_seq[k] !== 2'd0)
                    idx_early_err = idx_early_err + 1;
            end
            $display("     分析：到最暗档帧=%0d  离开最暗档帧=%0d  全程最暗 alpha=%0d  序列长度=%0d",
                     t_floor, t_rise, a_min, seq_len);
        end

        check(dn_bad == 0, "淡出阶段 alpha 单调不增");
        check(up_bad == 0, "淡入阶段 alpha 单调不减");
        check(idx_q === 2'd1, "转场结束后 bank 索引已切到 1");
        check(t_floor >= 0, "转场过程中 alpha 到达最暗档 MIN_ALPHA");
        check(a_min === MIN_ALPHA[7:0], "全程最暗 alpha 精确等于 MIN_ALPHA（不再压到纯黑）");
        check(idx_early_err == 0, "alpha>MIN_ALPHA 期间 bank 绝不提前切换");
        check(t_rise > 0 && i_seq[t_rise] === 2'd1,
              "bank 在画面重新变亮之前已完成切换");
        check(s_seq[t_rise] === 2'd3, "离开最暗档发生在淡入状态（S_IN）");
        check((t_rise - t_floor) >= (HOLD_FRAMES + 1),
              "最暗档保持帧数满足 HOLD_FRAMES");
        check(dut.state === 2'd0, "转场结束后回到 IDLE");
        check(alpha === 8'd255, "转场结束后 alpha=255（画面与原图逐位一致）");

        // ---------------- T5：转场途中再切图 ----------------
        $display("\n[T5] 转场途中再次切图（1 -> 3），应追到最新值");
        disp_buf_idx = 2'd3;
        frame; frame; frame;              // 淡出刚开始
        disp_buf_idx = 2'd2;              // 中途又改主意
        prev_a = alpha;
        for (i = 0; i < 90; i = i + 1) begin
            frame;
            if (dut.state === 2'd0) i = 999;
        end
        check(idx_q === 2'd2, "最终服从中途的最新请求（bank=2）");
        check(alpha === 8'd255, "再次转场后仍回到全亮");
        check(dut.state === 2'd0, "没有卡在转场状态");

        // ---------------- 汇总 ----------------
        $display("");
        $display("------------------------------------------");
        if (n_fail == 0) $display("RESULT: PASS   (%0d 项检查全部通过)", n_pass);
        else             $display("RESULT: FAIL   (通过 %0d, 失败 %0d)", n_pass, n_fail);
        $display("------------------------------------------");
        $finish;
    end

    initial begin
        #2_000_000;
        $display("RESULT: FAIL  （TIMEOUT）");
        $finish;
    end

endmodule
