// tb_alarm_tone.v —— 验证第16讲 FPGA 内合成的插播警报（2026-10-31 三修：step 驱动版）
//
// 为什么要改：板上"警报播放时偶发密集打火声"。根因是合成器自带 48kHz 相位累加器，
// 与 sink 真实取样节拍（mux_valid && source_ready，受 WAV FIFO 出水影响）不同源，
// 同一样本被重复送出或跳过 ⇒ 方波边沿抖动，听感像煤气灶打火。
// 现在合成器只认 step（= 桥回送的"这一拍真的送出了一个样本"），本 tb 直接造 step，
// 而且故意让 step 间隔抖动（512/520/528/536/544 循环），模拟 WAV 侧不规则节奏。
//
// 判定：
//   ① 幅度只允许 0 / ±AMP/4 / ±AMP/2 / ±AMP；左右同值；
//   ② 所有相邻上升沿间隔只能是 48（1000Hz）或 64（750Hz）个 step，且两种都要出现
//      —— 这就是"节拍完全由 step 决定、step 抖动不影响波形"的直接证据；
//   ③ run 掉下去后正好 720 个 step（3 档 × 5ms × 240）包络归零，之后样本保持 0；
//   ④ 重新 run 能从第一声重新起来。

`timescale 1ns/1ps

module tb_alarm_tone;

    localparam integer AMP      = 12000;
    localparam integer SEG      = 16800;    // 0.35s @48kHz
    localparam integer PER_A    = 48;       // 1000Hz 整周期 = 48 个 step
    localparam integer PER_B    = 64;       //  750Hz 整周期 = 64 个 step
    localparam integer RAMP_ST  = 240;      // 5ms = 240 个 step 一档
    localparam integer N_SEG    = 2;        // 跑两段 = 33600 step

    reg  clk = 1'b0;
    reg  rst_n = 1'b0;
    reg  run = 1'b0;
    wire signed [15:0] sample_left;
    wire signed [15:0] sample_right;
    wire active;

    always #20 clk = ~clk;          // 25MHz

    // ---- step 发生器：间隔故意抖动，模拟 sink 侧不规则取样 ----
    reg  [3:0] gap_idx = 4'd0;
    reg  [9:0] gap_cnt = 10'd0;
    reg        step = 1'b0;
    wire [9:0] gap_now = (gap_idx == 4'd0) ? 10'd520 :
                         (gap_idx == 4'd1) ? 10'd512 :
                         (gap_idx == 4'd2) ? 10'd536 :
                         (gap_idx == 4'd3) ? 10'd528 : 10'd544;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gap_cnt <= 10'd0;
            gap_idx <= 4'd0;
            step    <= 1'b0;
        end else begin
            if (gap_cnt >= (gap_now - 10'd1)) begin
                gap_cnt <= 10'd0;
                gap_idx <= (gap_idx >= 4'd4) ? 4'd0 : (gap_idx + 4'd1);
                step    <= 1'b1;
            end else begin
                gap_cnt <= gap_cnt + 10'd1;
                step    <= 1'b0;
            end
        end
    end

    alarm_tone_gen #(
        .CLOCK_HZ  (25_000_000),
        .SAMPLE_HZ (48_000),
        .AMP       (AMP[15:0])
    ) u_dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .run          (run),
        .step         (step),
        .sample_left  (sample_left),
        .sample_right (sample_right),
        .active       (active)
    );

    // ---------------- 监视 ----------------
    integer n_step = 0;
    integer n_pop  = 0;
    integer err_amp = 0;
    integer err_lr  = 0;
    integer n48 = 0;
    integer n64 = 0;
    integer n_other = 0;
    integer n_rise = 0;
    integer last_rise = -1;
    integer late_change = 0;
    integer release_step = -1;
    integer run0_step = -1;
    reg  signed [15:0] prev_s = 16'sd0;
    reg  act_d = 1'b0;
    reg  act_seen = 1'b0;

    wire [15:0] a2 = (sample_left > 0) ? sample_left : -sample_left;
    wire [15:0] lv_amp  = AMP;
    wire [15:0] lv_half = AMP / 2;
    wire [15:0] lv_qtr  = AMP / 4;

    always @(posedge clk) begin
        if (rst_n) begin
            act_d <= active;
            if (active) act_seen <= 1'b1;

            if (step) begin
                n_step <= n_step + 1;
                if (active) begin
                    n_pop <= n_pop + 1;
                    // ① 幅度与左右
                    if (!((a2 == 0) || (a2 == lv_qtr) || (a2 == lv_half) || (a2 == lv_amp))) begin
                        err_amp <= err_amp + 1;
                        if (err_amp < 5) $display("  [ERR] bad amplitude at step %0d: %0d", n_step, sample_left);
                    end
                    if (sample_left !== sample_right) err_lr <= err_lr + 1;
                    // ② 上升沿间隔：只允许 48 或 64
                    if ((prev_s <= 0) && (sample_left > 0) && (release_step < 0)) begin
                        n_rise <= n_rise + 1;
                        if (last_rise >= 0) begin
                            if ((n_step - last_rise) == PER_A)      n48 <= n48 + 1;
                            else if ((n_step - last_rise) == PER_B) n64 <= n64 + 1;
                            else begin
                                n_other <= n_other + 1;
                                if (n_other < 6) $display("  [ERR] bad period %0d at step %0d n_pop=%0d",
                                                          n_step - last_rise, n_step, n_pop);
                            end
                        end
                        last_rise <= n_step;
                    end
                end
                prev_s <= sample_left;
            end

            // ③ 收尾沿：不受 step 限制
            if (act_d && !active && (release_step < 0) && act_seen) begin
                release_step <= n_step;
                $display("  release after %0d steps (expect %0d)", n_step - run0_step, 3*RAMP_ST);
            end
            if (!run && (run0_step < 0) && act_seen && (n_pop >= N_SEG*SEG - 1)) run0_step <= n_step;
        end
    end

    // ④ 包络归零后样本必须停住（只在"撤掉 run 之后、重新拉响之前"这一段检查）
    reg phase_end_check = 1'b0;
    always @(posedge clk) begin
        if (rst_n && phase_end_check && (release_step >= 0) && (n_step > release_step + 3) && step) begin
            if ((sample_left !== prev_s) || (sample_left !== 16'sd0)) begin
                late_change <= late_change + 1;
                if (late_change < 5) $display("  [ERR] sample changed after zero: %0d", sample_left);
            end
        end
    end

    // ---------------- 激励 ----------------
    integer i;
    initial begin
        $display("=== tb_alarm_tone (step driven) ===");
        #200;
        rst_n = 1'b1;
        #2000;

        // ---- 阶段 1：拉响两段（1000Hz + 750Hz）----
        run = 1'b1;
        i = 0;
        while (n_pop < (N_SEG * SEG)) begin
            #100;
            i = i + 1;
            if (i > 40_000_000) begin $display("TIMEOUT stage1"); $display("RESULT: FAIL"); $finish; end
        end
        $display("  two segments done: n_pop=%0d n_step=%0d", n_pop, n_step);

        // ---- 阶段 2：撤掉 run，等包络收尾 ----
        run = 1'b0;
        i = 0;
        while (release_step < 0) begin
            #100;
            i = i + 1;
            if (i > 4_000_000) begin $display("TIMEOUT release"); $display("RESULT: FAIL"); $finish; end
        end
        phase_end_check = 1'b1;
        #200000;                      // 再跑十几个 step，确认样本停住
        phase_end_check = 1'b0;

        // ---- 阶段 3：重新拉响，必须重新有声 ----
        run = 1'b1;
        i = 0;
        while (!active) begin
            #100;
            i = i + 1;
            if (i > 4_000_000) begin $display("TIMEOUT restart"); $display("RESULT: FAIL"); $finish; end
        end
        #400000;
        run = 1'b0;
        #400000;

        $display("  n_step=%0d n_pop=%0d n_rise=%0d", n_step, n_pop, n_rise);
        $display("  spacing: 48x%0d 64x%0d other=%0d", n48, n64, n_other);
        $display("  err_amp=%0d err_lr=%0d late_change=%0d release=%0d",
                 err_amp, err_lr, late_change, (release_step - run0_step));

        if ((err_amp == 0) && (err_lr == 0) && (n_other == 0) &&
            (n48 > 300) && (n64 > 250) && (n_other <= 2) &&
            (late_change == 0) &&
            ((release_step - run0_step) >= (3*RAMP_ST - 2)) &&
            ((release_step - run0_step) <= (3*RAMP_ST + 2)))
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");
        $finish;
    end

endmodule
