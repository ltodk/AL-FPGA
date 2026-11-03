`timescale 1ns/1ps
// ============================================================================
// tb_wav_stream_player.v —— 第 11 讲阶段 B：WAV 播放器回归
//
// 覆盖点：
//   P1 未开声前不消费数据（fifo_rd_pop 被 playing 门控）
//   P2 sd_play_en 拉高、缓冲够水位后 active 拉起
//   P3 输出样本逐个与 (raw*volume)>>8 精确比对（音量缩放 + 顺序）
//   P4 数据断了以后是「持续补静音」而不是「停掉」：100 个节拍里照样 100 个样本，
//      且全是 0（★ 不能用"当前这一拍 valid=1"来断言：样本在输出队列里只停留
//      1 个 pix 时钟就被取走，占空比不到 1%，那样写必然假失败）
//   P5 断流超过 STALL_CYCLES 后 active 自动放下（回落琶音）
//   P6 sd_play_en 掉下来立刻放下 active
//   P7 48kHz 节拍精度：4800 个节拍 = 100ms（相位累加器无累计误差）
// ============================================================================
module tb_wav_stream_player;

    // ---------------- 时钟 ----------------
    reg pix_clk = 0;
    reg sd_clk  = 0;
    always #20 pix_clk = ~pix_clk;    // 25MHz
    always #5  sd_clk  = ~sd_clk;     // 100MHz

    reg pix_rst_n = 0;
    reg sd_rst_n  = 0;

    // ---------------- DUT ----------------
    reg         sd_play_en = 0;
    reg         sd_wr_en   = 0;
    reg  [15:0] sd_wr_data = 0;
    wire [12:0] sd_room;
    reg         sample_ready = 1'b1;
    wire        sample_valid;
    wire signed [15:0] sample_left;
    wire signed [15:0] sample_right;
    wire        active;
    wire        underrun;
    wire [12:0] level_dbg;
    reg  [7:0]  volume = 8'd255;

    wav_stream_player #(
        .CLOCK_HZ     (25_000_000),
        .SAMPLE_HZ    (48_000),
        .FIFO_AW      (10),
        .STALL_CYCLES (100_000),      // 4ms（默认 1s 太长，仿真按比例缩小）
        .REFILL_WORDS (32)
    ) dut (
        .sd_clk          (sd_clk),
        .sd_rst_n        (sd_rst_n),
        .sd_play_en      (sd_play_en),
        .sd_fifo_wr_en   (sd_wr_en),
        .sd_fifo_wr_data (sd_wr_data),
        .sd_fifo_room    (sd_room),
        .pix_clk         (pix_clk),
        .pix_rst_n       (pix_rst_n),
        .volume          (volume),
        .sample_valid    (sample_valid),
        .sample_ready    (sample_ready),
        .sample_left     (sample_left),
        .sample_right    (sample_right),
        .active          (active),
        .underrun        (underrun),
        .level_dbg       (level_dbg)
    );

    // ---------------- 测试素材：锯齿波 ----------------
    localparam integer NWAVE = 512;
    reg signed [15:0] wave [0:NWAVE-1];

    integer i;
    initial begin
        for (i = 0; i < NWAVE; i = i + 1)
            wave[i] = i * 64 - 8000;       // -8000 ~ 24656，正负都有
    end

    // ---------------- 输出样本监视 ----------------
    integer out_idx    = 0;
    integer out_bad    = 0;
    integer pop_cnt    = 0;
    integer zero_bad   = 0;
    reg     collect_en = 0;
    reg     zero_chk   = 0;
    integer exp_v;

    always @(posedge pix_clk) begin
        if (pix_rst_n && sample_valid && sample_ready) begin
            pop_cnt = pop_cnt + 1;
            if (collect_en) begin
                if (out_idx < NWAVE) begin
                    exp_v = ($signed(wave[out_idx]) * 255) >>> 8;
                    if ($signed(sample_left) !== exp_v) begin
                        out_bad = out_bad + 1;
                        if (out_bad < 6)
                            $display("  [player] sample %0d: got %0d expect %0d",
                                     out_idx, $signed(sample_left), exp_v);
                    end
                    out_idx = out_idx + 1;
                end
            end
            if (zero_chk && ($signed(sample_left) !== 16'sd0))
                zero_bad = zero_bad + 1;
        end
    end

    // ---------------- 检查工具 ----------------
    integer pass_cnt = 0;
    integer fail_cnt = 0;

    task check;
        input            cond;
        input [8*72-1:0] name;
        begin
            if (cond) begin
                pass_cnt = pass_cnt + 1;
                $display("  [PASS] %0s", name);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("  [FAIL] %0s", name);
            end
        end
    endtask

    task write_words;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) begin
                @(negedge sd_clk);
                sd_wr_en   <= 1'b1;
                sd_wr_data <= wave[k];
            end
            @(negedge sd_clk);
            sd_wr_en <= 1'b0;
        end
    endtask

    // ---------------- 主流程 ----------------
    time    t0, t1;
    integer c0;
    initial begin
        $display("============================================================");
        $display(" tb_wav_stream_player -- 48kHz playback over CDC FIFO");
        $display("============================================================");

        repeat (5) @(posedge pix_clk);
        pix_rst_n = 1; sd_rst_n = 1;
        repeat (5) @(posedge pix_clk);

        // ---------------- P1：先灌数据，但不该被消费 ----------------
        $display("\n--- P1 no consumption before playing ---");
        write_words(NWAVE);
        repeat (8) @(posedge pix_clk);
        check(level_dbg == NWAVE, "P1 all written samples still in fifo");
        check(active === 1'b0, "P1 active low while play_en=0");
        check(pop_cnt == 0, "P1 nothing popped before playing");

        // ---------------- P7：48kHz 节拍精度 ----------------
        $display("\n--- P7 4800 ticks must equal 100ms ---");
        @(posedge dut.sample_tick);
        t0 = $time;
        repeat (4800) @(posedge dut.sample_tick);
        t1 = $time;
        $display("  4800 ticks = %0d ns (expect 100000000 ns)", t1 - t0);
        check((t1 - t0) > 99_990_000 && (t1 - t0) < 100_010_000,
              "P7 sample rate = 48.000kHz (no accumulated error)");

        // ---------------- P2：开声 ----------------
        $display("\n--- P2 active rises after refill ---");
        collect_en = 1'b1;
        sd_play_en = 1'b1;
        wait (active === 1'b1);
        check(active === 1'b1, "P2 active rises when play_en && level >= refill");

        // ---------------- P3：512 个样本逐个比对 ----------------
        $display("\n--- P3 sample-by-sample compare (512 samples) ---");
        wait (out_idx >= NWAVE);
        repeat (4) @(posedge pix_clk);
        check(out_idx == NWAVE, "P3 512 samples output");
        check(out_bad == 0, "P3 every sample matches (raw*vol)>>8");

        // ---------------- P4：断流持续补静音 ----------------
        $display("\n--- P4 underrun keeps 48kHz stream alive with silence ---");
        collect_en = 1'b0;
        wait (underrun === 1'b1);
        check(underrun === 1'b1, "P4 underrun asserted when fifo empty");
        zero_chk = 1'b1;
        c0 = pop_cnt;
        repeat (100) @(posedge dut.sample_tick);
        zero_chk = 1'b0;
        check((pop_cnt - c0) >= 99, "P4 still ~100 samples per 100 ticks (no stall)");
        check(zero_bad == 0, "P4 every fill sample is exactly 0");

        // ---------------- P5：断流超时后放下 active ----------------
        $display("\n--- P5 stall timeout drops active ---");
        wait (active === 1'b0);
        check(active === 1'b0, "P5 active dropped after stall timeout");

        // ---------------- P6：play_en 掉下立刻放下 ----------------
        $display("\n--- P6 play_en low drops active ---");
        sd_play_en = 1'b0;
        repeat (6) @(posedge pix_clk);
        check(active === 1'b0, "P6 active low when play_en low");

        $display("\n============================================================");
        $display(" RESULT: %0d passed, %0d failed", pass_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display(" >>> PLAYER REGRESSION PASS");
        else
            $display(" >>> PLAYER REGRESSION FAIL");
        $display("============================================================");
        $finish;
    end

    initial begin
        #200000000;      // 200ms 上限（P7 自己就要 100ms）
        $display("[FATAL] timeout at %0t", $time);
        $display(" RESULT: %0d passed, %0d failed", pass_cnt, fail_cnt);
        $finish;
    end

endmodule
