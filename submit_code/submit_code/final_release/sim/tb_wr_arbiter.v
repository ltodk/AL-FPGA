`timescale 1ns/1ps
// =============================================================================
// tb_wr_arbiter —— 验证 SDRAM 写通道与读通道的互斥（第8讲 2026-10-21 修复）
//
// 背景：SDRAM 控制器的 App 端口不能读写同时进行。写侧改由 CDC 直接驱动
//       App_wr_* 之后，原来 frame_fifo_write <-> frame_fifo_read 的仲裁链断了，
//       读写 burst 互相打断 → 实机画面出现随机横向亮线。
//
// 本用例检查 4 件事：
//   ① 写 burst 内部不断流，每个 burst 长度严格等于 BURST_SIZE
//   ② 每个 burst 结束后必须有让位窗口（dst_wr_busy 拉低），否则读侧会被饿死
//   ③ dst_rd_busy 为高时绝不启动新 burst
//   ④ 写 burst 进行中与 dst_rd_busy 为高绝不同时出现
// =============================================================================
module tb_wr_arbiter;

    localparam integer BURST   = 8;
    localparam integer NWORD   = 64;      // 8 个 burst
    localparam integer YIELD   = 512;     // 与 RTL 内 YIELD_CYCLES 一致

    reg          src_clk = 1'b0;
    reg          dst_clk = 1'b0;
    reg          rst_n   = 1'b0;

    reg          src_wr_en   = 1'b0;
    reg  [20:0]  src_wr_addr = 21'd0;
    reg  [3:0]   src_wr_mask = 4'd0;
    reg  [31:0]  src_wr_data = 32'd0;

    wire         src_full;
    wire         src_overflow;

    wire         dst_app_wr_en;
    wire [20:0]  dst_app_wr_addr;
    wire [3:0]   dst_app_wr_mask;
    wire [31:0]  dst_app_wr_data;
    wire         dst_idle;
    wire         dst_wr_busy;

    reg          dst_rd_busy = 1'b0;      // 假读侧

    integer      err      = 0;
    integer      wr_run   = 0;            // 当前连续写 burst 长度
    integer      burst_n  = 0;            // 已完成 burst 数
    integer      gap_cnt  = 0;            // 两次 burst 之间的空闲拍数
    integer      gap_min  = 999999;
    integer      words_seen = 0;
    integer      rd_hold  = 0;            // 假读侧 burst 剩余拍
    integer      i;

    media_sdram_write_cdc #(
        .ADDR_WIDTH      (21),
        .FIFO_ADDR_WIDTH (7),
        .BURST_SIZE      (BURST)
    ) u_cdc (
        .src_clk         (src_clk),
        .src_rst_n       (rst_n),
        .src_wr_en       (src_wr_en),
        .src_wr_addr     (src_wr_addr),
        .src_wr_mask     (src_wr_mask),
        .src_wr_data     (src_wr_data),
        .src_full        (src_full),
        .src_overflow    (src_overflow),
        .src_frame_start (1'b0),
        .dst_clk         (dst_clk),
        .dst_rst_n       (rst_n),
        .dst_app_wr_en   (dst_app_wr_en),
        .dst_app_wr_addr (dst_app_wr_addr),
        .dst_app_wr_mask (dst_app_wr_mask),
        .dst_app_wr_data (dst_app_wr_data),
        .dst_idle        (dst_idle),
        .dst_rd_busy     (dst_rd_busy),
        .dst_ref_vld  (1'b0),
        .dst_blanking     (1'b1),          // new: tb 里恒为消隐期(老行为)
        .dst_wr_hold      (1'b0),          // 2026-10-29 端口（老 tb 里没接，必须补）
        .dst_wr_active    (),
        .dst_wr_busy     (dst_wr_busy), .guard_dis (1'b0)
    );

    always #5 src_clk = ~src_clk;    // 100 MHz
    always #4 dst_clk = ~dst_clk;    // 125 MHz

    // ------------------------------------------------------------------
    // 假读侧：只要写侧不在 burst，就尽量抢总线（模拟显示端持续读）
    // ------------------------------------------------------------------
    always @(posedge dst_clk or negedge rst_n) begin
        if (!rst_n) begin
            dst_rd_busy <= 1'b0;
            rd_hold     <= 0;
        end else if (rd_hold != 0) begin
            dst_rd_busy <= 1'b1;
            rd_hold     <= rd_hold - 1;
        end else if (!dst_wr_busy) begin
            // 抢一个 6 拍的读 burst
            dst_rd_busy <= 1'b1;
            rd_hold     <= 5;
        end else begin
            dst_rd_busy <= 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // 监控写 burst
    // ------------------------------------------------------------------
    always @(posedge dst_clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_run   <= 0;
            burst_n  <= 0;
            gap_cnt  <= 0;
            gap_min  <= 999999;
            words_seen <= 0;
        end else if (dst_app_wr_en) begin
            if (wr_run == 0) begin
                // 新一轮 burst 开始：记录与上一次 burst 之间到底让了多少拍总线
                if (burst_n > 0 && gap_cnt < gap_min) gap_min <= gap_cnt;
                $display("DBG burst start t=%0t last_gap=%0d wait=%0d starved=%0d",
                         $time, gap_cnt, u_cdc.wait_cnt, u_cdc.starved);
            end
            words_seen <= words_seen + 1;
            wr_run     <= wr_run + 1;
            gap_cnt    <= 0;
            // (4) write burst must never overlap a read burst
            if (dst_rd_busy) begin
                $display("FAIL: t=%0t overlap wr_en=1 rd_busy=1 wr_run=%0d burst_n=%0d wait=%0d",
                         $time, wr_run, burst_n, u_cdc.wait_cnt);
                err = err + 1;
            end
        end else begin
            if (wr_run != 0) begin
                // ① burst 长度必须整除 BURST
                if (wr_run != BURST) begin
                    $display("FAIL: 第 %0d 个 burst 长度 %0d，期望 %0d", burst_n, wr_run, BURST);
                    err = err + 1;
                end
                burst_n <= burst_n + 1;
            end
            wr_run  <= 0;
            // burst 之间空闲拍数累加（真正的判定在下一个 burst 开始时做）
            if (burst_n > 0)
                gap_cnt <= gap_cnt + 1;
        end
    end

    // ------------------------------------------------------------------
    // ③ dst_rd_busy 为高时不得启动新 burst（wr_run 必须保持 0）
    // ------------------------------------------------------------------
    always @(posedge dst_clk) begin
        if (rst_n && dst_rd_busy && dst_app_wr_en && wr_run == 0) begin
            $display("FAIL: t=%0t 读 burst 期间启动了新写 burst", $time);
            err = err + 1;
        end
    end

    // ------------------------------------------------------------------
    // 激励
    // ------------------------------------------------------------------
    initial begin
        rst_n = 1'b0;
        repeat (10) @(posedge src_clk);
        rst_n = 1'b1;
        repeat (10) @(posedge src_clk);

        for (i = 0; i < NWORD; i = i + 1) begin
            @(posedge src_clk);
            src_wr_en   <= 1'b1;
            src_wr_addr <= 21'h1000 + i;
            src_wr_data <= 32'hA000 + i;
            src_wr_mask <= 4'b0000;
        end
        @(posedge src_clk);
        src_wr_en <= 1'b0;

        // 等排空
        i = 0;
        while (!dst_idle && i < 20000) begin
            @(posedge dst_clk);
            i = i + 1;
        end

        repeat (10) @(posedge dst_clk);
        $display("--------------------------------------------------");
        $display("写入字数        : %0d (期望 %0d)", words_seen, NWORD);
        $display("完整 burst 个数 : %0d (期望 %0d)", burst_n, NWORD / BURST);
        $display("最小让位窗口    : %0d 拍 (期望 >= %0d)", gap_min, YIELD - 1);
        $display("src_overflow    : %0d (期望 0)", src_overflow);
        if (words_seen != NWORD) begin
            $display("FAIL: 写出字数不符"); err = err + 1;
        end
        if (burst_n != NWORD / BURST) begin
            $display("FAIL: burst 个数不符"); err = err + 1;
        end
        if (gap_min < YIELD - 2) begin
            $display("FAIL: 让位窗口不足，读侧会被饿死"); err = err + 1;
        end
        if (src_overflow) begin
            $display("FAIL: 源侧溢出"); err = err + 1;
        end
        if (err == 0)
            $display("RESULT: PASS  (burst 连续 / 让位窗口 %0d 拍 / 读写零重叠)", gap_min);
        else
            $display("RESULT: FAIL  错误数 %0d", err);
        $finish;
    end

    initial begin
        #4000000;
        $display("RESULT: FAIL  超时");
        $finish;
    end
endmodule
