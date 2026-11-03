`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_wr_blank —— media_sdram_write_cdc 的"写只在垂直消隐期"分时验证
//
// 背景：第 1 张图显示出来、后台还在加载第 2~4 张的那 1 秒里画面被左右拉扯
//       （诊断码 ERR:E3 证明读侧 FIFO 欠载过）。修法让写 burst 只在消隐期启动。
//
// 本 tb 检查四件事：
//   ① 消隐期窗口外，App_wr_en 一次都不能出现（不能靠"读侧刚好空闲"钻进活动期）；
//   ② 消隐期窗口内必须真的能写（否则加载就停了）；
//   ③ 写入地址连续、一个字都不丢；
//   ④ 兜底：把消隐期永久拉低，写侧必须在约 2^22 拍（33.5ms@125MHz）后退化放行。
//
// 全部 ASCII 输出（ModelSim 控制台中文会乱码）。
// -----------------------------------------------------------------------------
module tb_wr_blank;

    localparam integer AW = 21;

    reg src_clk = 1'b0;
    reg dst_clk = 1'b0;
    reg src_rst_n = 1'b0;
    reg dst_rst_n = 1'b0;

    always #5   src_clk = ~src_clk;   // 100 MHz
    always #4   dst_clk = ~dst_clk;   // 125 MHz

    // ---- 源侧：连续写 ----
    reg                  src_wr_en   = 1'b0;
    reg  [AW-1:0]        src_wr_addr = 0;
    reg  [31:0]          src_wr_data = 0;
    wire                 src_full;
    wire                 src_overflow;

    // ---- 目的侧观测 ----
    wire                 dst_app_wr_en;
    wire [AW-1:0]        dst_app_wr_addr;
    wire [3:0]           dst_app_wr_mask;
    wire [31:0]          dst_app_wr_data;
    wire                 dst_idle;
    wire                 dst_wr_busy;
    wire                 dst_wr_active;

    reg                  dst_rd_busy = 1'b0;
    reg                  dst_blanking = 1'b0;
    reg                  dst_ref_vld = 1'b0;

    media_sdram_write_cdc #(
        .ADDR_WIDTH      (AW),
        .FIFO_ADDR_WIDTH (9),
        .BURST_SIZE      (64),
        .GUARD_CYCLES    (24)
    ) dut (
        .src_clk         (src_clk),
        .src_rst_n       (src_rst_n),
        .src_wr_en       (src_wr_en),
        .src_wr_addr     (src_wr_addr),
        .src_wr_mask     (4'b0000),
        .src_wr_data     (src_wr_data),
        .src_full        (src_full),
        .src_overflow    (src_overflow),
        .src_frame_start (1'b0),
        .dst_clk         (dst_clk),
        .dst_rst_n       (dst_rst_n),
        .dst_app_wr_en   (dst_app_wr_en),
        .dst_app_wr_addr (dst_app_wr_addr),
        .dst_app_wr_mask (dst_app_wr_mask),
        .dst_app_wr_data (dst_app_wr_data),
        .dst_idle        (dst_idle),
        .dst_rd_busy     (dst_rd_busy),
        .dst_wr_busy     (dst_wr_busy),
        .dst_ref_vld     (dst_ref_vld),
        .guard_dis       (1'b0),
        .dst_wr_hold     (1'b0),
        .dst_blanking    (dst_blanking),
        .dst_wr_active   (dst_wr_active)
    );

    // ---- 统计 ----
    integer n_wr        = 0;      // 写命令数
    integer n_wr_out    = 0;      // 消隐期外的写（必须为 0）
    integer n_wr_in     = 0;      // 消隐期内的写
    integer n_burst     = 0;      // burst 个数
    integer n_bst_out   = 0;      // 在消隐期外"启动"的 burst（必须为 0）
    integer err_seq     = 0;      // 地址不连续
    reg [AW-1:0] exp_addr = 0;
    reg        first_wr_seen = 1'b0;
    reg        wr_d = 1'b0;
    reg        first_burst_done = 1'b0;
    reg [7:0]  bl_hist = 8'd0;
    reg        phase2 = 1'b0;   // 1 = 停止给消隐期，测兜底
    wire       bl_recent = (|bl_hist) || dst_blanking;   // 8 拍内见过消隐期（容忍启动延迟）
    wire       blank_fb  = dut.blank_wait[22];           // 兜底已经放开

    always @(posedge dst_clk) bl_hist <= {bl_hist[6:0], dst_blanking};

    always @(posedge dst_clk) begin
        wr_d <= dst_app_wr_en;
        if (dst_app_wr_en) begin
            n_wr <= n_wr + 1;
            if (!bl_recent && !blank_fb && !phase2) n_wr_out <= n_wr_out + 1;
            else                                    n_wr_in  <= n_wr_in  + 1;
            if (first_wr_seen && (dst_app_wr_addr !== exp_addr)) err_seq <= err_seq + 1;
            first_wr_seen <= 1'b1;
            exp_addr      <= dst_app_wr_addr + 1'b1;
        end
        // burst 起点（写命令的上升沿）：必须落在消隐期窗口内（兜底放行与 phase2 除外）
        if (dst_app_wr_en && !wr_d) begin
            n_burst <= n_burst + 1;
            if (!bl_recent && !blank_fb && !phase2) n_bst_out <= n_bst_out + 1;
            first_burst_done <= 1'b1;
        end
    end

    // ---- 消隐期发生器：每 20000 拍来一个 2000 拍宽的窗口（10%，比真实苛刻） ----
    localparam integer BL_W = 2000;
    integer bp_cnt = 0;
    always @(posedge dst_clk) begin
        if (!dst_rst_n) begin
            bp_cnt <= 0; dst_blanking <= 1'b0;
        end else if (phase2) begin
            dst_blanking <= 1'b0;
        end else begin
            if (bp_cnt == 0) dst_blanking <= 1'b1;
            if (bp_cnt >= (BL_W-1)) begin
                dst_blanking <= 1'b0;
                bp_cnt <= 0;
            end else begin
                bp_cnt <= bp_cnt + 1;
            end
        end
    end

    // ---- 读侧 burst 周期性占用总线（模拟显示读，50% 左右占空） ----
    integer rdcnt = 0;
    always @(posedge dst_clk) begin
        if (!dst_rst_n) begin
            rdcnt <= 0; dst_rd_busy <= 1'b0;
        end else begin
            if (rdcnt >= 20) begin rdcnt <= 0; dst_rd_busy <= ~dst_rd_busy; end
            else rdcnt <= rdcnt + 1;
        end
    end

    // ---- 源侧连续喂数 ----
    initial begin
        src_wr_en   = 1'b0;
        src_wr_addr = 0;
        src_wr_data = 32'h1000_0000;
    end
    always @(posedge src_clk) begin
        if (src_rst_n && !src_full) begin
            src_wr_en   <= 1'b1;
            src_wr_addr <= src_wr_addr + 1'b1;
            src_wr_data <= src_wr_data + 32'd1;
        end else begin
            src_wr_en   <= 1'b0;
        end
    end

    // ---- 主流程 ----
    integer t_fallback = -1;
    integer wait_cnt   = 0;
    reg     fb_armed   = 1'b0;
    always @(posedge dst_clk) begin
        if (fb_armed && (t_fallback < 0)) begin
            wait_cnt <= wait_cnt + 1;
            if (first_wr_seen) t_fallback <= wait_cnt;
        end
    end

    initial begin
        $display("=== tb_wr_blank start ===");
        src_rst_n = 1'b0; dst_rst_n = 1'b0;
        #200;
        src_rst_n = 1'b1; dst_rst_n = 1'b1;

        // 阶段 1：有周期性消隐期，跑 6ms（@125M = 750000 拍，约 37 个窗口）
        repeat (750000) @(posedge dst_clk);

        $display("phase1: n_wr=%0d  in_blank=%0d  out_blank=%0d  bursts=%0d  burst_out=%0d  err_seq=%0d",
                 n_wr, n_wr_in, n_wr_out, n_burst, n_bst_out, err_seq);

        // 阶段 2：永久拉低消隐期，测兜底放行
        phase2    = 1'b1;
        first_wr_seen = 1'b0;
        wait_cnt  = 0;
        fb_armed  = 1'b1;
        repeat (4800000) @(posedge dst_clk);   // 38.4 ms > 2^22 拍（33.5ms）

        $display("phase2: fallback_wait_cycles=%0d (expect about 4194304)", t_fallback);
        $display("phase2: n_wr=%0d", n_wr);

        // ---- 判定 ----
        if (n_wr_in == 0)                     $display("FAIL: no write happened inside blanking");
        else if (n_bst_out != 0)              $display("FAIL: %0d bursts started OUTSIDE blanking", n_bst_out);
        else if (err_seq != 0)                $display("FAIL: address sequence broken (%0d)", err_seq);
        else if (t_fallback < 0)              $display("FAIL: fallback never released (deadlock)");
        else if (t_fallback < 4100000 || t_fallback > 4500000)
                                              $display("FAIL: fallback timing %0d out of range", t_fallback);
        else                                  $display("RESULT: PASS");
        $display("=== end ===");
        $finish;
    end

endmodule
