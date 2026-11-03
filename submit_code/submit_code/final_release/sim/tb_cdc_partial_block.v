`timescale 1ns/1ps
// =============================================================================
// tb_cdc_partial_block —— 复现并验证 ERR:3A / ERR:0x38 两个"静默"故障
//
// 背景（实机现象，2026-10-27 晚）：
//   ① ERR:3A  —— CDC 源侧 FIFO 报过溢出，但把深度从 512 加深到 1024 照样报；
//   ② ERR:38  —— 切不了图；loader 自己一点错都不报（数码管上没有 0x4x），
//                sd_card_bmp 只是"等不到写完成信号"超时；期间不停重写同一张图
//                （V_FLIP 自下而上，看起来就是"从下往上刷一屏"）。
//
// 两个用例分别打这两件事：
//   TEST A（满判据相位差 → 掉字 → 3A）
//     模拟 media_frame_loader 的写使能是**寄存器输出**：
//         pop_allowed(t) = ... && !out_full(t)
//         app_wr_en(t+1) = pop_allowed(t)
//     也就是"按 t 拍的水位决定 t+1 拍写"。若 t 拍还差一个字没满、
//     t+1 拍刚好写满，这一拍就会被判满丢掉，而且不报错。
//     判据：src_overflow 必须为 0，且目的侧写出的字数必须等于源侧发出的字数
//           且目的侧真正写出的字数必须等于源侧被接收的字数（不丢不重）。
//
//   TEST B（残留不满块 → dst_idle 永不拉高 → 0x38 卡死）
//     BURST_SIZE=4，写 6 个字 = 1 个整块 + 2 个残留。
//     期望：dst_idle 保持 0（2 个字永远发不出去）；
//           打一拍 src_frame_start（= 新帧的 load_start）后 dst_idle 必须拉高，
//           且 6 个字全部写进 SDRAM 端口。
//
//   TEST C（正常整帧不受影响）：写 8 个字 = 2 个整块，直接通。
// =============================================================================
module tb_cdc_partial_block;

localparam integer AW = 4;    // FIFO 16 深
localparam integer BS = 4;    // burst 4 字

reg  src_clk, dst_clk, rst_n;
reg  manual_wr;
reg  auto_mode;
reg  [15:0] auto_left;
reg  reg_wr;
wire src_wr_en = auto_mode ? reg_wr : manual_wr;

reg  [20:0] src_addr;
reg  frame_start;
reg  rd_busy;

wire [20:0] dst_addr;
wire [31:0] dst_data;
wire [3:0]  dst_mask;
wire        dst_en;
wire        dst_idle;
wire        src_full;
wire        src_overflow;

integer errors;
integer full_cyc;     // src_wr_en && src_full 的拍数（仅供参考，不算失败：
                      //   src_full 现在是"提前 2 个字"的反压预告，为高时写仍会被收下）
integer issue_cnt;    // 源侧发出的字数（不管有没有被收下）
integer wr_count;     // 目的侧写出的字数
integer i;

reg [23:0]  seq_log [0:1023];
integer     addr_log [0:1023];
integer     seq_exp;
integer     guard;



// ---------------- DUT ----------------
media_sdram_write_cdc #(
    .ADDR_WIDTH      (21),
    .FIFO_ADDR_WIDTH (AW),
    .BURST_SIZE      (BS),
    .GUARD_CYCLES    (24)
) u_cdc (
    .src_clk         (src_clk),
    .src_rst_n       (rst_n),
    .src_wr_en       (src_wr_en),
    .src_wr_addr     (src_addr),
    .src_wr_mask     (4'b0000),
    .src_wr_data     ({8'h00, issue_cnt[23:0]}),
    .src_full        (src_full),
    .src_overflow    (src_overflow),
    .src_frame_start (frame_start),
    .dst_clk         (dst_clk),
    .dst_rst_n       (rst_n),
    .dst_app_wr_en   (dst_en),
    .dst_app_wr_addr (dst_addr),
    .dst_app_wr_mask (dst_mask),
    .dst_app_wr_data (dst_data),
    .dst_idle        (dst_idle),
    .dst_rd_busy     (rd_busy),
    .dst_ref_vld  (1'b0),
    .dst_blanking     (1'b1),          // new: tb 里恒为消隐期(老行为)
    .dst_wr_hold      (1'b0),          // 2026-10-29 端口（老 tb 里没接，必须补）
    .dst_wr_active    (),
    .dst_wr_busy     (),
    .guard_dis       (1'b0)
);

always #5  src_clk = ~src_clk;    // 100 MHz
always #4  dst_clk = ~dst_clk;    // 125 MHz

// ---------------- 源侧：模拟 loader 的"寄存器输出写使能" ----------------
always @(posedge src_clk or negedge rst_n) begin
    if (!rst_n) begin
        reg_wr    <= 1'b0;
        auto_left <= 16'd0;
        issue_cnt <= 0;
        src_addr  <= 21'd0;
        full_cyc  <= 0;
    end else begin
        reg_wr <= auto_mode && (auto_left != 16'd0) && !src_full;
        if (reg_wr)
            auto_left <= auto_left - 16'd1;
        if (src_wr_en && src_full)
            full_cyc <= full_cyc + 1;
        // 发出即计数：被拒收的那一笔也会让序号出现空洞，从而被下面的检查抓到
        if (src_wr_en) begin
            issue_cnt <= issue_cnt + 1;
            src_addr  <= src_addr + 21'd1;
        end
    end
end

// ---------------- 目的侧：记录每一次真正的写 ----------------
always @(posedge dst_clk) begin
    if (dst_en && wr_count < 1024) begin
        seq_log[wr_count]  = dst_data[23:0];
        addr_log[wr_count] = dst_addr;
        wr_count = wr_count + 1;
    end
end

task send_word;
    begin
        @(negedge src_clk);
        manual_wr = 1'b1;
        @(negedge src_clk);
        manual_wr = 1'b0;
    end
endtask

task do_reset;
    begin
        @(negedge src_clk);
        rst_n = 1'b0;
        repeat (6) @(negedge src_clk);
        rst_n = 1'b1;
        repeat (6) @(negedge src_clk);
        wr_count  = 0;
        issue_cnt = 0;
        full_cyc  = 0;
        src_addr  = 21'd0;
        guard     = 0;
    end
endtask

// 等 dst_idle（或超时）——返回 1 表示等到了
task wait_idle;
    input integer limit;
    output        ok;
    integer n;
    begin
        ok = 1'b0;
        n  = 0;
        while (!dst_idle && n < limit) begin
            @(posedge dst_clk);
            n = n + 1;
        end
        if (dst_idle) ok = 1'b1;
    end
endtask

reg idle_ok;

initial begin
    src_clk    = 1'b0;
    dst_clk    = 1'b0;
    rst_n      = 1'b1;
    manual_wr  = 1'b0;
    auto_mode  = 1'b0;
    auto_left  = 16'd0;
    reg_wr     = 1'b0;
    src_addr   = 21'd0;
    frame_start= 1'b0;
    rd_busy    = 1'b0;
    errors     = 0;
    wr_count   = 0;
    issue_cnt  = 0;
    full_cyc   = 0;
    guard      = 0;

    // =====================================================================
    // TEST A：满判据相位差
    //   rd_busy 反复抬起/放下，让写侧水位在"满/不满"之间反复摩擦 ——
    //   相位差只要存在，一定会在边界上掉字。
    // =====================================================================
    $display("---- TEST A: full-flag phase skew (loader-style registered wr_en) ----");
    do_reset;
    auto_mode = 1'b1;
    auto_left = 16'd200;

    // 发满 200 个字（= 50 个整块，故意凑整，让本用例只考察"掉字"，不掺残留块）
    // rd_busy 周期性抬起，把写侧水位反复顶到"满"的边界 —— 相位差只要存在，
    // 一定在这里丢字。注意让位窗口 512 拍是排空速度的上限，这里要给足时间。
    guard = 0;
    while (auto_left != 16'd0 && guard < 400000) begin
        // ★ 占空比不能太高：can_start 要求读侧连续 8 拍不 busy（rd_quiet>=8），
        //   读侧占空比一旦超过 8/11，rd_quiet 永远到不了 8，写通道会永久饥饿。
        //   这里取 3/16 ≈ 19%，接近实机显示读的占空比，既能排空又能摩擦满边界。
        rd_busy = ((guard % 16) < 3) ? 1'b1 : 1'b0;
        @(posedge dst_clk);
        guard = guard + 1;
    end
    rd_busy = 1'b0;
    if (auto_left != 16'd0) begin
        $display("FAIL[A]: 源侧只发出 %0d/200 字就卡住了", 200 - auto_left);
        errors = errors + 1;
    end
    auto_mode = 1'b0;
    wait_idle(600000, idle_ok);

    if (src_overflow) begin
        $display("FAIL[A]: src_overflow set - words silently dropped");
        errors = errors + 1;
    end
    if (!idle_ok) begin
        $display("FAIL[A]: dst_idle never asserted");
        errors = errors + 1;
    end
    if (wr_count != issue_cnt) begin
        $display("FAIL[A]: dst wrote %0d words, src issued %0d words - words lost", wr_count, issue_cnt);
        errors = errors + 1;
    end
    seq_exp = 0;
    for (i = 0; i < wr_count && i < 1024; i = i + 1) begin
        if (seq_log[i] !== seq_exp[23:0]) begin
            $display("FAIL[A]: word %0d data %0d, expected %0d (order broken / lost)",
                     i, seq_log[i], seq_exp);
            errors = errors + 1;
            i = 1024;
        end
        if (addr_log[i] !== i) begin
            $display("FAIL[A]: word %0d addr %0d, expected %0d", i, addr_log[i], i);
            errors = errors + 1;
            i = 1024;
        end
        seq_exp = seq_exp + 1;
    end
    $display("     A: issued %0d / wrote %0d / src_full-while-wr %0d", issue_cnt, wr_count, full_cyc);

    // =====================================================================
    // TEST B：残留不满块（0x38 卡死）
    // =====================================================================
    $display("---- TEST B: 6 words (1 full block + 2 leftover) ----");
    do_reset;
    for (i = 0; i < 6; i = i + 1)
        send_word;
    repeat (200) @(posedge dst_clk);

    if (dst_idle) begin
        $display("NOTE[B]: leftover went out by itself (dst_idle early)");
        // 这不是错误方向：能自己发出去说明逻辑更宽松。只提示，不判失败。
    end else begin
        $display("     B: dst_idle=0 (leftover block stuck, matches ERR:0x38)");
    end
    if (wr_count != 4) begin
        $display("FAIL[B]: expected 4 words written here, got %0d", wr_count);
        errors = errors + 1;
    end

    // 新帧启动 → 必须把残留放掉
    @(negedge src_clk);
    frame_start = 1'b1;
    @(negedge src_clk);
    frame_start = 1'b0;
    wait_idle(2000, idle_ok);

    if (!idle_ok) begin
        $display("FAIL[B]: dst_idle still low after src_frame_start - fix ineffective");
        errors = errors + 1;
    end
    if (wr_count != 6) begin
        $display("FAIL[B]: expected 6 words after close-up, got %0d", wr_count);
        errors = errors + 1;
    end
    $display("     B: after close-up wrote %0d / dst_idle=%b", wr_count, dst_idle);

    // =====================================================================
    // TEST C：正常整帧（2 个整块）不受影响
    // =====================================================================
    $display("---- TEST C: 8 words (2 full blocks), close-up must be harmless ----");
    do_reset;
    for (i = 0; i < 8; i = i + 1)
        send_word;
    @(negedge src_clk);
    frame_start = 1'b1;
    @(negedge src_clk);
    frame_start = 1'b0;
    wait_idle(2000, idle_ok);

    if (!idle_ok || wr_count != 8) begin
        $display("FAIL[C]: dst_idle=%b, wrote %0d (expect 8)", dst_idle, wr_count);
        errors = errors + 1;
    end else begin
        $display("     C: all 8 words written, dst_idle=1");
    end

    repeat (20) @(posedge dst_clk);
    $display("========================================");
    if (errors == 0)
        $display("RESULT: PASS  (no full-flag skew / leftover closable / frame intact)");
    else
        $display("RESULT: FAIL  errors %0d", errors);
    $display("========================================");
    $finish;
end

endmodule
