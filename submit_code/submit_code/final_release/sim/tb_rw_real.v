`timescale 1ns/1ps
// =============================================================================
// tb_rw_real —— 用"真实读侧 frame_fifo_read"压读写仲裁
//
// 为什么需要这个 tb：
//   之前的 tb_wr_arbiter 里读侧只是一个"看到 !busy 就开 128 拍 burst"的假设模型，
//   而真实 frame_fifo_read 进 burst 的路径是：
//     S_CHECK_FIFO --(同拍检查 ~App_wr_busy)--> S_READ_BURST --> App_rd_en 延后 2 拍
//   这与模型不完全一致，必须用真 RTL 才能看出真正的重叠窗口。
//
// 检查项：
//   ① 同一拍 App_rd_en 与 App_wr_en 同时为高    → SDRAM App 端口违规
//   ② 写 burst 期间出现 App_rd_en                → 同上
//   ③ 读 burst 期间写 burst 启动（App_wr_en 上升沿落在 rd_busy 内）
//   ④ 统计写 burst 数与最小"读→写"间隔拍数（给硬件总线换向余量参考）
// =============================================================================
module tb_rw_real;

    localparam integer TOTAL_WORDS = 614400;   // 2 帧
    localparam integer WR_GAP      = 20;       // 写源速率：每 20 拍 1 字

    reg clk     = 1'b0;    // 写源域 100M
    reg dst_clk = 1'b0;    // SDRAM 侧 125M
    always #5 clk     = ~clk;
    always #4 dst_clk = ~dst_clk;
    reg rst_n = 1'b0;

    // ---------------- 写源：模拟 loader 的 app_wr_en / addr / data ----------------
    reg         src_wr_en;
    reg  [20:0] src_wr_addr;
    reg  [3:0]  src_wr_mask;
    reg  [31:0] src_wr_data;
    wire        src_full, src_overflow;
    integer     wr_left, wr_gap_cnt;
    reg  [20:0] waddr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            src_wr_en  <= 1'b0;
            src_wr_addr<= 21'd0;
            src_wr_data<= 32'd0;
            src_wr_mask<= 4'd0;
            wr_left    <= TOTAL_WORDS;
            wr_gap_cnt <= 0;
            waddr      <= 21'd0;
        end else begin
            src_wr_en <= 1'b0;
            if (wr_left > 0) begin
                if (wr_gap_cnt == WR_GAP - 1) begin
                    wr_gap_cnt  <= 0;
                    src_wr_en   <= 1'b1;
                    src_wr_addr <= waddr;
                    src_wr_data <= {7'd0, waddr};
                    src_wr_mask <= 4'b0000;
                    waddr       <= waddr + 1'b1;
                    wr_left     <= wr_left - 1;
                end else begin
                    wr_gap_cnt <= wr_gap_cnt + 1;
                end
            end
        end
    end

    // ---------------- 真实 CDC ----------------
    wire        dst_app_wr_en;
    wire [20:0] dst_app_wr_addr;
    wire [3:0]  dst_app_wr_mask;
    wire [31:0] dst_app_wr_data;
    wire        dst_idle, dst_wr_busy;

    // ---------------- 真实读侧 frame_fifo_read ----------------
    wire        app_rd_en, app_rd_busy;
    wire        vid_consume = 1'b1;     // 视频每拍都在消费，读侧不会因 FIFO 满而停
    wire [20:0] app_rd_addr;
    reg         read_req;
    wire        read_req_ack;
    wire        read_finish;

    // read_req 是"电平保持直到 ack"的握手：一帧读完(read_finish)再发下一次请求，
    // 等价于视频每帧向读通道要一次整帧数据。
    always @(posedge dst_clk or negedge rst_n) begin
        if (!rst_n)          read_req <= 1'b1;
        else if (read_req_ack) read_req <= 1'b0;
        else if (read_finish)  read_req <= 1'b1;
    end
    // 读 FIFO 占用恒为 0 = 视频消费足够快，读侧会"背靠背"发 burst。
    // 这是对写仲裁最恶劣的激励：写侧只能在读 burst 之间的极短空隙抢总线。
    wire [9:0] rdusedw_model = 10'd0;

    frame_fifo_read #(
        .MEM_DATA_BITS (32), .ADDR_BITS (21), .BURST_BITS (9),
        .FIFO_DEPTH (512), .BURST_SIZE (128)
    ) u_rd (
        .rst               (~rst_n),
        .mem_clk           (dst_clk),
        .Sdr_init_done     (1'b1),
        .Sdr_init_ref_vld  (1'b1),
        .Sdr_busy          (1'b0),
        .Sdr_rd_en         (1'b1),
        .App_wr_busy       (dst_wr_busy),
        .O_rd_busy         (app_rd_busy),
        .App_rd_en         (app_rd_en),
        .App_rd_addr       (app_rd_addr),
        .read_req          (read_req),
        .read_req_ack      (read_req_ack),
        .read_finish       (read_finish),
        .read_addr_0       (21'd0),
        .read_addr_1       (21'd307200),
        .read_addr_2       (21'd0),
        .read_addr_3       (21'd0),
        .read_addr_index   (2'd0),          // 固定读 bank0
        .read_len          (21'd307200),
        .fifo_aclr         (),
        .wrusedw           (rdusedw_model[9:0])
    );

    media_sdram_write_cdc #(
        .ADDR_WIDTH (21), .FIFO_ADDR_WIDTH (9), .BURST_SIZE (128)
    ) u_cdc (
        .src_clk (clk), .src_rst_n (rst_n),
        .src_wr_en (src_wr_en), .src_wr_addr (src_wr_addr),
        .src_wr_mask (src_wr_mask), .src_wr_data (src_wr_data),
        .src_full (src_full), .src_overflow (src_overflow),
        .src_frame_start (1'b0),
        .dst_clk (dst_clk), .dst_rst_n (rst_n),
        .dst_app_wr_en (dst_app_wr_en), .dst_app_wr_addr (dst_app_wr_addr),
        .dst_app_wr_mask (dst_app_wr_mask), .dst_app_wr_data (dst_app_wr_data),
        .dst_idle (dst_idle),
        .dst_rd_busy (app_rd_busy), .dst_wr_busy (dst_wr_busy), .guard_dis (1'b0)
        .dst_ref_vld  (1'b0),
        .dst_blanking     (1'b1),          // new: tb 里恒为消隐期(老行为)
        .dst_wr_hold      (1'b0),          // 2026-10-29 端口（老 tb 里没接，必须补）
        .dst_wr_active    (),
    );

    // ---------------- 检查 ----------------
    integer same_cycle_err = 0;
    integer overlap_err    = 0;
    integer wr_bursts      = 0;
    integer wr_order_err   = 0;      // 写地址/数据不连续（丢字或重字）
    integer wr_words       = 0;
    reg [20:0] expect_addr = 21'd0;
    integer rd_bursts      = 0;
    integer last_rd_cycle  = -1000;
    integer min_gap        = 999999;
    integer cyc            = 0;
    reg     wr_run, rd_run;
    integer rd_start_cycle, rd_len;

    always @(posedge dst_clk) begin
        if (rst_n) begin
            cyc <= cyc + 1;

            // ①② 同一拍读写同时出现
            if (app_rd_en && dst_app_wr_en) begin
                same_cycle_err <= same_cycle_err + 1;
                if (same_cycle_err < 6)
                    $display("  [ERR] t=%0t same-cycle App_rd_en & App_wr_en (rd_addr=%0d wr_addr=%0d)",
                             $time, app_rd_addr, dst_app_wr_addr);
            end
            // ③ 写 burst 期间出现读请求
            if (dst_app_wr_en && app_rd_en) overlap_err <= overlap_err + 1;

            // ④ 写流完整性：地址必须严格 0,1,2,...（丢字/重字会立刻跳号）
            if (dst_app_wr_en) begin
                wr_words <= wr_words + 1;
                if (dst_app_wr_addr !== expect_addr || dst_app_wr_data[20:0] !== expect_addr) begin
                    wr_order_err <= wr_order_err + 1;
                    if (wr_order_err < 6)
                        $display("  [ERR] t=%0t wr order: addr=%0d expect=%0d data=%0d",
                                 $time, dst_app_wr_addr, expect_addr, dst_app_wr_data[20:0]);
                end
                expect_addr <= expect_addr + 1'b1;
            end

            // 统计 burst
            if (dst_app_wr_en && !wr_run) begin
                wr_run <= 1'b1;
                wr_bursts <= wr_bursts + 1;
                if (cyc - last_rd_cycle < min_gap && last_rd_cycle > 0)
                    min_gap <= cyc - last_rd_cycle;
            end else if (!dst_app_wr_en) wr_run <= 1'b0;

            if (app_rd_en && !rd_run) begin
                rd_run <= 1'b1;
                rd_bursts <= rd_bursts + 1;
            end else if (!app_rd_en) begin
                rd_run <= 1'b0;
            end
            if (app_rd_en) last_rd_cycle <= cyc;
        end
    end

    // ---------------- 主流程 ----------------
    integer same_cycle_final, overlap_final, order_final, words_final;
    initial begin
        #200 rst_n = 1'b1;
        // 等两帧写完
        wait (wr_left == 0);
        #200000;
        same_cycle_final = same_cycle_err;
        overlap_final    = overlap_err;
        order_final      = wr_order_err;
        words_final      = wr_words;
        $display("=================================================");
        $display(" 写 burst 数        : %0d", wr_bursts);
        $display(" 读 burst 数        : %0d", rd_bursts);
        $display(" 同拍读写同时有效    : %0d  (必须 0)", same_cycle_final);
        $display(" 写 burst 内出现读   : %0d  (必须 0)", overlap_final);
        $display(" 最小 读->写 间隔    : %0d 拍", min_gap);
        $display(" CDC overflow       : %b  (必须 0)", src_overflow);
        $display(" written words      : %0d (expect %0d)", words_final, TOTAL_WORDS);
        $display(" write order errors : %0d (must be 0)", order_final);
        if (same_cycle_final == 0 && overlap_final == 0 && src_overflow == 1'b0 &&
            order_final == 0 && words_final == TOTAL_WORDS)
            $display("RESULT: PASS (读写严格互斥)");
        else
            $display("RESULT: FAIL");
        $display("=================================================");
        $finish;
    end

endmodule
