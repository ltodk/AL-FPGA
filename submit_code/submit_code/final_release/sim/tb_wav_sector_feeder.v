`timescale 1ns/1ps
// ============================================================================
// tb_wav_sector_feeder.v —— 第 11 讲阶段 B：WAV 供给器回归
//
// 被测对象是一个"音频文件 → 16bit 小端样本字"的转换器，内部复用
// fat32_file_streamer 做簇链遍历。本 tb 用一个小型 FAT32 扇区模型喂它：
//
//   每簇 8 扇区；FAT 表在 LBA 10；数据区起点 LBA 100
//   文件：first_cluster=2、size=1024 字节 → 占 1 个簇的前 2 个扇区
//   FAT[2] = 0x0FFFFFFF（EOF，因为 1024 < 8*512，一个簇装得下）
//   扇区内容按可预测公式生成 → 能精确断言"字节拼装 + 顺序"完全正确
//
// 覆盖点：
//   F1 基本功能：512 个 16bit 字、顺序正确、字节拼装（小端）、无错误
//   F2 流控：fifo_room 不足一个扇区时**一个扇区都不发**（不是发了再丢）
//   F3 循环：LOOP_EN=1 时整首播完自动重头来，且第二轮数据依旧正确
//   F4 停止/重启：play_en 掉下来立刻停；再拉高从文件头重新开始
// ============================================================================
module tb_wav_sector_feeder;

    localparam integer FAT_LBA  = 10;
    localparam integer DATA_LBA = 100;
    localparam integer FILSZ    = 1024;

    reg clk = 0;
    reg rst_n = 0;
    reg abort = 0;
    always #5 clk = ~clk;                 // 100MHz

    // ---------------- 被测模块端口 ----------------
    reg         play_en = 0;
    reg  [12:0] fifo_room = 0;
    wire        fifo_wr_en;
    wire [15:0] fifo_wr_data;
    wire        busy;
    wire        done_pulse;
    wire [7:0]  fs_error;
    wire        sector_req;
    wire [31:0] sector_lba;
    wire        sector_busy;
    wire        sector_valid;
    wire [7:0]  sector_byte;
    wire [8:0]  sector_byte_index;
    wire        sector_done;

    wav_sector_feeder #(
        .LOOP_EN          (1),
        .LOOP_GAP_CYCLES  (100),
        .RETRY_GAP_CYCLES (200),
        .FIFO_ROOM_MIN    (256)
    ) dut (
        .clk                       (clk),
        .rst_n                     (rst_n),
        .abort                     (abort),
        .play_en                   (play_en),
        .first_cluster             (32'd2),
        .file_size                 (FILSZ),
        .sectors_per_cluster       (8'd8),
        .sectors_per_cluster_shift (3'd3),
        .fat_start_lba             (FAT_LBA),
        .data_start_lba            (DATA_LBA),
        .max_cluster               (32'd1000),
        .fifo_room                 (fifo_room),
        .fifo_wr_en                (fifo_wr_en),
        .fifo_wr_data              (fifo_wr_data),
        .busy                      (busy),
        .done_pulse                (done_pulse),
        .fs_error                  (fs_error),
        .sector_req                (sector_req),
        .sector_lba                (sector_lba),
        .sector_busy               (sector_busy),
        .sector_valid              (sector_valid),
        .sector_byte               (sector_byte),
        .sector_byte_index         (sector_byte_index),
        .sector_done               (sector_done),
        .sector_error              (8'd0)
    );

    // ========================================================================
    // FAT32 扇区模型
    // ========================================================================
    function [7:0] fdata;
        input [31:0] lba;
        input [9:0]  idx;
        integer k;
        begin
            if (lba == FAT_LBA) begin
                // FAT[2] 落在偏移 8..11 → 0x0FFFFFFF
                case (idx)
                    10'd8, 10'd9, 10'd10: fdata = 8'hFF;
                    10'd11:               fdata = 8'h0F;
                    default:              fdata = 8'h00;
                endcase
            end else if (lba == DATA_LBA) begin
                k = idx;                      // 文件字节 0..511
                fdata = (k * 3) & 8'hFF;
            end else if (lba == DATA_LBA + 1) begin
                k = idx;
                fdata = ((k + 512) * 3) & 8'hFF;   // 与文件字节序列连续
            end else begin
                fdata = 8'hEE;
            end
        end
    endfunction

    reg [1:0]  mst;
    reg [31:0] m_lba;
    reg [9:0]  m_cnt;
    reg        m_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mst    <= 2'd0;
            m_lba  <= 32'd0;
            m_cnt  <= 10'd0;
            m_done <= 1'b0;
        end else begin
            m_done <= 1'b0;
            case (mst)
                2'd0: if (sector_req) begin
                    m_lba <= sector_lba;
                    m_cnt <= 10'd0;
                    mst   <= 2'd1;
                end
                2'd1: begin
                    m_cnt <= m_cnt + 10'd1;
                    if (m_cnt == 10'd511)
                        mst <= 2'd2;
                end
                2'd2: begin
                    m_done <= 1'b1;
                    mst    <= 2'd0;
                end
                default: mst <= 2'd0;
            endcase
        end
    end

    assign sector_busy        = (mst != 2'd0);
    assign sector_valid       = (mst == 2'd1);
    assign sector_byte        = fdata(m_lba, m_cnt);
    assign sector_byte_index  = m_cnt[8:0];
    assign sector_done        = m_done;

    // ---------------- 捕获写出的字 ----------------
    localparam integer MAXCAP = 4096;
    reg [15:0] cap [0:MAXCAP-1];
    integer    cap_n = 0;
    integer    req_n = 0;        // 扇区请求计数

    always @(posedge clk) begin
        if (fifo_wr_en && cap_n < MAXCAP) begin
            cap[cap_n] = fifo_wr_data;
            cap_n = cap_n + 1;
        end
        if (sector_req)
            req_n = req_n + 1;
    end

    // ---------------- 期望值 ----------------
    function [15:0] exp_word;
        input integer j;         // 第 j 个 16bit 字
        reg [7:0] lo, hi;
        integer k;
        begin
            k  = 2 * j;
            lo = (k * 3) & 8'hFF;
            hi = ((k + 1) * 3) & 8'hFF;
            exp_word = {hi, lo};        // 小端：先到的低字节在 bit[7:0]
        end
    endfunction

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

    task check_block;
        input integer base;      // 这一批的起始下标
        input integer n;
        input [8*48-1:0] name;
        integer j;
        integer bad;
        begin
            bad = 0;
            for (j = 0; j < n; j = j + 1)
                if (cap[base + j] !== exp_word(j)) begin
                    bad = bad + 1;
                    if (bad < 5)
                        $display("    word %0d: got %04h expect %04h",
                                 j, cap[base + j], exp_word(j));
                end
            check(bad == 0, name);
        end
    endtask

    // ---------------- 主流程 ----------------
    integer base;
    initial begin
        $display("============================================================");
        $display(" tb_wav_sector_feeder -- PCM bytes -> 16bit words");
        $display("============================================================");

        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // ---------------- F2：流控（先测，从"缓冲不够"开始）----------------
        $display("\n--- F2 flow control: no sector request when fifo_room < 256 ---");
        fifo_room = 13'd0;
        play_en   = 1'b1;
        repeat (2000) @(posedge clk);
        $display("  sector_req count = %0d, words written = %0d", req_n, cap_n);
        check(req_n == 0, "F2 zero sector request while fifo is full");
        check(cap_n == 0, "F2 no word written while fifo is full");
        check(dut.file_pause === 1'b1, "F2 internal streamer paused");

        // ---------------- F1：放开流控，正常读完整首 ----------------
        $display("\n--- F1 basic: 1024 bytes -> 512 words ---");
        fifo_room = 13'd1024;
        base = cap_n;
        wait (cap_n >= base + 512);
        repeat (20) @(posedge clk);
        check((cap_n - base) >= 512, "F1 at least 512 words produced");
        check_block(base, 512, "F1 all 512 words correct (little-endian packing)");
        check(fs_error == 8'd0, "F1 no filesystem error");

        // ---------------- F3：循环 ----------------
        $display("\n--- F3 loop: second pass identical ---");
        base = cap_n;
        wait (cap_n >= base + 512);
        repeat (20) @(posedge clk);
        check((cap_n - base) >= 512, "F3 second pass produced 512 more words");
        check_block(base, 512, "F3 second pass data identical");

        // ---------------- F4：停止 / 重启 ----------------
        $display("\n--- F4 stop then restart from head ---");
        play_en = 1'b0;
        repeat (30) @(posedge clk);
        base = cap_n;
        repeat (2000) @(posedge clk);
        check(cap_n == base, "F4 playback stops when play_en low");
        check(busy === 1'b0, "F4 internal streamer idle when play_en low");

        play_en = 1'b1;
        wait (cap_n >= base + 512);
        repeat (20) @(posedge clk);
        check_block(base, 512, "F4 restart begins from the first sample again");

        // 收尾：停掉，避免无限循环刷屏
        play_en = 1'b0;
        repeat (10) @(posedge clk);

        $display("\n============================================================");
        $display(" RESULT: %0d passed, %0d failed", pass_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display(" >>> FEEDER REGRESSION PASS");
        else
            $display(" >>> FEEDER REGRESSION FAIL");
        $display("============================================================");
        $finish;
    end

    initial begin
        #2000000;
        $display("[FATAL] timeout at %0t", $time);
        $display(" RESULT: %0d passed, %0d failed", pass_cnt, fail_cnt);
        $finish;
    end

endmodule
