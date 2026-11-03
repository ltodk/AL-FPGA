`timescale 1ns/1ps
// =============================================================================
// tb_loader_fullframe —— 全帧写入链路验证（第8讲 阶段A 排障）
//
// 目的：小尺寸 tb 只验证了 12 个字，掩盖了 FIFO/burst 边界问题。
//       这里跑完整 307200 像素，检查 CDC 输出到 SDRAM 的 (addr,data) 序列：
//         1) 地址是否 0..307199 全覆盖、且每个地址恰好写一次
//         2) 数据是否等于期望的 V_FLIP 映射值
//       任何不符都打印前若干条，并统计总数。
// =============================================================================
module tb_loader_fullframe;

    localparam integer W = 640;
    localparam integer H = 480;
    localparam integer WORDS = W * H;   // 307200

    reg         sd_clk = 1'b0;
    reg         dst_clk = 1'b0;
    reg         rst_n = 1'b0;

    always #5.000  sd_clk  = ~sd_clk;    // 100MHz
    always #4.000  dst_clk = ~dst_clk;   // 125MHz

    // ---- DUT 接口 ----
    reg         load_start = 1'b0;
    reg  [1:0]  image_index = 2'd0;
    reg         pixel_valid = 1'b0;
    reg  [23:0] pixel_rgb = 24'd0;
    reg         frame_commit = 1'b0;

    wire        load_busy, load_done;
    wire [3:0]  image_complete;
    wire [7:0]  memory_error;

    wire        ldr_wr_en;
    wire [20:0] ldr_wr_addr;
    wire [3:0]  ldr_wr_mask;
    wire [31:0] ldr_wr_data;

    wire        cdc_full, cdc_overflow;
    wire        cdc_wr_en;
    wire [20:0] cdc_wr_addr;
    wire [3:0]  cdc_wr_mask;
    wire [31:0] cdc_wr_data;
    wire        cdc_idle;

    // ---- DUT 例化（与顶层完全一致的参数）----
    media_frame_loader #(
        .ADDR_WIDTH    (21),
        .IMAGE_WORDS   (WORDS),
        .FIFO_DEPTH    (512),
        .BURST_SIZE    (128),
        .STALL_TIMEOUT (65535),
        .WRITE_V_FLIP  (1),
        .FRAME_WIDTH   (W),
        .FRAME_HEIGHT  (H)
    ) u_loader (
        .clk                 (sd_clk),
        .rst_n               (rst_n),
        .load_start          (load_start),
        .image_index         (image_index),
        .pixel_valid         (pixel_valid),
        .pixel_rgb           (pixel_rgb),
        .frame_commit        (frame_commit),
        .decode_error        (8'd0),
        .load_busy           (load_busy),
        .load_done           (load_done),
        .image_complete      (image_complete),
        .memory_error        (memory_error),
        .controller_init_done(1'b1),
        .controller_busy     (1'b0),
        .refresh_active      (1'b0),
        .app_wr_en           (ldr_wr_en),
        .app_wr_addr         (ldr_wr_addr),
        .app_wr_mask         (ldr_wr_mask),
        .app_wr_data         (ldr_wr_data),
        .out_full            (cdc_full)              // 下游 CDC 满则暂停出队
    );

    media_sdram_write_cdc #(
        .ADDR_WIDTH      (21),
        .FIFO_ADDR_WIDTH (9),
        .BURST_SIZE      (128)
    ) u_cdc (
        .src_clk         (sd_clk),
        .src_rst_n       (rst_n),
        .src_wr_en       (ldr_wr_en),
        .src_wr_addr     (ldr_wr_addr),
        .src_wr_mask     (ldr_wr_mask),
        .src_wr_data     (ldr_wr_data),
        .src_full        (cdc_full),
        .src_overflow    (cdc_overflow),
        .src_frame_start (1'b0),
        .dst_clk         (dst_clk),
        .dst_rst_n       (rst_n),
        .dst_app_wr_en   (cdc_wr_en),
        .dst_app_wr_addr (cdc_wr_addr),
        .dst_app_wr_mask (cdc_wr_mask),
        .dst_app_wr_data (cdc_wr_data),
        .dst_idle        (cdc_idle),
        // 本 tb 独占 SDRAM 写端口，没有读侧争用，读忙恒 0
        .dst_rd_busy     (1'b0),
        .dst_ref_vld  (1'b0),
        .dst_blanking     (1'b1),          // new: tb 里恒为消隐期(老行为)
        .dst_wr_hold      (1'b0),          // 2026-10-29 端口（老 tb 里没接，必须补）
        .dst_wr_active    (),
        .dst_wr_busy     (), .guard_dis (1'b0)
    );

    // ---- 检查器：记录每个地址被写了多少次、最后一次写入了什么 ----
    reg [7:0]  write_count [0:WORDS-1];
    reg [31:0] last_data   [0:WORDS-1];

    integer i;
    initial begin
        for (i = 0; i < WORDS; i = i + 1) begin
            write_count[i] = 8'd0;
            last_data[i]   = 32'd0;
        end
    end

    integer n_writes = 0;
    integer n_dup = 0;
    integer n_oob = 0;
    integer n_data_bad = 0;
    integer n_printed = 0;

    // 期望数据：第 n 个像素 → 地址 (H-1-row)*W + col，数据 {8'h00, R, G, B}
    // 取一个可反算的编码，任何一位错都能看出来
    function [23:0] pix_of;
        input integer n;
        begin
            pix_of = {(n >> 8) & 8'hFF, n & 8'hFF, ((n >> 4) ^ n) & 8'hFF};
        end
    endfunction

    always @(posedge dst_clk) begin
        if (rst_n && cdc_wr_en) begin
            n_writes = n_writes + 1;
            if (cdc_wr_addr >= WORDS) begin
                n_oob = n_oob + 1;
                if (n_printed < 12) begin
                    $display("[OOB ] t=%0t addr=%0d data=%08h", $time, cdc_wr_addr, cdc_wr_data);
                    n_printed = n_printed + 1;
                end
            end else begin
                write_count[cdc_wr_addr] = write_count[cdc_wr_addr] + 8'd1;
                last_data[cdc_wr_addr]   = cdc_wr_data;
                if (write_count[cdc_wr_addr] > 8'd1) begin
                    n_dup = n_dup + 1;
                    if (n_printed < 12) begin
                        $display("[DUP ] t=%0t addr=%0d data=%08h (cnt=%0d)",
                                 $time, cdc_wr_addr, cdc_wr_data, write_count[cdc_wr_addr]);
                        n_printed = n_printed + 1;
                    end
                end
            end
        end
    end

    // ---- 驱动：整帧像素流 ----
    integer n;
    integer row, col;
    integer n_missing;
    integer n_misplaced;

    initial begin
        if ($test$plusargs("dump")) begin
            $display("(dump 未实现，全部结果在结束时汇总)");
        end

        repeat (10) @(posedge sd_clk);
        rst_n = 1'b1;
        repeat (5) @(posedge sd_clk);

        // 启动一帧（bank0）
        @(posedge sd_clk);
        load_start  <= 1'b1;
        image_index <= 2'd0;
        @(posedge sd_clk);
        load_start  <= 1'b0;
        repeat (5) @(posedge sd_clk);

        // 像素流：每 4 拍一个像素，模拟 SD 卡的真实字节节奏
        for (n = 0; n < WORDS; n = n + 1) begin
            pixel_valid <= 1'b1;
            pixel_rgb   <= pix_of(n);
            frame_commit <= (n == WORDS - 1);
            @(posedge sd_clk);
            pixel_valid <= 1'b0;
            frame_commit <= 1'b0;
            repeat (7) @(posedge sd_clk);   // 约 11M 字/s，与真实 SPI 供数速率同量级
        end

        // 等写完
        wait (load_done === 1'b1);
        $display("load_done @ %0t, memory_error=%02h", $time, memory_error);
        wait (cdc_idle === 1'b1);
        $display("cdc_idle   @ %0t", $time);

        // ---- 汇总 ----
        n_missing = 0;
        n_data_bad = 0;
        for (i = 0; i < WORDS; i = i + 1) begin
            if (write_count[i] == 8'd0) begin
                n_missing = n_missing + 1;
                if (n_missing <= 8)
                    $display("[MISS] addr=%0d (row=%0d col=%0d)", i, i / W, i % W);
            end
        end

        // 抽查数据：对每个地址反算它应该来自哪个像素
        n_misplaced = 0;
        for (i = 0; i < WORDS; i = i + 1) begin
            // 地址 i = mrow*W + mcol，对应输入行 row = H-1-mrow，col = mcol
            begin : chk
                integer mrow, mcol, src_n;
                reg [23:0] expect_pix;
                mrow = i / W;
                mcol = i % W;
                src_n = (H - 1 - mrow) * W + mcol;
                expect_pix = pix_of(src_n);
                if (last_data[i] !== {8'h00, expect_pix}) begin
                    n_data_bad = n_data_bad + 1;
                    if (n_data_bad <= 8)
                        $display("[DATA] addr=%0d(行%0d 列%0d) 应为 %08h 实为 %08h",
                                 i, mrow, mcol, {8'h00, expect_pix}, last_data[i]);
                end
            end
        end

        $display("=========================================================");
        $display(" 总写入次数      : %0d (期望 %0d)", n_writes, WORDS);
        $display(" 越界地址        : %0d", n_oob);
        $display(" 重复写的地址数  : %0d", n_dup);
        $display(" 从未写到的地址数: %0d", n_missing);
        $display(" 数据不符的地址数: %0d", n_data_bad);
        if (n_writes == WORDS && n_dup == 0 && n_oob == 0 &&
            n_missing == 0 && n_data_bad == 0)
            $display(" RESULT: PASS");
        else
            $display(" RESULT: FAIL");
        $display("=========================================================");
        $finish;
    end

    // 超时保护
    initial begin
        #200_000_000;
        $display("TIMEOUT: 仿真超时，n_writes=%0d", n_writes);
        $finish;
    end

endmodule
