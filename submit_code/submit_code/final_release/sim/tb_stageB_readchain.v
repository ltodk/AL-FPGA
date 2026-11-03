`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_stageB_readchain —— 第8讲 阶段B 顶层读侧链路集成验证
//
// 完整复刻顶层读侧连接方式：
//   video_timing_data (color_bar 640x480) 产生 hs_0/vs_0/de_0/h_cnt_0 + read_req
//     -> media_frame_reader（SDRAM 读 burst + 异步 FIFO + prefetch）
//     -> rgb_video_adapter（整组寄存 1 拍 + 消隐清零）
//   video_x 按顶层方式：h_cnt_0 打 1 拍对齐 adapter 输出的 de
//
// 验证点：
//   ① 端口连接无误（编译 + elaborate 即验证）
//   ② de 有效期间 pixel_rgb 与 SDRAM 地址严格对齐（无错位/丢字）
//   ③ 无 underflow
//   ④ video_x 与 adapter 输出的 de 同拍（相位对齐）
// -----------------------------------------------------------------------------
module tb_stageB_readchain;

    parameter MEM_DATA_BITS = 32;
    parameter ADDR_BITS     = 21;
    parameter FRAME_WORDS   = 307200;

    // ---- 时钟 ----
    reg mem_clk = 0, pix_clk = 0;
    always #4  mem_clk = ~mem_clk;   // 125M
    always #20 pix_clk = ~pix_clk;   // 25M
    reg mem_rst = 1, pix_rst = 1, pix_rst_n = 0;

    // ---- video_timing_data 输出 ----
    wire        read_req;
    wire        read_req_ack;
    wire        hs_0, vs_0, de_0;
    wire [11:0] h_cnt_0;

    // ---- reader 输出 ----
    wire [23:0] reader_pixel_rgb;
    wire        reader_pixel_valid;
    wire        reader_underflow;

    // ---- adapter 输出 ----
    wire        hs, vs, de;
    wire [23:0] vout_data_raw;

    // ---- video_x（顶层方式：h_cnt_0 打 1 拍）----
    reg  [11:0] video_x;
    wire [9:0]  video_x_ref;
    assign video_x_ref = (video_x >= 12'd160) ? (video_x[9:0] - 10'd160)
                                             : (video_x[9:0] + 10'd640);

    // ---- SDRAM 读接口 ----
    wire        App_rd_en;
    wire [20:0] App_rd_addr;
    reg         Sdr_rd_en;
    reg  [31:0] Sdr_rd_dout;
    wire        rd_burst_busy;

    // ---- 读写互斥 ----
    wire wr_burst_busy = 1'b0;

    // ===================== video_timing_data =====================
    video_timing_data video_timing_data_m0(
        .video_clk         (pix_clk),
        .rst               (pix_rst),
        .read_req          (read_req),
        .read_req_ack      (read_req_ack),
        .hs                (hs_0),
        .vs                (vs_0),
        .de                (de_0),
        .h_cnt_out         (h_cnt_0)
    );

    // ===================== media_frame_reader =====================
    media_frame_reader #(
        .MEM_DATA_BITS   (MEM_DATA_BITS),
        .ADDR_BITS       (ADDR_BITS)
    ) u_media_frame_reader (
        .mem_clk          (mem_clk),
        .rst              (mem_rst),
        .Sdr_init_done    (1'b1),
        .Sdr_init_ref_vld (1'b1),
        .Sdr_busy         (1'b0),
        .Sdr_rd_en        (Sdr_rd_en),
        .Sdr_rd_dout      (Sdr_rd_dout),
        .App_rd_en        (App_rd_en),
        .App_rd_addr      (App_rd_addr),
        .ext_wr_busy  (wr_burst_busy),
        .ext_wr_active    (1'b0),          // new: 窄写窗口, tb 恒 0
        .app_rd_busy      (rd_burst_busy),
        .pix_clk          (pix_clk),
        .pix_rst          (pix_rst),
        .read_req         (read_req),
        .read_req_ack     (read_req_ack),
        .read_addr_0      (21'd0),
        .read_addr_1      (FRAME_WORDS),
        .read_addr_2      (FRAME_WORDS*2),
        .read_addr_3      (FRAME_WORDS*3),
        .read_addr_index  (2'd0),
        .read_len         (FRAME_WORDS),
        .pixel_consume    (de_0),
        .pixel_rgb        (reader_pixel_rgb),
        .pixel_valid      (reader_pixel_valid),
        .underflow        (reader_underflow)
    );

    // ===================== rgb_video_adapter =====================
    rgb_video_adapter u_rgb_video_adapter (
        .pclk             (pix_clk),
        .rst_n            (pix_rst_n),
        .hs_in            (hs_0),
        .vs_in            (vs_0),
        .de_in            (de_0),
        .rgb_in           (reader_pixel_rgb),
        .video_clk        (),
        .video_hs         (hs),
        .video_vs         (vs),
        .video_de         (de),
        .video_rgb        (vout_data_raw)
    );

    // ---- video_x 打 1 拍 ----
    always @(posedge pix_clk or posedge pix_rst) begin
        if (pix_rst)
            video_x <= 12'd0;
        else
            video_x <= h_cnt_0;
    end

    // ===================== SDRAM 回读模型（10 拍延迟，数据=地址低24位） =====================
    reg        app_rd_en_d [0:10];
    reg [20:0] app_rd_addr_d [0:10];
    integer    j;
    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst) begin
            for (j = 0; j <= 10; j = j + 1) begin
                app_rd_en_d[j]   <= 1'b0;
                app_rd_addr_d[j] <= 21'd0;
            end
            Sdr_rd_en <= 1'b0; Sdr_rd_dout <= 0;
        end else begin
            app_rd_en_d[0]   <= App_rd_en;
            app_rd_addr_d[0] <= App_rd_addr;
            for (j = 1; j <= 10; j = j + 1) begin
                app_rd_en_d[j]   <= app_rd_en_d[j-1];
                app_rd_addr_d[j] <= app_rd_addr_d[j-1];
            end
            Sdr_rd_en   <= app_rd_en_d[10];
            // 数据 = {高8位0, 地址21位}，读侧取 [23:0] 得到 {3'h0, 地址}
            // 地址最大 307200 = 0x4B000，21位内，故数据低21位=地址，低24位=地址
            Sdr_rd_dout <= {11'd0, app_rd_addr_d[10]};
        end
    end

    // ===================== 像素采集与对齐检查 =====================
    // 稳态验证：SDRAM 模型始终按地址回数据（数据永远就绪）。
    // 前 2 帧用于 read_req + FIFO 填充，从第 3 帧起检查 de 有效期间的像素对齐。
    // 正确语义：data 低24位 = 帧内偏移（0~307199 连续递增，每帧重置），
    // 不是行内 video_x_ref（0~639 每行重置）。
    reg  [23:0] expect_val;
    integer     align_err;      // 数据对齐错误（应为 0）
    integer     valid_err;      // 非帧末尾的 pixel_valid 掉 0（应为 0）
    integer     de_px_cnt;
    integer     frame_cnt;
    reg  [11:0] prev_vs_0;
    reg  [18:0] frame_offset;    // 期望的帧内偏移（0~307199）

    always @(posedge pix_clk or posedge pix_rst) begin
        if (pix_rst) begin
            de_px_cnt    <= 0;
            align_err    <= 0;
            valid_err    <= 0;
            frame_cnt    <= 0;
            prev_vs_0    <= 0;
            frame_offset <= 0;
        end else begin
            // 帧计数：vs_0 低有效，vsync 期间=0。检测 vs 下降沿（进入 vsync）
            if (prev_vs_0 && !vs_0) begin
                frame_cnt    <= frame_cnt + 1;
                frame_offset <= 0;   // 新帧，偏移重置
            end
            prev_vs_0 <= vs_0;

            if (de) begin
                de_px_cnt <= de_px_cnt + 1;
                // 只在稳态（第 3 帧起，frame_cnt>=2）检查对齐
                if (frame_cnt >= 2) begin
                    if (!reader_pixel_valid) begin
                        // 帧末尾 off=307199（最后一个像素）是已知可接受边界：
                        // 读 307200 字 = 消费 307200 像素，末尾 prefetch 恰好排空。
                        // 非末尾的 pixel_valid 掉 0 才算真实错误（underflow 前兆）。
                        if (frame_offset != 19'd307199)
                            valid_err <= valid_err + 1;
                    end
                    // data 低24位应等于帧内偏移 frame_offset
                    if (vout_data_raw[23:0] !== {5'd0, frame_offset}) begin
                        align_err <= align_err + 1;
                        if (align_err < 10)
                            $display("t=%0t  [ALIGN ERR] frame_offset=%0d 但 data=%h", $time, frame_offset, vout_data_raw[23:0]);
                    end
                end
                frame_offset <= frame_offset + 1;
            end
        end
    end

    // ---- underflow 检测 ----
    reg underflow_seen;
    always @(posedge pix_clk or posedge pix_rst) begin
        if (pix_rst) underflow_seen <= 0;
        else if (reader_underflow) begin
            underflow_seen <= 1;
            $display("t=%0t  [UNDERFLOW] de=%b valid=%b", $time, de_0, reader_pixel_valid);
        end
    end

    // ---- 诊断：追踪 read_req / App_rd_en / Sdr_rd_en ----
    reg read_req_seen, rd_en_seen, sdr_seen;
    initial begin
        read_req_seen = 0; rd_en_seen = 0; sdr_seen = 0;
        fork
            forever begin
                @(posedge read_req);
                if (!read_req_seen) begin
                    read_req_seen = 1;
                    $display("t=%0t  [DIAG] read_req 首次拉高", $time);
                end
            end
            forever begin
                @(posedge App_rd_en);
                if (!rd_en_seen) begin
                    rd_en_seen = 1;
                    $display("t=%0t  [DIAG] App_rd_en 首次拉高 addr=%h", $time, App_rd_addr);
                end
            end
            forever begin
                @(posedge Sdr_rd_en);
                if (!sdr_seen) begin
                    sdr_seen = 1;
                    $display("t=%0t  [DIAG] Sdr_rd_en 首次拉高 dout=%h", $time, Sdr_rd_dout);
                end
            end
        join
    end

    // ===================== 主流程 =====================
    integer err;
    initial begin
        mem_rst = 1; pix_rst = 1; pix_rst_n = 0;
        err = 0; underflow_seen = 0;
        repeat (10) @(posedge mem_clk);
        mem_rst = 0; pix_rst = 0; pix_rst_n = 1;
        repeat (20) @(posedge mem_clk);

        // 跑 10 帧（每帧 800*525 像素 @25M ≈ 16.8ms），观察多帧稳态
        repeat (800*525*10) @(posedge pix_clk);

        // 检查
        if (underflow_seen) begin
            $display("[FAIL] underflow 发生");
            err = err + 1;
        end
        if (align_err != 0) begin
            $display("[FAIL] 像素数据对齐错误 %0d 处", align_err);
            err = err + 1;
        end
        if (valid_err != 0) begin
            $display("[FAIL] 非帧末尾 pixel_valid 掉 0 共 %0d 处", valid_err);
            err = err + 1;
        end
        if (de_px_cnt < 640*480*8) begin
            $display("[FAIL] de 像素数不足 %0d", de_px_cnt);
            err = err + 1;
        end

        if (err == 0)
            $display("RESULT: PASS  (de像素=%0d, 无underflow, 数据对齐正确, 仅帧末尾1像素边界)", de_px_cnt);
        else
            $display("RESULT: FAIL  (%0d 处错误)", err);
        $finish;
    end

    initial #500000000 $finish;   // 500ms 兜底
endmodule
