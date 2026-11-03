`timescale 1ns/1ps
// ============================================================================
// wav_async_fifo16.v —— WAV 音频字节流跨时钟域 FIFO（第 11 讲 阶段 B 新增）
//
// 位置：夹在「SD 卡侧（sd_card_clk 100MHz）」与「音频播放侧（video_clk 25MHz）」
//       之间，宽 16bit（正好一个采样样本），深 1024 字 = 2048 字节。
//
// 为什么是这个尺寸：
//   音频数据率 48kHz × 2B = 96KB/s → 2048 字节 = **21.3ms** 缓冲。
//   图片一个扇区约 0.35ms，即使图片连续抢 15 个扇区（5.3ms），缓冲也够；
//   1024×16bit = 16Kb → 2 个 9Kb BRAM，代价可接受。
//
// 为什么字宽取 16bit 而不是 8bit：
//   写侧（SD）给的是字节流，读侧（播放）要的是 16bit 小端样本。若 FIFO 按字节存，
//   读侧就得处理"读到奇数个字节要暂存半样本"的边界；按 16bit 存，写侧顺手把
//   两个字节拼成一个字（低字节先到 → 小端），读侧一次弹一个字就是一个样本，
//   两边都干净。奇数字节的文件末尾半个样本直接丢弃（PCM 文件必为偶数长度）。
//
// 结构：标准 gray 指针异步 FIFO
//   - 写指针（wr_clk 域）经 gray 编码后 2FF 同步到读域，用于产生 rd_empty；
//   - 读指针（rd_clk 域）经 gray 编码后 2FF 同步到写域，用于产生 wr_room；
//   - 指针位宽 AW+1，最高位区分"绕了一圈"，因此 gray 同步无需额外边界处理。
//
// ★ 读侧是「1 拍延迟」接口，不是组合输出
//   BRAM 端口的输出天然带一级寄存器。与其做看不见摸不着的组合读（会退化成
//   分布式 RAM 吃掉上千个 LUT），不如把这一拍延迟**显式化**：
//       rd_pop=1  ──1 拍后──>  rd_valid=1 且 rd_data 有效
//   读侧速率只有 48kHz（周期 20.8μs），1 拍延迟（40ns）完全无所谓。
//
// ★ wr_room 是写给 SD 侧做流控用的
//   SD 侧每读一个扇区要 512 字节 = 256 字，所以只要 wr_room >= 256 就可以安全
//   地再发一个扇区请求。它是"写域内的悲观估计"：读指针同步过来最多滞后 2 个
//   rd_clk，得到的 level 偏大、room 偏小 —— 只会少灌，不会溢出。
// ============================================================================
module wav_async_fifo16 #(
    parameter integer AW = 10            // 深度 = 2**AW 字；1024 字 = 2048 字节
)(
    // === 写侧（sd_card_clk 域）===
    input  wire            wr_clk,
    input  wire            wr_rst_n,
    input  wire            wr_en,
    input  wire [15:0]     wr_data,
    output wire [AW:0]     wr_room,      // 还能再写多少字（0 ~ 2**AW）

    // === 读侧（video_clk 域）===
    input  wire            rd_clk,
    input  wire            rd_rst_n,
    input  wire            rd_pop,       // 请求弹出一个字
    output reg  [15:0]     rd_data,      // rd_valid=1 时有效
    output reg             rd_valid,     // 单拍：rd_data 有效
    output wire            rd_empty,
    output wire [AW:0]     rd_level      // 当前可读字数（读域估计）
);

    localparam integer DEPTH = 1 << AW;

    reg [15:0] mem [0:DEPTH-1];

    reg [AW:0] wr_bin, wr_gray;
    reg [AW:0] rd_bin, rd_gray;
    reg [AW:0] rd_gray_s1, rd_gray_s2;   // 读指针 → 写域
    reg [AW:0] wr_gray_s1, wr_gray_s2;   // 写指针 → 读域

    // ---------------- gray 编解码 ----------------
    function [AW:0] bin2gray;
        input [AW:0] b;
        begin
            bin2gray = b ^ (b >> 1);
        end
    endfunction

    function [AW:0] gray2bin;
        input [AW:0] g;
        integer i;
        begin
            gray2bin[AW] = g[AW];
            for (i = AW-1; i >= 0; i = i - 1)
                gray2bin[i] = gray2bin[i+1] ^ g[i];
        end
    endfunction

    // ---------------- 写侧 ----------------
    wire [AW:0] wr_bin_next = wr_bin + 1'b1;

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_bin       <= {(AW+1){1'b0}};
            wr_gray      <= {(AW+1){1'b0}};
            rd_gray_s1   <= {(AW+1){1'b0}};
            rd_gray_s2   <= {(AW+1){1'b0}};
        end else begin
            rd_gray_s1 <= rd_gray;
            rd_gray_s2 <= rd_gray_s1;
            if (wr_en) begin
                wr_bin  <= wr_bin_next;
                wr_gray <= bin2gray(wr_bin_next);
            end
        end
    end

    always @(posedge wr_clk) begin
        if (wr_en)
            mem[wr_bin[AW-1:0]] <= wr_data;
    end

    // 写域占用量：写指针 - 已同步过来的读指针（悲观估计，room 只会偏小）
    wire [AW:0] wr_used = wr_bin - gray2bin(rd_gray_s2);
    assign wr_room = DEPTH - wr_used;

    // ---------------- 读侧 ----------------
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_bin     <= {(AW+1){1'b0}};
            rd_gray    <= {(AW+1){1'b0}};
            wr_gray_s1 <= {(AW+1){1'b0}};
            wr_gray_s2 <= {(AW+1){1'b0}};
            rd_data    <= 16'd0;
            rd_valid   <= 1'b0;
        end else begin
            wr_gray_s1 <= wr_gray;
            wr_gray_s2 <= wr_gray_s1;

            // 弹出：指针自增，同时把该地址的数据打一拍送出去
            rd_data  <= mem[rd_bin[AW-1:0]];
            rd_valid <= rd_pop && !rd_empty;

            if (rd_pop && !rd_empty) begin
                rd_bin  <= rd_bin + 1'b1;
                rd_gray <= bin2gray(rd_bin + 1'b1);
            end
        end
    end

    // ★ 可读字数 = 同步过来的写指针(解 gray 后) − 本域读指针(本身就是二进制)
    assign rd_empty = (rd_gray == wr_gray_s2);
    assign rd_level = gray2bin(wr_gray_s2) - rd_bin;

endmodule
