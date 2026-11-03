`timescale 1ns/1ps
// ============================================================================
// wav_progress.v —— 第11讲阶段C：音频播放进度换算（字节数 → 秒）
//
// 职责：把 `fat32_file_streamer` 的文件内字节偏移（file_offset）和文件大小
//       （file_size）换算成"秒"，供 OSD 显示 `POS 12/180s`。
//
// ★ 为什么需要它、又为什么要自己写一个除法
//   秒 = 字节 / 96000（48000 样本/s × 2 字节/样本）。任何单位换算都躲不开
//   一次除法。这里用**一次性移位减法（restoring division）**，不做流水线：
//   32 拍算完一个商 @100MHz = 320ns，而它服务的量（file_offset）每 5.33ms
//   才变一次（一个扇区），余量有 16000 倍，完全不需要并行除法器/乘法器。
//   实测综合代价：18 位比较+减法 + 32 位商移位寄存器，约一百个 LUT、零 DSP。
//
// ★ 一个除法器算两个数（时分复用）
//   交替计算：第 1 轮算 pos = file_offset/BPS，第 2 轮算 tot = file_size/BPS，
//   一轮 32 拍、两轮 66 拍 ≈ 660ns。合起来只用一份硬件。
//   除法开始时把被除数锁进 `dividend`，所以中途 file_offset 变化不会污染结果。
//
// ★ 约束：BYTES_PER_SEC 必须 < 2^17（=131072），否则 18 位余数会溢出。
//   当前 96000 满足；改大时要同步加宽 RW。
//
// 复位：低有效 rst_n。
// ============================================================================
module wav_progress #(
    parameter integer BYTES_PER_SEC = 96000,    // 48000 样本/s × 2 字节/样本
    parameter integer MAX_SEC       = 999       // 显示上限（3 位十进制）
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        size_stable,     // file_size 可信（扫描完成且找到音频）
    input  wire [31:0] file_size,       // 文件总字节数
    input  wire [31:0] file_offset,     // 当前已读字节数（0..file_size）
    output reg  [9:0]  pos_sec,         // 已播秒数（0..MAX_SEC）
    output reg  [9:0]  tot_sec          // 总秒数（0..MAX_SEC）
);

    // 余数位宽：BPS < 2^17 → 2*BPS < 2^18
    localparam integer RW = 18;

    reg  [5:0]  cnt;
    reg         busy;
    reg         sel;            // 0 = 正在算 pos，1 = 正在算 tot
    reg  [31:0] dividend;
    reg  [RW-1:0] rem;
    reg  [31:0] quo;

    // 一拍做一位的移位减法
    wire [RW-1:0] rem_sh = {rem[RW-2:0], dividend[31]};
    wire          ge     = (rem_sh >= BYTES_PER_SEC[RW-1:0]);
    wire [RW-1:0] rem_nx = ge ? (rem_sh - BYTES_PER_SEC[RW-1:0]) : rem_sh;
    wire [31:0]   quo_nx = {quo[30:0], ge};

    // 结果限幅到 MAX_SEC（不足补 0，超出截顶）
    wire [9:0] lim = (quo_nx > MAX_SEC) ? MAX_SEC[9:0] : quo_nx[9:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt      <= 6'd0;
            busy     <= 1'b0;
            sel      <= 1'b0;
            dividend <= 32'd0;
            rem      <= {RW{1'b0}};
            quo      <= 32'd0;
            pos_sec  <= 10'd0;
            tot_sec  <= 10'd0;
        end else if (!busy) begin
            if (!sel) begin
                // 第 1 轮：算 pos
                dividend <= file_offset;
                rem      <= {RW{1'b0}};
                quo      <= 32'd0;
                cnt      <= 6'd32;
                busy     <= 1'b1;
            end else if (size_stable) begin
                // 第 2 轮：算 tot（file_size 不可信时跳过）
                dividend <= file_size;
                rem      <= {RW{1'b0}};
                quo      <= 32'd0;
                cnt      <= 6'd32;
                busy     <= 1'b1;
            end else begin
                sel <= 1'b0;    // 没有 total 可算，直接回到 pos
            end
        end else begin
            rem      <= rem_nx;
            quo      <= quo_nx;
            dividend <= {dividend[30:0], 1'b0};
            cnt      <= cnt - 6'd1;
            if (cnt == 6'd1) begin
                busy <= 1'b0;
                sel  <= ~sel;
                if (sel) tot_sec <= lim;
                else     pos_sec <= lim;
            end
        end
    end

endmodule
