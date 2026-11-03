`timescale 1ns/1ps
// =============================================================================
// media_write_commit —— 写通道的"整帧提交"判定（第8讲）
//
// 职责（只做三件事，全部在 sd_card_clk 域）：
//   1) 数像素：从 load_start 起累计 pixel_valid，数满 FRAME_PIXELS 个后
//      产生一拍 frame_commit，送给 media_frame_loader 收尾。
//      不能用"文件读完"代替——最后几个像素那时还没进 loader，会被判短帧。
//   2) 判落地：loader 的 load_done 只表示"像素都进了 CDC"，数据可能还在
//      125M 域排队。必须再等 cdc_idle 才认为整帧真正写进 SDRAM，
//      否则上层会切到新 bank 去看一张正在写的图（花屏）。
//   3) 出 toggle：每成功提交一帧，frame_write_toggle 翻转一次，
//      作为跨帧累积的写完成握手信号给 sd_card_bmp。
//
// 【历史坑，务必保留】早先把"待翻计数"写成
//     else if (cnt != 0 && cdc_idle) toggle <= ~toggle;
//   每个时钟周期都翻一次，cnt=2 时翻两次互相抵消，上层永远收不到
//   写完成 → 500ms 超时 → ERR:0x38（切不了图）。现在严格 1 帧 1 翻。
//
// 【第二个坑】cdc_idle 是两级同步后的旧值，load_done 那一刻 FIFO 里其实
//   还有几百个像素，同步链上却仍是"上一帧早已空闲"的 1。若直接判定，
//   toggle 会提前翻转。故 load_done 后强制等待 DRAIN_GUARD_CYCLES 拍。
// =============================================================================
module media_write_commit #(
    parameter [18:0] FRAME_PIXELS       = 19'd307200,
    parameter [7:0]  DRAIN_GUARD_CYCLES = 8'd255
) (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       load_start,      // 新帧启动（与 loader 的 load_start 同源）
    input  wire       pixel_valid,     // 像素有效，每拍最多 1 个
    input  wire       load_done,       // loader 整帧完成（单拍）
    input  wire [7:0] memory_error,    // loader 错误码，非 0 表示本帧失败
    input  wire       cdc_idle,        // CDC 已排空（已同步到本时钟域）

    output reg        frame_commit,    // 第 FRAME_PIXELS 个像素之后打一拍
    output reg        frame_write_toggle
);

    reg [18:0] px_cnt;
    reg        commit_done;

    // ------------------------------------------------------------------
    // 像素计数 → frame_commit
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            px_cnt      <= 19'd0;
            commit_done <= 1'b0;
            frame_commit<= 1'b0;
        end else begin
            frame_commit <= 1'b0;
            if (load_start) begin
                px_cnt      <= 19'd0;
                commit_done <= 1'b0;
            end else if (pixel_valid) begin
                if (px_cnt == FRAME_PIXELS - 19'd1) begin
                    if (!commit_done) begin
                        frame_commit <= 1'b1;      // 最后一个像素送出的同拍打提交
                        commit_done  <= 1'b1;
                    end
                end else begin
                    px_cnt <= px_cnt + 19'd1;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // 排空判定 → toggle
    // ------------------------------------------------------------------
    reg [7:0] drain_guard;
    reg       frame_pending;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drain_guard        <= 8'd0;
            frame_pending      <= 1'b0;
            frame_write_toggle <= 1'b0;
        end else begin
            // load_done 后的强制等待窗口，覆盖 CDC 侧 idle 拉低 + 两级同步延迟
            if (load_done)
                drain_guard <= DRAIN_GUARD_CYCLES;
            else if (drain_guard != 8'd0)
                drain_guard <= drain_guard - 8'd1;

            if (load_done) begin
                // 失败帧（短帧/超时/解码错）不置 pending：让上层走超时错误路径，
                // 而不是把半帧当成功提交。
                if (memory_error == 8'd0)
                    frame_pending <= 1'b1;
            end else if (frame_pending && drain_guard == 8'd0 && cdc_idle) begin
                frame_pending      <= 1'b0;
                frame_write_toggle <= ~frame_write_toggle;
            end
        end
    end

endmodule
