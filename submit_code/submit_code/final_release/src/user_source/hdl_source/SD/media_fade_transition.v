`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// media_fade_transition —— 图片切换转场控制器（第 10 讲）
//
// 功能：把"瞬间硬切"变成"糊到灰幕 → 换 bank → 化回原图"。
//       本模块只负责产生 alpha 斜坡，alpha 的**视觉含义由 fade_alpha_apply 决定**：
//         alpha = 255      → 原图（逐位旁路）
//         alpha = MIN_ALPHA → 最糊（fade_alpha_apply 把它解释为"几乎纯灰"）
//       注意 alpha 不再表示"亮度"，而是"原图权重"。早期版本用 alpha 做乘性压暗，
//       但编码域乘 0.25 实际发光只剩 4.8%（2.2 伽马），肉眼就是全黑，
//       所以改用"向灰阶收敛"，详见 fade_alpha_apply.v 头注释。
//
// ★ 与课程讲义（video_fade_transition.v）的核心差异
//   讲义是为双缓冲 + 需现场加载新图的架构写的，所以必须做跨时钟域握手：
//     控制域发 next_event → 像素域淡出到黑 → 回传 commit → 控制层真正切图
//   本工程是四 bank 帧缓存（第 8 讲），切图只是改一个指针、**零延迟**，
//   且 frame_fifo_read 只在每帧 S_ACK 时锁存一次 bank 基址（帧内只做地址递增），
//   也就是说"切哪个 bank"天然只在帧边界生效。
//   于是这里完全不需要控制域握手，只需要：
//       淡出到最糊档 → 在帧边界把"送给 reader 的 bank 索引"更新为新值 → 淡入
//   → 单时钟域（video_clk 25M）、无 toggle 握手、无 commit 回传，代码量约为讲义 1/3。
//
// ★ 为什么 alpha 只在帧边界更新
//   一帧 307200 像素。若帧内改 alpha，上半帧旧值、下半帧新值 → 一条水平分界线（撕裂）。
//   state/ramp 只在 frame_start（场同步下降沿）变化，而 alpha 由二者组合导出，
//   因此整帧 alpha 恒定。且 alpha 用组合导出（不用寄存器），
//   避免非阻塞赋值晚一拍造成的"淡出晚一帧才开始"。
//
// ★ alpha 的数学（本节只讲斜坡，像素处理见 fade_alpha_apply.v）
//   alpha 从 255 线性降到 MIN_ALPHA，再由 MIN_ALPHA 升回 255；
//   斜坡步数 FADE_FRAMES，每步 STEP = (256 - MIN_ALPHA) / FADE_FRAMES。
//
// ★ 为什么不再压到 alpha=0（纯黑）
//   早期版本淡到 0，中间"啪"一下全黑再亮，实机反馈"太刺眼"；
//   中途改成压到 64（编码 25%）仍然刺眼 —— 因为 2.2 伽马让 25% 编码只剩 4.8% 发光。
//   现在 alpha 到 MIN_ALPHA 就直接换 bank，像素侧向灰阶收敛而不是乘性压暗，
//   全程没有亮度暗谷。MIN_ALPHA=0 时行为与最早版本一致（SPAN=256，STEP 不变）。
//
// 端口命名沿用工程习惯：rst 高有效（与 osd_overlay 一致）。
// -----------------------------------------------------------------------------
module media_fade_transition #(
    // 淡出/淡入各占多少帧。★ (256 - MIN_ALPHA) 必须能被它整除，否则 STEP 取整后
    // 最后一级 alpha 跳不满 255，收尾会"顿一下"。
    // 与顶层 FADE_FRAMES_CFG 保持一致：8 = 全程约 0.35s。
    parameter integer FADE_FRAMES = 8,
    // 最糊档保留的原图权重（alpha 语义，不是亮度）：
    //   0   → 完全糊成纯灰（`(256-0)/8=32`，整除）
    //   16  → 保留 6.25% 原图 ← 采用（肉眼纯灰，但留一点层次，不像"凭空变灰"）
    //   48  → 保留 18.75%，能看出转场过程，但轮廓会露出来
    //   64  → 保留 25%，灰幕下偏"半透"
    // ★ S_HOLD（换 bank 那一刻）的 alpha 就是它，所以同时决定"跳变露不露馅"。
    // ★ 约束：(256 - MIN_ALPHA) 必须能被 FADE_FRAMES 整除。
    //   当前 8 帧 + 16 → 240/8 = 30，正好。改 FADE_FRAMES 时要一起复核。
    parameter integer MIN_ALPHA   = 16,
    parameter integer HOLD_FRAMES = 1     // 最糊档保持帧数（给读侧换 bank 留余量）
)(
    input  wire       clk,           // video_clk 25MHz（像素域）
    input  wire       rst,           // 高有效
    input  wire       vs,            // 场同步（来自 rgb_video_adapter）
    input  wire [1:0] disp_buf_idx,  // 上游请求的 bank（sd_card_bmp，100M 域）
    input  wire       enable,        // display_valid：无图时不介入

    output reg  [1:0] disp_buf_idx_q,// 实际送给 reader 的 bank（延迟到最糊档才更新）
    output wire [7:0] alpha,         // 255=原图（逐位旁路），MIN_ALPHA=最糊；视觉语义见 fade_alpha_apply.v
    output wire       fade_busy      // 转场进行中（供观测）
);

    // ---------------- 参数推导 ----------------
    // SPAN = 256 - MIN_ALPHA：需要跨越的 alpha 区间（不再是从 0 到 256）
    // STEP = SPAN/FADE_FRAMES：ramp 走满 FADE_FRAMES 时正好覆盖 MIN_ALPHA~255
    //   MIN_ALPHA=0 时 SPAN=256、STEP=256/FADE_FRAMES，与加 MIN_ALPHA 之前逐位一致。
    localparam integer SPAN = 256 - MIN_ALPHA;
    localparam integer STEP = SPAN / FADE_FRAMES;
    // ramp 需表示 0~FADE_FRAMES（含"请求落在帧中间"的未对齐值），故多一位
    localparam integer CNT_W = (FADE_FRAMES <= 2) ? 2 : $clog2(FADE_FRAMES + 1);
    // 把参数转成定宽常量，避免对 integer 参数做部分选择（部分工具不支持）
    localparam [CNT_W-1:0] RAMP_MAX   = FADE_FRAMES;
    localparam [7:0]       MIN_ALPHA_C = MIN_ALPHA;

    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_OUT  = 2'd1;   // 淡出
    localparam [1:0] S_HOLD = 2'd2;   // 最糊档保持 + 换 bank
    localparam [1:0] S_IN   = 2'd3;   // 淡入

    // ---------------- 跨时钟域：disp_buf_idx（100M -> 25M）----------------
    // 原设计把 disp_buf_idx 直接接到 frame_fifo_read（裸跨域，2 bit 多位翻转）。
    // 这里加两级同步 + 寄存器输出，顺带把这条路径变成规范 CDC。
    reg [1:0] idx_s0, idx_s1;
    reg       en_s0,  en_s1;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            idx_s0 <= 2'd0; idx_s1 <= 2'd0;
            en_s0  <= 1'b0; en_s1  <= 1'b0;
        end else begin
            idx_s0 <= disp_buf_idx;
            idx_s1 <= idx_s0;
            en_s0  <= enable;
            en_s1  <= en_s0;
        end
    end

    // ---------------- 帧起始：场同步下降沿 ----------------
    // 与 osd_overlay 行计数复位点是同一个沿：vs 落下 → 本帧第 0 行开始
    reg vs_d;
    always @(posedge clk or posedge rst) begin
        if (rst) vs_d <= 1'b0;
        else     vs_d <= vs;
    end
    wire frame_start = vs_d && !vs;

    // ---------------- 转场状态机 ----------------
    reg [1:0]       state;
    reg [CNT_W-1:0] ramp;
    reg [3:0]       hold_cnt;

    assign fade_busy = (state != S_IDLE);

    // ramp -> alpha：ramp 走满 FADE_FRAMES 时 alpha 恰为 255
    // （ramp*STEP 最大 = MIN_ALPHA + SPAN = 256，必须钳位，否则 [7:0] 会截成 0）
    wire [CNT_W+7:0] ramp_mul   = ramp * STEP;
    wire [7:0]       ramp_alpha = (ramp >= RAMP_MAX) ? 8'd255
                                                     : (MIN_ALPHA_C + ramp_mul[7:0]);

    // alpha 组合导出：state/ramp 只在 frame_start 变 → 整帧恒定，无撕裂
    // S_HOLD 用 MIN_ALPHA_C（不是 0）：换 bank 那一刻画面停在最糊档（灰幕），不是纯黑
    assign alpha = (state == S_IDLE) ? 8'd255 :
                   (state == S_HOLD) ? MIN_ALPHA_C :
                   ramp_alpha;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state          <= S_IDLE;
            ramp           <= {CNT_W{1'b0}};
            hold_cnt       <= 4'd0;
            disp_buf_idx_q <= 2'd0;
        end else begin
            if (!en_s1) begin
                // 无图（未提交首图 / 拔卡）：bank 索引直接跟随、不介入转场，
                // 保持与第 9 讲完全一致的上电/无卡行为
                state          <= S_IDLE;
                ramp           <= {CNT_W{1'b0}};
                hold_cnt       <= 4'd0;
                disp_buf_idx_q <= idx_s1;
            end else begin
                case (state)
                // -------- 空闲：检测切换请求 --------
                S_IDLE: begin
                    if (idx_s1 != disp_buf_idx_q) begin
                        // 上游已换指针（sd_card_bmp 零延迟切），但显示仍停在旧 bank
                        // → 先淡出，到最糊档再换
                        state <= S_OUT;
                        ramp  <= RAMP_MAX;
                    end
                end
                // -------- 淡出：alpha 255 -> MIN_ALPHA --------
                S_OUT: begin
                    if (frame_start) begin
                        if (ramp == {CNT_W{1'b0}}) begin
                            // 已到最糊档，帧边界换 bank（读侧下一帧 S_ACK 锁新基址）
                            state          <= S_HOLD;
                            disp_buf_idx_q <= idx_s1;
                            hold_cnt       <= HOLD_FRAMES[3:0];
                        end else begin
                            ramp <= ramp - 1'b1;
                        end
                    end
                end
                // -------- 最糊档保持：等读侧稳定拿到新 bank --------
                S_HOLD: begin
                    if (frame_start) begin
                        if (hold_cnt == 4'd0) begin
                            state <= S_IN;
                            ramp  <= {CNT_W{1'b0}};
                        end else begin
                            hold_cnt <= hold_cnt - 1'b1;
                        end
                    end
                end
                // -------- 淡入：alpha MIN_ALPHA -> 255 --------
                S_IN: begin
                    if (frame_start) begin
                        if (ramp >= RAMP_MAX)
                            state <= S_IDLE;      // 已全亮
                        else
                            ramp <= ramp + 1'b1;
                    end
                end
                default: begin
                    state <= S_IDLE;
                end
                endcase
            end
        end
    end

endmodule
