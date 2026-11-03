`timescale 1ns/1ps
// =============================================================================
// media_frame_loader —— 四 bank 帧缓存 · 帧加载器（第8讲）
//
// 职责：
//   接收 BMP 解码器输出的像素流（pixel_valid / pixel_rgb），先写入 512×32 的
//   同步 FIFO 吸收"解码速率 ≠ SDRAM 写入速率"的抖动，攒够 128 字（或收到
//   frame_commit）后发起一次连续 burst 写，落到 image_index 指定的 bank。
//
// 相对讲义参考代码的三处工程适配（必改，否则在我校工程上跑不对）：
//   1) WRITE_V_FLIP —— BMP 文件是自下而上存储的（文件第一行 = 屏幕最底行），
//      原来的 frame_read_write/frame_fifo_write 用行地址倒序把它翻正。四 bank
//      架构里 loader 只做线性递增，必须在这里补上同样的翻转，否则整幅图上下颠倒。
//        写第 n 个字（n = row*W + col）→ 目标偏移 = (H-1-row)*W + col
//        即：起始偏移 (H-1)*W，行内 +1，行末跳到 -(2W-1)
//   2) app_wr_data = {8'h00, pixel_rgb} —— RGB 放低 24 位，与 reader 的
//      prefetch_data[23:0] 严格对应。官方老桥接是 {R,G,B,8'b0} 放高 24 位，
//      两者不可混用，否则颜色通道整体错位。
//   3) bank 基址在 load_start 时预锁存进 bank_offset，写地址只做一级加法，
//      避免 case 选择 + 加法串在一拍里。
//
// 错误码（6 个）：
//   0x41 ERR_NOT_ACTIVE    未激活时收到像素
//   0x42 ERR_FIFO_OVERFLOW FIFO 溢出
//   0x43 ERR_PIXEL_COUNT   像素数超过 IMAGE_WORDS
//   0x44 ERR_DECODE        解码错误
//   0x45 ERR_SHORT_FRAME   像素数不足（短帧）
//   0x46 ERR_WRITE_STALL   写超时（FIFO 有数据但发不出 burst）
// =============================================================================
module media_frame_loader #(
    parameter integer ADDR_WIDTH    = 21,
    parameter integer IMAGE_WORDS   = 307200,   // 640*480
    parameter integer FIFO_DEPTH    = 512,
    parameter integer BURST_SIZE    = 128,
    parameter integer STALL_TIMEOUT = 65535,
    parameter integer WRITE_V_FLIP  = 1,        // 1: 行地址倒序，抵消 BMP 自下而上
    parameter integer FRAME_WIDTH   = 640,
    parameter integer FRAME_HEIGHT  = 480,
    // 空闲超时：active 但长时间既收不到像素也发不出数据（例如上层加载失败、
    // write_req 被撤销而像素永远不来）时强制退出，避免卡在 active 态。
    // 必须大于 sd_card_bmp 自身的 3s 加载兜底，让它先超时重试。
    parameter integer IDLE_TIMEOUT_CYCLES = 600_000_000,  // 6s @100MHz
    // 【2026-10-28 噪点排查】四帧整体平移量（字）。
    //   顶层的 BUF0..3_ADDR 必须用**同一个值**，否则读写基址不一致 → 画面错位。
    //   用途：验证"噪点分层边界 = SDRAM 内部 bank(BA) 边界(0x80000/0x100000)"。
    parameter integer BASE_OFFSET   = 0,
    // 【第13讲】上游"端出数据但我收不下"的容忍拍数。
    //   见下方 ERR_FIFO_OVERFLOW 分支：AXI-Stream 语义下上游会一直握着 valid
    //   等 ready，这类握手最多容忍一段时间；连续超过这个拍数才算真洪水。
    //   1024 拍 ≈ 10us @100MHz，远大于任何正常的 FIFO 排空间隔。
    parameter integer OVF_TOLERANCE = 1024
) (
    input  wire                  clk,               // 与 BMP 解码器同域（sd_card_clk）
    input  wire                  rst_n,
    input  wire                  load_start,        // 单拍启动脉冲
    input  wire [1:0]            image_index,       // 目标 bank（0~3）
    input  wire                  pixel_valid,       // 像素有效
    input  wire [23:0]           pixel_rgb,         // RGB888（R 在高位）
    input  wire                  frame_commit,      // 一帧像素全部输出后的单拍脉冲
    input  wire [7:0]            decode_error,      // 来自 BMP 解码器的错误码
    output wire                  load_busy,
    output reg                   load_done,         // 加载完成单拍脉冲
    output reg  [3:0]            image_complete,    // 各 bank 是否已加载完成（粘滞）
    output reg  [7:0]            memory_error,
    input  wire                  controller_init_done,
    input  wire                  controller_busy,
    input  wire                  refresh_active,
    output reg                   app_wr_en,
    output reg  [ADDR_WIDTH-1:0] app_wr_addr,
    output reg  [3:0]            app_wr_mask,
    output reg  [31:0]           app_wr_data,
    // 下游（跨时钟域搬运 FIFO）已满：暂停出队，数据留在本模块 FIFO 里。
    // 没有这条反压时，一旦下游 FIFO 满，这里输出的字会被静默丢弃，
    // 而 pixels_written 照常自增 → 那些地址永远写不到 → 画面出现横条。
    input  wire                  out_full,
    // 【第13讲新增】本模块当前能否接收一个新像素（= FIFO 有空位或本拍能出队）。
    //   给写侧缩放器（bmp_scale_640x480）做反压用：垂直放大时要重复输出整行，
    //   若不加反压就会以 100MHz 猛灌，把 512 深 FIFO 冲爆并触发
    //   ERR_FIFO_OVERFLOW(0x42)。加上之后缩放器按需输出，速率自动匹配 SDRAM。
    //
    //   ★★★ 上游契约（很容易踩，第 13 讲 B 轮实机因此报废过一整版）：
    //     pixel_valid **不是** AXI-Stream 的 valid。下方
    //       push_allowed = pixel_ready && pixel_valid;
    //       else if (pixel_valid && !push_allowed) fail_load(ERR_FIFO_OVERFLOW);
    //     把"valid 为高但收不下"直接当成溢出故障，**一个周期就 abort 整帧**。
    //     所以上游必须保证：pixel_ready 为低的每一拍，pixel_valid 都是 0。
    //     即"valid 与 ready 必须同拍成立"，而不是"valid 拉高后握到 ready 为止"。
    //     改上游时若用标准握手写法，实机表现为闪 ERR:42 + 加载反复失败。
    output wire                  pixel_ready
);
    localparam [7:0] ERR_NONE          = 8'h00;
    localparam [7:0] ERR_NOT_ACTIVE    = 8'h41;
    localparam [7:0] ERR_FIFO_OVERFLOW = 8'h42;
    localparam [7:0] ERR_PIXEL_COUNT   = 8'h43;
    localparam [7:0] ERR_DECODE        = 8'h44;
    localparam [7:0] ERR_SHORT_FRAME   = 8'h45;
    localparam [7:0] ERR_WRITE_STALL   = 8'h46;
    localparam [7:0] ERR_IDLE_TIMEOUT  = 8'h47;

    localparam integer PTR_WIDTH = (FIFO_DEPTH <= 2) ? 1 : $clog2(FIFO_DEPTH);
    localparam [PTR_WIDTH:0] EFFECTIVE_BURST =
        (BURST_SIZE < FIFO_DEPTH) ? BURST_SIZE : FIFO_DEPTH;

    // ★ BASE_OFFSET 必须与顶层 BUF0..3_ADDR 用的同一个值保持一致
    localparam [ADDR_WIDTH-1:0] BANK0_BASE = BASE_OFFSET;
    localparam [ADDR_WIDTH-1:0] BANK1_BASE = BASE_OFFSET + IMAGE_WORDS;
    localparam [ADDR_WIDTH-1:0] BANK2_BASE = BASE_OFFSET + IMAGE_WORDS * 2;
    localparam [ADDR_WIDTH-1:0] BANK3_BASE = BASE_OFFSET + IMAGE_WORDS * 3;

    // ---- 垂直翻转常量（与老 frame_fifo_write 的 VFLIP_* 完全一致）----
    localparam [ADDR_WIDTH-1:0] VFLIP_FIRST_OFFSET = (FRAME_HEIGHT - 1) * FRAME_WIDTH;
    localparam [ADDR_WIDTH-1:0] VFLIP_LINE_JUMP    = (FRAME_WIDTH * 2) - 1;
    localparam [15:0]           LINE_LAST_X        = FRAME_WIDTH - 1;

    reg        active;
    reg [1:0]  active_image;
    reg        commit_seen;
    reg [31:0] pixels_received;
    reg [31:0] pixels_written;
    reg [31:0] stall_count;
    reg [15:0] ovf_wait;                // 上游 valid 悬空容忍计数（见 0x42 分支）
    reg [31:0] idle_cnt;                // 空闲看门狗（既无入队也无出队）
    reg [31:0] fifo_mem [0:FIFO_DEPTH-1];
    reg [PTR_WIDTH-1:0] wr_ptr;
    reg [PTR_WIDTH-1:0] rd_ptr;
    reg [PTR_WIDTH:0]   fifo_count;
    reg        burst_active;
    reg [PTR_WIDTH:0] burst_remaining;

    // ---- 写地址生成（V_FLIP 版）----
    reg [ADDR_WIDTH-1:0] bank_offset;   // load_start 时锁存的 bank 基址
    reg [ADDR_WIDTH-1:0] flip_addr;     // 相对 bank 的写偏移（翻转映射后）
    reg [15:0]           line_x;        // 当前行内列号 0..FRAME_WIDTH-1

    wire pop_allowed;
    wire push_allowed;
    wire start_burst;
    wire [PTR_WIDTH:0] next_burst_words;

    assign load_busy = active;

    // SDRAM 控制器内部处理 busy/refresh，这两类状态不是"每字就绪"信号，
    // 不能用它们打断一个已经发起连续 burst。
    assign start_burst = active && controller_init_done && !burst_active &&
                         fifo_count != 0 &&
                         (fifo_count >= EFFECTIVE_BURST || commit_seen);
    assign next_burst_words = (fifo_count >= EFFECTIVE_BURST) ?
                              EFFECTIVE_BURST : fifo_count;
    assign pop_allowed = active && controller_init_done && burst_active &&
                         fifo_count != 0 && !out_full;
    // 「能否再收一个像素」：FIFO 有空位，或本拍正好能出队腾出一个位置
    assign pixel_ready = active && (fifo_count < FIFO_DEPTH || pop_allowed);
    assign push_allowed = pixel_ready && pixel_valid;

    function [ADDR_WIDTH-1:0] bank_base;
        input [1:0] index;
        begin
            case (index)
                2'd0:    bank_base = BANK0_BASE;
                2'd1:    bank_base = BANK1_BASE;
                2'd2:    bank_base = BANK2_BASE;
                default: bank_base = BANK3_BASE;
            endcase
        end
    endfunction

    task fail_load;
        input [7:0] code;
        begin
            memory_error   <= code;
            active         <= 1'b0;
            burst_active   <= 1'b0;
            burst_remaining<= 0;
            idle_cnt       <= 32'd0;
            load_done      <= 1'b1;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active          <= 1'b0;
            active_image    <= 2'd0;
            commit_seen     <= 1'b0;
            pixels_received <= 32'd0;
            pixels_written  <= 32'd0;
            stall_count     <= 32'd0;
            ovf_wait        <= 16'd0;
            idle_cnt        <= 32'd0;
            wr_ptr          <= {PTR_WIDTH{1'b0}};
            rd_ptr          <= {PTR_WIDTH{1'b0}};
            fifo_count      <= {(PTR_WIDTH+1){1'b0}};
            burst_active    <= 1'b0;
            burst_remaining <= 0;
            bank_offset     <= {ADDR_WIDTH{1'b0}};
            flip_addr       <= {ADDR_WIDTH{1'b0}};
            line_x          <= 16'd0;
            load_done       <= 1'b0;
            image_complete  <= 4'd0;
            memory_error    <= ERR_NONE;
            app_wr_en       <= 1'b0;
            app_wr_addr     <= {ADDR_WIDTH{1'b0}};
            app_wr_mask     <= 4'b0000;
            app_wr_data     <= 32'd0;
        end else begin
            load_done <= 1'b0;
            app_wr_en <= 1'b0;

            // 新的 load_start 无条件抢占：否则一旦上层加载失败把 write_req 撤掉、
            // 像素永远不会来，loader 会永久停在 active，后续加载全部被 !active 挡掉。
            if (load_start) begin
                active          <= 1'b1;
                active_image    <= image_index;
                commit_seen     <= 1'b0;
                pixels_received <= 32'd0;
                pixels_written  <= 32'd0;
                stall_count     <= 32'd0;
                ovf_wait        <= 16'd0;
                idle_cnt        <= 32'd0;
                wr_ptr          <= {PTR_WIDTH{1'b0}};
                rd_ptr          <= {PTR_WIDTH{1'b0}};
                fifo_count      <= {(PTR_WIDTH+1){1'b0}};
                burst_active    <= 1'b0;
                burst_remaining <= 0;
                memory_error    <= ERR_NONE;
                bank_offset     <= bank_base(image_index);
                flip_addr       <= WRITE_V_FLIP ? VFLIP_FIRST_OFFSET
                                                : {ADDR_WIDTH{1'b0}};
                line_x          <= 16'd0;
                case (image_index)
                    2'd0:    image_complete[0] <= 1'b0;
                    2'd1:    image_complete[1] <= 1'b0;
                    2'd2:    image_complete[2] <= 1'b0;
                    default: image_complete[3] <= 1'b0;
                endcase
            end

            if (active) begin
                // 空闲看门狗：既没有像素进队、也没有数据出队，说明上游解码停了
                // 或下游 SDRAM 一直不初始化。超时退出，避免永久占用 active。
                if (push_allowed || pop_allowed)
                    idle_cnt <= 32'd0;
                else if (idle_cnt < IDLE_TIMEOUT_CYCLES)
                    idle_cnt <= idle_cnt + 1'b1;

                if (idle_cnt >= IDLE_TIMEOUT_CYCLES) begin
                    fail_load(ERR_IDLE_TIMEOUT);
                end else if (decode_error != 8'd0) begin
                    fail_load(ERR_DECODE);
                end else if (pixel_valid && pixels_received >= IMAGE_WORDS) begin
                    fail_load(ERR_PIXEL_COUNT);
                end else if (pixel_valid && !push_allowed) begin
                    // =====================================================
                    // 上游端出了数据但我这拍收不下。
                    //
                    // ★ 第 13 讲 B 轮修正（10-26）：原来这里**一拍就 fail_load**，
                    //   于是"标准 AXI-Stream 握手"（valid 拉高后握到 ready 为止）
                    //   会在 ready 出现任何一个低谷时把整帧 abort 掉 —— 实机症状
                    //   就是闪 ERR:42、切图反复失败、最后退化成 0x38 全栈重扫。
                    //
                    //   改为**容忍窗口**：上游若真在"握着等"，等 ready 一升，
                    //   下一个 ready 拍就会走 push_allowed 把它收进来，一个像素
                    //   都不会丢；只有连续 OVF_TOLERANCE 拍都收不下，才说明上游
                    //   是真洪水（每拍都推进新像素），此时才判 0x42。
                    //
                    //   这样即便将来又有人接了个不合契约的上游，最坏也只是"慢"，
                    //   而不是"整帧报废 + 一分钟不能切图"。
                    // =====================================================
                    if (ovf_wait >= OVF_TOLERANCE)
                        fail_load(ERR_FIFO_OVERFLOW);
                    else
                        ovf_wait <= ovf_wait + 1'b1;
                end else begin
                    if (push_allowed)
                        ovf_wait <= 16'd0;
                    if (start_burst) begin
                        burst_active    <= 1'b1;
                        burst_remaining <= next_burst_words;
                    end

                    // ---- 入队：RGB 放低 24 位 ----
                    if (push_allowed) begin
                        fifo_mem[wr_ptr] <= {8'h00, pixel_rgb};
                        wr_ptr           <= wr_ptr + 1'b1;
                        pixels_received  <= pixels_received + 1'b1;
                    end

                    // ---- 出队：写 SDRAM + V_FLIP 地址递推 ----
                    if (pop_allowed) begin
                        app_wr_en   <= 1'b1;
                        app_wr_addr <= bank_offset +
                                       (WRITE_V_FLIP ? flip_addr
                                                     : pixels_written[ADDR_WIDTH-1:0]);
                        app_wr_data <= fifo_mem[rd_ptr];
                        app_wr_mask <= 4'b0000;   // dm=0 表示四字节全写
                        rd_ptr          <= rd_ptr + 1'b1;
                        pixels_written  <= pixels_written + 1'b1;
                        stall_count     <= 32'd0;
                        if (WRITE_V_FLIP) begin
                            if (line_x == LINE_LAST_X) begin
                                line_x    <= 16'd0;
                                flip_addr <= flip_addr - VFLIP_LINE_JUMP;
                            end else begin
                                line_x    <= line_x + 16'd1;
                                flip_addr <= flip_addr + 1'b1;
                            end
                        end
                        if (burst_remaining == 1) begin
                            burst_active    <= 1'b0;
                            burst_remaining <= 0;
                        end else begin
                            burst_remaining <= burst_remaining - 1'b1;
                        end
                    end else if (fifo_count != 0 &&
                                 (commit_seen || fifo_count >= EFFECTIVE_BURST)) begin
                        // FIFO 有数据却发不出 burst：累计超时
                        if (stall_count == STALL_TIMEOUT)
                            fail_load(ERR_WRITE_STALL);
                        else
                            stall_count <= stall_count + 1'b1;
                    end else begin
                        stall_count <= 32'd0;
                    end

                    case ({push_allowed, pop_allowed})
                        2'b10:   fifo_count <= fifo_count + 1'b1;
                        2'b01:   fifo_count <= fifo_count - 1'b1;
                        default: fifo_count <= fifo_count;
                    endcase

                    if (frame_commit)
                        commit_seen <= 1'b1;

                    // ---- 完成判断：收到的和写出的都必须是整帧 ----
                    if (commit_seen && fifo_count == 0 && !pop_allowed &&
                        !burst_active) begin
                        if (pixels_received != IMAGE_WORDS ||
                            pixels_written  != IMAGE_WORDS) begin
                            fail_load(ERR_SHORT_FRAME);
                        end else begin
                            case (active_image)
                                2'd0:    image_complete[0] <= 1'b1;
                                2'd1:    image_complete[1] <= 1'b1;
                                2'd2:    image_complete[2] <= 1'b1;
                                default: image_complete[3] <= 1'b1;
                            endcase
                            active    <= 1'b0;
                            load_done <= 1'b1;
                        end
                    end
                end
            end else if (pixel_valid && memory_error == ERR_NONE) begin
                // 只在"还没有任何错误"时记录：否则一次真实故障（如 0x46 写停滞）
                // 会被后面漂来的像素改写成 0x41，把真正的错误码掩盖掉，实机
                // 只能看到 0x41 而查不出根因。首个错误最有诊断价值。
                memory_error <= ERR_NOT_ACTIVE;
            end
        end
    end
endmodule
