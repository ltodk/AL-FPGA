`timescale 1ns/1ps
// ============================================================================
// wav_sector_feeder.v —— WAV 文件的「SD 侧供给器」（第 11 讲 阶段 B 新增）
//
// 职责：把「一个音频文件的簇链」变成「不断流的 16bit 样本字」，写进跨时钟域 FIFO。
//       它内部就是一个 fat32_file_streamer 实例（复用第 7 讲已验证的簇链遍历、
//       环检测、提前 EOF 检测），外面加三件事：
//         ① 字节 → 16bit 小端样本的拼装；
//         ② 流控：FIFO 剩余空间不足一个扇区时暂停发扇区请求；
//         ③ 起停与循环：play_en 上升启动，播完按 LOOP_EN 决定重头再来还是停。
//
// ★ 为什么流控要做在「扇区边界」而不是「字节边界」
//   扇区内的 512 字节是 SD 侧按 SPI 时钟连续吐出来的，中途停就等于丢字节；
//   而扇区与扇区之间本来就是状态机的一个空档（ST_DATA_REQ 等 !sector_busy），
//   在那里停一分钱额外开销都没有。音频 FIFO 有 2048 字节（21ms），
//   以扇区为粒度做流控完全够用。
//
// ★ 为什么整首歌要能被「随时掐断」
//   play_en 掉下来说明上层不要播了（拔卡 / 重扫 / 关闭 WAV）。此时立刻 abort
//   内部的 streamer，让它回 IDLE，绝不能让它停在 DATA_WAIT 等一个不会来的
//   sector_done —— 那正是第 5 讲踩过的死锁坑，这里从一开始就堵住。
//
// 复位：低有效 rst_n；abort 输入由上层（sd_card_bmp）在整栈软复位时拉高一拍。
// ============================================================================
module wav_sector_feeder #(
    parameter integer LOOP_EN          = 1,             // 1 = 单曲循环
    parameter integer LOOP_GAP_CYCLES  = 5_000,         // 循环接缝留 50μs 空档
    parameter integer RETRY_GAP_CYCLES = 10_000_000,    // 出错后 100ms 重试
    parameter integer FIFO_ROOM_MIN    = 256            // 一个扇区 = 256 字
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        abort,

    // === 播放控制与文件信息（来自 scanner）===
    input  wire        play_en,             // 电平：1 = 应该正在放这首
    input  wire [31:0] first_cluster,
    input  wire [31:0] file_size,
    input  wire [7:0]  sectors_per_cluster,
    input  wire [2:0]  sectors_per_cluster_shift,
    input  wire [31:0] fat_start_lba,
    input  wire [31:0] data_start_lba,
    input  wire [31:0] max_cluster,

    // === 输出到跨时钟域 FIFO（写侧，本时钟域）===
    input  wire [12:0] fifo_room,           // 还能写多少字
    output reg         fifo_wr_en,
    output reg  [15:0] fifo_wr_data,

    // === 状态（诊断 / UART）===
    output wire        busy,
    output reg         done_pulse,          // 整首播完（循环模式下也会打一拍）
    output reg  [7:0]  fs_error,
    // ★ 第11讲阶段C：文件内字节偏移（0..file_size），供 OSD 播放进度换算
    //   注意它反映的是"已读进 FIFO 的字节数"，比实际出声领先 ≤21ms（FIFO 深度），
    //   对 1 秒分辨率的进度显示完全不可见；循环/重启时自动归零。
    output wire [31:0] file_offset_out,

    // === 扇区读接口（接 sd_sector_arbiter 的客户端 2）===
    output wire        sector_req,
    output wire [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);

    localparam [1:0] S_IDLE  = 2'd0;    // 等待 play_en / 等待重试
    localparam [1:0] S_START = 2'd1;    // 打一拍 file_start
    localparam [1:0] S_RUN   = 2'd2;    // 正在读文件
    localparam [1:0] S_GAP   = 2'd3;    // 播完 / 出错后的空档

    reg [1:0]  state;
    reg [31:0] gap_cnt;
    reg        start_pulse;

    wire       file_busy;
    wire       file_done;
    wire       file_valid;
    wire [7:0] file_byte;
    wire [31:0] file_offset;
    wire [7:0] streamer_err;

    // ------------------------------------------------------------------
    // 流控：FIFO 放不下一个整扇区时，先不发下一个扇区请求
    // ------------------------------------------------------------------
    wire file_pause = (fifo_room < FIFO_ROOM_MIN);

    // ------------------------------------------------------------------
    // 内部文件流读器（复用第 7 讲已验证实现）
    // ------------------------------------------------------------------
    fat32_file_streamer #(
        .SECTOR_TIMEOUT_CYCLES (20_000_000)
    ) u_streamer (
        .clk                       (clk),
        .rst_n                     (rst_n),
        .abort                     (abort || !play_en),
        .file_start                (start_pulse),
        .file_pause                (file_pause),
        .first_cluster             (first_cluster),
        .file_size                 (file_size),
        .sectors_per_cluster       (sectors_per_cluster),
        .sectors_per_cluster_shift (sectors_per_cluster_shift),
        .fat_start_lba             (fat_start_lba),
        .data_start_lba            (data_start_lba),
        .max_cluster               (max_cluster),
        .file_busy                 (file_busy),
        .file_done                 (file_done),
        .file_valid                (file_valid),
        .file_byte                 (file_byte),
        .file_byte_offset          (file_offset),
        .fs_error                  (streamer_err),
        .sector_req                (sector_req),
        .sector_lba                (sector_lba),
        .sector_busy               (sector_busy),
        .sector_valid              (sector_valid),
        .sector_byte               (sector_byte),
        .sector_byte_index         (sector_byte_index),
        .sector_done               (sector_done),
        .sector_error              (sector_error)
    );

    assign busy = file_busy;

    // ------------------------------------------------------------------
    // 字节 → 16bit 小端样本
    //   裸 PCM 是 little-endian：先到的字节是低 8 位。所以第一字节先存起来，
    //   第二字节到来时拼成 {高字节, 低字节} 一次写入。
    //   ★ wr_en / wr_data 同一拍寄存，保证 FIFO 两端看到的对齐（差一拍就全错位）。
    // ------------------------------------------------------------------
    reg [7:0] byte_lo;
    reg       byte_phase;      // 0 = 正在等低字节，1 = 已收到低字节等高位

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_wr_en   <= 1'b0;
            fifo_wr_data <= 16'd0;
            byte_lo      <= 8'd0;
            byte_phase   <= 1'b0;
        end else begin
            fifo_wr_en <= 1'b0;

            if (!play_en || abort || (state == S_START)) begin
                // 每次重新开始都从"等低字节"起算，绝不能带着上一首的半样本
                byte_phase <= 1'b0;
            end else if (file_valid) begin
                if (!byte_phase) begin
                    byte_lo    <= file_byte;
                    byte_phase <= 1'b1;
                end else begin
                    byte_phase   <= 1'b0;
                    fifo_wr_data <= {file_byte, byte_lo};   // 高字节, 低字节（小端）
                    fifo_wr_en   <= 1'b1;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // 主状态机
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            start_pulse <= 1'b0;
            gap_cnt     <= 32'd0;
            done_pulse  <= 1'b0;
            fs_error    <= 8'd0;
        end else begin
            start_pulse <= 1'b0;
            done_pulse  <= 1'b0;

            if (abort) begin
                state    <= S_IDLE;
                gap_cnt  <= 32'd0;
                fs_error <= 8'd0;
            end else if (!play_en) begin
                // 上层不要播了：回 IDLE，等 play_en 重新拉高再从头来
                state    <= S_IDLE;
                gap_cnt  <= 32'd0;
                fs_error <= 8'd0;
            end else begin
                case (state)
                    S_IDLE: begin
                        gap_cnt     <= 32'd0;
                        start_pulse <= 1'b1;
                        state       <= S_START;
                    end

                    S_START: begin
                        // start_pulse 只打一拍，这里等 file_busy 起来
                        if (file_busy)
                            state <= S_RUN;
                    end

                    S_RUN: begin
                        if (file_done) begin
                            // 整首读完：循环模式下歇 50μs 再从头来；否则停在 GAP 等 play_en 掉
                            done_pulse <= 1'b1;
                            fs_error   <= 8'd0;
                            gap_cnt    <= 32'd0;
                            state      <= S_GAP;
                        end else if (streamer_err != 8'd0) begin
                            // 读失败（拔卡 / FAT 环 / 扇区超时）：记下来，歇 100ms 再试
                            fs_error <= streamer_err;
                            gap_cnt  <= 32'd0;
                            state    <= S_GAP;
                        end
                    end

                    S_GAP: begin
                        if (gap_cnt >= ((fs_error != 8'd0) ? RETRY_GAP_CYCLES
                                                           : LOOP_GAP_CYCLES)) begin
                            // 出错 → 无限重试（卡插回来就自动恢复）；
                            // 正常播完 → 只有 LOOP_EN 才重头来
                            if ((fs_error != 8'd0) || LOOP_EN)
                                state <= S_IDLE;
                            else
                                state <= S_GAP;   // 单次播放：停在这里，直到 play_en 掉
                        end else begin
                            gap_cnt <= gap_cnt + 32'd1;
                        end
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

    // 第11讲阶段C：把文件内偏移引出，供 OSD 进度换算
    assign file_offset_out = file_offset;

endmodule
