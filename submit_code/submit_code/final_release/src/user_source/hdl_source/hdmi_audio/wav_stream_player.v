`timescale 1ns/1ps
// ============================================================================
// wav_stream_player.v —— WAV（裸 PCM）流式播放器（第 11 讲 阶段 B 新增）
//
// 职责：把 SD 侧送来的 16bit 小端 PCM 字节流，按 48kHz 节拍变成
//       valid/ready 样本流，供 audio_source_mux 选用，最终进 HDMI 音频包。
//
// 本模块横跨两个时钟域，文件内用清晰分区隔开：
//   ▸ SD 侧（sd_clk，100MHz）：只做一件事 —— 把 FIFO 写端口透出去给
//     wav_sector_feeder，并输出 sd_fifo_room 让它知道"还能灌多少"。
//   ▸ 播放侧（pix_clk，25MHz）：48kHz 节拍生成、样本取出、音量缩放、
//     输出小 FIFO、以及"什么时候该算 WAV 在播"的状态机。
//
// ★ 为什么 48kHz 节拍必须用相位累加器，不能用固定分频
//   25MHz / 48kHz = 520.8333… 不是整数。取整成 520 会让实际采样率变成
//   48077Hz（偏快 0.16%），取 521 会变成 47985Hz（偏慢 0.03%）。单次听着没差，
//   但这是**系统性偏差**：一首 3 分钟的歌会累计漂移几百毫秒，和画面/字幕
//   对不上。相位累加器没有累计误差：
//       acc += 48000;  if (acc >= 25000000) { acc -= 25000000; tick; }
//   长期平均严格等于 48kHz。pcm_media_tone 用的是同一套写法，保持一致。
//   （它写成 acc >= CLOCK_HZ - SAMPLE_HZ 的形式，与本式等价，这里沿用其写法
//     以便两路音源在仿真里可以并排对比。）
//
// ★ 为什么每个节拍都必须吐出"一个样本"，哪怕没数据
//   HDMI 音频是把样本塞进固定节奏的 sample packet 里发出去的。如果 WAV 路在
//   缓冲空的时候不吐样本，下游 packer 就会长时间收不到 valid，音频时钟与
//   采样率的对应关系被打断（表现为爆音或长静音后突然恢复）。
//   正确做法是：缓冲空 → 吐 16'sd0（数字静音），保持 48kHz 节拍不断。
//   所以本模块的"underrun"是**静音填充**，不是停止。
//
// ★ 什么时候把 wav_active 拉起来 / 放下
//   拉起：sd_play_en=1（SD 侧已扫到音频文件并进入播放态）且 FIFO 里攒到
//         REFILL_WORDS 个样本（约 2.7ms），避免"开一下断一下"。
//   放下：① sd_play_en 掉了（文件播完 / 卡被拔 / 重扫）
//         ② 连续 STALL_CYCLES（默认 1s @25MHz）一个样本都没有 —— 说明 SD 侧
//            已经供不上（拔卡、读失败、仲裁死锁）。这时回落琶音，
//            用户至少还能听到声音，也等于给了个"音乐没了"的可听提示。
//   两个条件都保留：放下后只要数据恢复（level 回到 REFILL_WORDS 以上）会自动
//   重新拉起，不需要人工干预 —— 与全工程"拔插卡自恢复"的风格一致。
// ============================================================================
module wav_stream_player #(
    parameter integer CLOCK_HZ     = 25_000_000,
    parameter integer SAMPLE_HZ    = 48_000,
    parameter integer FIFO_AW      = 10,          // 音频 FIFO 深度 = 2**AW 字
    parameter integer STALL_CYCLES = 25_000_000,  // 断流判定 1s @25MHz
    parameter integer REFILL_WORDS = 128          // 重新开声所需的最低水位（字）
)(
    // ================= SD 侧（sd_card_clk 域）=================
    input  wire               sd_clk,
    input  wire               sd_rst_n,
    input  wire               sd_play_en,       // 电平：1 = 应该正在放 WAV
    input  wire               sd_fifo_wr_en,    // 来自 wav_sector_feeder
    input  wire [15:0]        sd_fifo_wr_data,
    // 统一用 13 位（足够描述到 8192 字），避免顶层线宽不匹配：
    //   本参数下实际量程 0~1024，高位恒 0
    output wire [12:0]        sd_fifo_room,     // 还能写多少字（0 ~ 2**AW）

    // ================= 播放侧（video_clk 域）=================
    input  wire               pix_clk,
    input  wire               pix_rst_n,
    input  wire [7:0]         volume,           // 与琶音同一套音量（128 ≈ 半幅）
    output wire               sample_valid,
    input  wire               sample_ready,
    output wire signed [15:0] sample_left,
    output wire signed [15:0] sample_right,
    output wire               active,           // 交给 audio_source_mux 的 wav_active

    // ================= 诊断 =================
    output wire               underrun,         // 正在用静音填充（数据没跟上）
    output wire [12:0]        level_dbg         // 当前可读样本数
);

    // ------------------------------------------------------------------
    // 跨时钟域 FIFO（写侧 = sd_clk，读侧 = pix_clk）
    // ------------------------------------------------------------------
    wire        fifo_rd_pop;
    wire [15:0] fifo_rd_data;
    wire        fifo_rd_valid;
    wire        fifo_rd_empty;
    wire [FIFO_AW:0] fifo_rd_level;
    wire [FIFO_AW:0] fifo_wr_room;

    wav_async_fifo16 #(.AW(FIFO_AW)) u_fifo (
        .wr_clk   (sd_clk),
        .wr_rst_n (sd_rst_n),
        .wr_en    (sd_fifo_wr_en),
        .wr_data  (sd_fifo_wr_data),
        .wr_room  (fifo_wr_room),
        .rd_clk   (pix_clk),
        .rd_rst_n (pix_rst_n),
        .rd_pop   (fifo_rd_pop),
        .rd_data  (fifo_rd_data),
        .rd_valid (fifo_rd_valid),
        .rd_empty (fifo_rd_empty),
        .rd_level (fifo_rd_level)
    );

    // ------------------------------------------------------------------
    // sd_play_en 同步到播放域（电平信号，2FF 足够）
    // ------------------------------------------------------------------
    reg [1:0] play_en_sync;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n)
            play_en_sync <= 2'b00;
        else
            play_en_sync <= {play_en_sync[0], sd_play_en};
    end
    wire play_en_pix = play_en_sync[1];

    // 播放状态（声明提前：fifo_rd_pop / underrun 等组合逻辑要先用到它）
    reg        playing;
    reg [24:0] stall_cnt;

    // ------------------------------------------------------------------
    // 48kHz 节拍（相位累加器，见文件头说明）
    // ------------------------------------------------------------------
    reg [24:0] rate_acc;
    wire sample_tick = (rate_acc >= CLOCK_HZ - SAMPLE_HZ);

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n)
            rate_acc <= 25'd0;
        else if (sample_tick)
            rate_acc <= rate_acc + SAMPLE_HZ - CLOCK_HZ;
        else
            rate_acc <= rate_acc + SAMPLE_HZ;
    end

    // 每个节拍向 BRAM FIFO 要一个字（空了就不弹，自然静音）。
    // ★ 必须用 playing 门控：否则"尚未开声"的那段时间也会把 FIFO 里的数据
    //   弹出来塞进 8 深输出队列然后丢掉 —— 仿真里表现为开头 100 多个样本
    //   凭空消失（听起来就是开头"咔"一下少了 2~3ms）。
    assign fifo_rd_pop = playing && sample_tick && !fifo_rd_empty;

    // 节拍打一拍：与 FIFO 的 1 拍读延迟对齐（rd_valid 正好在这一拍有效）
    reg tick_d1;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n)
            tick_d1 <= 1'b0;
        else
            tick_d1 <= sample_tick;
    end

    // ------------------------------------------------------------------
    // 音量缩放（与 pcm_media_tone 同一算法：×volume>>8 + 饱和截断）
    //   两路音源用同一套归一化，mux 切换时响度不跳；也让阶段 C 的音量
    //   调节对 WAV 和琶音是同一个旋钮。
    // ------------------------------------------------------------------
    wire signed [15:0] raw_sample = $signed(fifo_rd_data);
    wire signed [23:0] scaled     = raw_sample * $signed({1'b0, volume});
    wire signed [15:0] scaled_clamped =
        (scaled > 24'sd8388352)  ? 16'sd32767 :
        (scaled < -24'sd8388608) ? -16'sd32768 : scaled >>> 8;

    // ------------------------------------------------------------------
    // 输出小 FIFO（8 深，结构同 pcm_media_tone）
    //   吸收下游取样本的瞬时抖动；稳态下每拍进一个、下游取一个，深度不变。
    // ------------------------------------------------------------------
    reg signed [15:0] out_fifo [0:7];
    reg [2:0]  wr_ptr, rd_ptr;
    reg [3:0]  out_count;

    wire sample_pop  = sample_valid && sample_ready;
    // 满 8 且本拍没有出队 → 跳过这一次入队（宁可丢一个样本，也不让指针回绕串数据）。
    // 稳态下入队/出队严格 1:1（都是 48kHz），不会走到这个分支。
    // ★ sample_push 用 playing 门控：没在播的时候一个样本都不往外送，
    //   与 mux 的 use_wav = wav_active 严格对齐 —— 否则播放器会在"没开声"
    //   期间往输出队列塞静音，下游看到 valid 却选的是琶音，白白制造歧义。
    wire sample_push = tick_d1 && playing && ((out_count != 4'd8) || sample_pop);

    assign sample_valid = (out_count != 4'd0);
    // 单声道 PCM：左右接同一份
    assign sample_left  = out_fifo[rd_ptr];
    assign sample_right = out_fifo[rd_ptr];

    // ------------------------------------------------------------------
    // active 状态机（拉起 / 放下条件见文件头）
    // ------------------------------------------------------------------
    assign active   = playing;
    assign underrun = playing && fifo_rd_empty;

    // 顶层端口统一 13 位，这里显式补零，避免综合器报位宽不匹配
    assign sd_fifo_room = {{(12-FIFO_AW){1'b0}}, fifo_wr_room};
    assign level_dbg    = {{(12-FIFO_AW){1'b0}}, fifo_rd_level};

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            playing   <= 1'b0;
            stall_cnt <= 25'd0;
        end else if (!play_en_pix) begin
            // SD 侧说不该播了（文件结束 / 拔卡 / 重扫）→ 立刻回落琶音
            playing   <= 1'b0;
            stall_cnt <= 25'd0;
        end else if (fifo_rd_empty) begin
            // 没数据：累计断流时间，超时放弃
            if (stall_cnt >= STALL_CYCLES) begin
                playing   <= 1'b0;
                stall_cnt <= 25'd0;
            end else begin
                stall_cnt <= stall_cnt + 25'd1;
            end
        end else begin
            // 有数据：断流计时清零；攒够水位才开声（防"开一下断一下"）
            stall_cnt <= 25'd0;
            if (!playing && fifo_rd_level >= REFILL_WORDS)
                playing <= 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // 输出 FIFO 读写指针
    // ------------------------------------------------------------------
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            wr_ptr    <= 3'd0;
            rd_ptr    <= 3'd0;
            out_count <= 4'd0;
        end else begin
            if (sample_push) begin
                // 断流时补数字静音，保持 48kHz 节拍不中断（见文件头说明）
                out_fifo[wr_ptr] <= fifo_rd_valid ? scaled_clamped : 16'sd0;
                wr_ptr <= wr_ptr + 1'b1;
            end
            if (sample_pop)
                rd_ptr <= rd_ptr + 1'b1;

            case ({sample_push, sample_pop})
                2'b10:   out_count <= out_count + 1'b1;
                2'b01:   out_count <= out_count - 1'b1;
                default: out_count <= out_count;
            endcase
        end
    end

endmodule
