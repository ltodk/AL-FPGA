`timescale 1ns/1ps

// Player-owned boundary between the registered video bundle and the verified
// Route B HDMI packet/symbol chain. The EG4 DDR PHY remains in the board Top so
// this bridge can be exercised in ordinary RTL simulation without vendor pads.
// media_id drives the per-image tone selection and volume scales the output,
// turning the tone source into the audio half of the AV-linked feature.
module media_hdmi_audio_bridge #(
    // 1 = 强制走琶音、屏蔽 WAV（调试用：可单独确认琶音链路是否完好）
    parameter integer FORCE_ARPEGGIO = 1'b0
) (
    input  wire        pixel_clk,
    input  wire        pixel_rst_n,
    input  wire        video_hsync,
    input  wire        video_vsync,
    input  wire        video_de,
    input  wire [9:0]  video_x,
    input  wire [23:0] video_rgb,
    input  wire [1:0]  media_id,
    input  wire [7:0]  volume,
    // === WAV 流输入（第 11 讲：阶段 A 悬空为 0 → 自动走琶音；阶段 B 由
    //     wav_stream_player 驱动。wav_active=0 时本桥行为与加端口前完全一致）===
    input  wire               wav_valid,
    output wire               wav_ready,
    input  wire signed [15:0] wav_left,
    input  wire signed [15:0] wav_right,
    input  wire               wav_active,
    // === 第16讲：插播警报音（FPGA 内合成，三级音源里优先级最高）===
    // alarm_active=1 时把 WAV 与琶音都压住，只出警报；不需要卡上第二个文件。
    input  wire               alarm_valid,      // 保留：合成器现为"节拍驱动"，此口不再参与握手
    output wire               alarm_step,       // 1 = 这一拍真的送出一个样本（警报合成器的节拍）
    input  wire signed [15:0] alarm_left,
    input  wire signed [15:0] alarm_right,
    input  wire               alarm_active,
    output wire [9:0]  channel0_symbol,
    output wire [9:0]  channel1_symbol,
    output wire [9:0]  channel2_symbol,
    output wire        sequence_error,
    output wire        tone_overflow,
    output wire        pcm_contract_error,
    output wire        blip_active,
    output wire        pcm_tap_valid,
    output wire signed [15:0] pcm_tap_left
);
    // 琶音源（pcm_media_tone）内部信号
    wire               arp_valid;
    wire               arp_ready;
    wire signed [15:0] arp_left;
    wire signed [15:0] arp_right;
    // 音源选择后的统一接口（送 m06 PCM 契约适配层）
    wire source_valid;
    wire source_ready;
    wire signed [15:0] source_left;
    wire signed [15:0] source_right;
    wire sample_valid;
    wire sample_ready;
    wire signed [15:0] sample_left;
    wire signed [15:0] sample_right;

    reg video_hsync_q;
    reg video_vsync_q;
    reg video_de_q;
    reg [9:0] video_x_q;
    reg [23:0] video_rgb_q;
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            video_hsync_q <= 1'b1;
            video_vsync_q <= 1'b1;
            video_de_q <= 1'b0;
            video_x_q <= 10'd0;
            video_rgb_q <= 24'h000000;
        end else begin
            video_hsync_q <= video_hsync;
            video_vsync_q <= video_vsync;
            video_de_q <= video_de;
            video_x_q <= video_x;
            video_rgb_q <= video_de ? video_rgb : 24'h000000;
        end
    end

    pcm_media_tone u_tone (
        .clk(pixel_clk), .rst_n(pixel_rst_n),
        .media_id(media_id), .volume(volume),
        .sample_valid(arp_valid), .sample_ready(arp_ready),
        .sample_left(arp_left), .sample_right(arp_right),
        .overflow(tone_overflow), .blip_active(blip_active)
    );

    // 三级音源：警报 > WAV > 琶音（第16讲加入警报）
    //   ★ 2026-10-31 定稿语义（v3）：**替换 payload、握手不动**
    //     ① wav_active_eff 保持 = wav_active，mux 的 out_ready 保持 = source_ready
    //        ⇒ WAV / 琶音照常被消费（数据岛一秒都不断流，sink 不会静音），
    //          但警报期间它们的样本不被采用（听感上音乐静音）。
    //     ② 警报合成器由 alarm_step（= source_valid && source_ready）驱动前进，
    //        与 mux 完全同节拍，因此不会重复送/跳过样本。
    //   （历史：v1 把 mux 整个停掉 ⇒ 数据岛断流 ⇒ 只有波形没声音；v2 做加法叠加
    //     ⇒ 音乐还在响。两版都已废弃，详见 §23.3。）
    wire mux_valid;
    wire signed [15:0] mux_left;
    wire signed [15:0] mux_right;
    // 2026-10-31 修订：警报的握手只跟"真正被接受的样本"走。
    //   source_valid = mux_valid，所以只有 mux_valid && source_ready 这一拍样本才算数；
    //   警报合成器必须用同一个条件前进，否则 mux 空档期它会白走几拍（听起来是跳音）。
    //   WAV / 琶音本身照常被消费，只是 payload 不被采用（音乐静音但数据岛不断流）。
    wire wav_active_eff = wav_active;
    wire mux_out_ready  = source_ready;

    audio_source_mux u_audio_mux (
        .wav_valid      (wav_valid),
        .wav_ready      (wav_ready),
        .wav_left       (wav_left),
        .wav_right      (wav_right),
        .wav_active     (wav_active_eff),
        .arp_valid      (arp_valid),
        .arp_ready      (arp_ready),
        .arp_left       (arp_left),
        .arp_right      (arp_right),
        .force_arpeggio (FORCE_ARPEGGIO[0]),
        .out_valid      (mux_valid),
        .out_ready      (mux_out_ready),
        .out_left       (mux_left),
        .out_right      (mux_right)
    );

    // 警报合成器的节拍：只有"这一拍真的有样本被送出去"才算一步。
    //   ★ 2026-10-31 三修：原来合成器用自己独立的 48kHz 相位累加器，
    //     而 sink 实际取样的节拍由 mux_valid && source_ready 决定（WAV 的 FIFO
    //     出水节奏 / 换曲空档都会变），两个节拍不完全一致 ⇒ 同一个警报样本会被
    //     重复送出或跳过 ⇒ 方波边沿抖动，听感就是"密集打火声"。
    //     现在把 sink 的实际握手当节拍，送一个样本才走一步，谁快谁慢都不会抖。
    assign alarm_step = source_valid && source_ready;

    // ---- 最终音源合成：主节拍永远来自 mux，警报只"替换 payload" ----
    //   ★ 2026-10-31 二次修订（用户实测："放警报时普通音乐还在播放"）：
    //     第一版"叠加"解决了无声问题，但音乐跟警报一起响，用户要的是"只剩警报"。
    //     注意不能再回到"停 mux"那条老路（v1 就是那样导致数据岛断流、sink 静音）。
    //     折中做法：**握手/节拍完全不动**（mux_valid、mux_out_ready、alarm_ready 都照旧），
    //     只把 payload 换掉 —— 警报期间 mux 的样本照常被消费、只是不被采用。
    //     这样数据岛一秒都没断，音乐听不到，警报一定出声。
    assign source_valid = mux_valid;
    assign source_left  = alarm_active ? alarm_left  : mux_left;
    assign source_right = alarm_active ? alarm_right : mux_right;

    m06_pcm_contract_adapter u_m06_pcm_contract (
        .clk(pixel_clk), .rst_n(pixel_rst_n),
        .source_valid(source_valid), .source_ready(source_ready),
        .source_left(source_left), .source_right(source_right),
        .pcm_valid(sample_valid), .pcm_ready(sample_ready),
        .pcm_left(sample_left), .pcm_right(sample_right),
        .contract_error(pcm_contract_error)
    );

    hdmi_audio_symbol_core u_symbol_core (
        .pixel_clk(pixel_clk), .rst_n(pixel_rst_n),
        .video_hsync(video_hsync_q), .video_vsync(video_vsync_q),
        .video_de(video_de_q), .video_x(video_x_q), .video_rgb(video_rgb_q),
        .sample_valid(sample_valid), .sample_ready(sample_ready),
        .sample_left(sample_left), .sample_right(sample_right),
        .channel0_symbol(channel0_symbol),
        .channel1_symbol(channel1_symbol),
        .channel2_symbol(channel2_symbol),
        .sequence_error(sequence_error)
    );

    assign pcm_tap_valid = sample_valid && sample_ready;
    assign pcm_tap_left = sample_left;
endmodule
