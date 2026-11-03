`timescale 1ns/1ps
// ============================================================================
// audio_source_mux.v —— 音频源二选一（第 11 讲 新增）
//
// 职责：在「WAV 流式播放」与「图片联动琶音」两个音源之间切换，
//       输出统一的 valid/ready/left/right 握手接口给下游 PCM 适配层。
//
// 为什么是纯组合：
//   两个音源本身就工作在同一个 pixel_clk 域（25MHz），数据路径上再加一级
//   寄存器只会引入延迟、打乱 valid/ready 时序，没有任何收益。组合逻辑在
//   25MHz（周期 40ns）下毫无压力。
//
// ★ ready 路由是本模块唯一的关键设计：
//   ready 只回给「被选中」的那一路，未选中的源恒收 ready=0。
//   - 选中 WAV 时 arp_ready=0 → 琶音模块看到下游不收，其内部 valid 保持、
//     数据不丢（pcm_media_tone 自带 8 深 FIFO）；
//   - 选中琶音时 wav_ready=0 → wav_stream_player 的读侧看到 sample_ready=0，
//     不消费读指针，其 BRAM 缓冲保持（不会溢出也不会丢样本）。
//   如果把 ready 同时给两路（常见错误写法），未选中的源会边产生边丢弃，
//   切回来时听到的是"跳了一段"的音频。
//
// 优先级：WAV 正在播放（wav_active=1）且未被强制琶音 → 用 WAV；
//         否则回落琶音。这样 WAV 没准备好/播完时自动回到琶音，不会哑掉。
//
// 切换瞬间的爆音：切换只发生在 sample 边界（valid/ready 握手间隙），
//   且两路音源的幅度都已经被各自的 volume 归一化，不会突然从静音跳到满幅。
//   若后续需要更平滑，可在下游串 sync_fade_gain（音画同步淡入淡出）。
// ============================================================================
module audio_source_mux (
    // === 源 A：WAV 流（来自 wav_stream_player，阶段 B 接入）===
    input  wire               wav_valid,
    output wire               wav_ready,
    input  wire signed [15:0] wav_left,
    input  wire signed [15:0] wav_right,
    input  wire               wav_active,      // 电平：WAV 正在播放

    // === 源 B：琶音合成（现有的 pcm_media_tone，按图片切调）===
    input  wire               arp_valid,
    output wire               arp_ready,
    input  wire signed [15:0] arp_left,
    input  wire signed [15:0] arp_right,

    // === 强制琶音（调试/降级用）===
    input  wire               force_arpeggio,

    // === 输出到 m06_pcm_contract_adapter ===
    output wire               out_valid,
    input  wire               out_ready,
    output wire signed [15:0] out_left,
    output wire signed [15:0] out_right
);

    // WAV 优先：正在播放 且 未被强制切走
    wire use_wav = wav_active && !force_arpeggio;

    assign out_valid = use_wav ? wav_valid : arp_valid;
    assign out_left  = use_wav ? wav_left  : arp_left;
    assign out_right = use_wav ? wav_right : arp_right;

    // ready 只路由给被选中源（见文件头说明）
    assign wav_ready = use_wav ? out_ready : 1'b0;
    assign arp_ready = use_wav ? 1'b0      : out_ready;

endmodule
