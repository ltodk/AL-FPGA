// =====================================================================
// audio_waveform_overlay: 音频波形 + 峰值条 像素级叠加（第 12 讲）
//
//   在画面中下部叠加一块「示波器面板」：
//     - 半透明暗底（亮度减半，Alpha≈128，与 subtitle_scroll_overlay 同一套移位近似）
//     - 中心轴线（深灰，表示 PCM 的 0 电平）
//     - 时域波形（亮绿，16bit 有符号样本以中轴为 0 双向展开）
//     - 右侧峰值条（橙色，本帧 64 个采集点区间的绝对峰值）
//
//   ★ 采样率核算（本讲最关键的一条，决定 DECIMATE 的取值）
//     一帧 = 1/60s = 16.67ms，48kHz 音频在此时间内只到达 800 个样本，
//     所以「每帧能采集的样本数」上限就是 800，与屏幕分辨率无关。
//     本模块取 SAMPLE_COUNT(256) × DECIMATE(2) = 512 个样本/帧，
//     余量 800/512 = 1.56 倍；窗口 = 512/48000 = 10.67ms。
//
//   ★ DECIMATE 的选取受 Nyquist 约束，不能图省存储就取大
//     显示采样率 = 48000 / DECIMATE。取 8 时只有 6000Hz，
//     对 3200Hz 素材（教材六段阶梯音的最高一段）已经越过 Nyquist(3000Hz)，
//     波形会混叠成慢速拍频 —— 屏幕上看着像低频，完全是假的。
//     取 2 得到 24000Hz，Nyquist 12000Hz，覆盖全部可听素材的高频段。
//     ★ 混叠是「骗人」的失真，比画得粗糙严重得多，宁可多花一点存储。
//
//   ★ 相邻点必须连起来画（本实现在 y[i] 与 y[i+1] 之间画竖线）
//     点抽样画「每点一段水平线」时，只要相邻点高度差大于线宽就会出现断口：
//     3200Hz 在 24000Hz 显示率下每周期只有 7.5 个点，相邻点最多差 20+ 像素，
//     屏幕上就是一片散落的短横线，完全看不出波形。
//     连成竖线后：低频时竖线只有 1~2 像素，仍是一条细曲线；
//     高频时竖线拉长，自然变成密集的实心带 —— 这恰好如实反映了「振荡很密」。
//     这比插值便宜（不需要乘法），也比 min/max 包络更接近曲线观感。
//
//   ★ 峰值统计与抽取解耦：每个 pcm_valid 都参与峰值比较，而不是只在
//     抽中的那 1/8 样本上比。参考实现把峰值比较写在 decimate 分支内，
//     对正弦会系统性低估约 1~2dB 且数值抖动，本实现修正了这一点。
//
//   时钟域：全部在 video_clk（像素时钟）域。采集与显示同一个时钟，
//     不存在跨时钟域问题 —— 因为 wav_stream_player 本身就跑在 video_clk。
//
//   ★ 显示侧位宽必须收窄（上板后回头优化，省下约 1/3 个模块的 LUT）
//     坐标量程只有 0~639 / 0~479，第一版把 ya/yb/ylo/yhi/peak_height/
//     sample_index 全写成 integer（32 位），结果 `line_y >= ylo` 被综合成
//     32 位比较器、`CENTER_Y - x` 成了 32 位减法、`±1` 也是 32 位 ——
//     综合出来 1426 LUT，slices 直接顶到 75%。改法有三条：
//       ① 样本值转「行偏移」时走偏移二进制：补码加 128 恰好等于翻转符号位，
//          于是 `sel ^ 8'h80` 就把有符号样本变成 0~255 的无符号量，
//          右移 2 位即 /4，全程不再出现有符号加减和符号扩展；
//       ② 所有 y 相关变量压到 10 位、offsets 压到 6 位；
//       ③ 峰值条顶行改 10 位减法，比较只做一次。
//     ★ 唯一的代价：`>>2` 是向下取整，原先 `/4` 是向零取整，对 -1~-4
//       这类小负数会差 1 像素。64 像素高的面板里看不见，换来的是
//       一个 8 位有符号除法器都不用。
// =====================================================================
`timescale 1ns / 1ps

module audio_waveform_overlay #(
    parameter integer WAVE_X        = 64,     // 面板左上角 X
    parameter integer WAVE_Y        = 320,    // 面板左上角 Y
    parameter integer WAVE_WIDTH    = 512,    // 波形区宽度（像素）
    parameter integer WAVE_HEIGHT   = 64,     // 波形区高度（像素，偶数）
    parameter integer DECIMATE      = 2,      // 抽取率：每 2 个样本取 1 个（显示率 24kHz）
    parameter integer SAMPLE_COUNT  = 256     // 每帧采集的显示点数（= WAVE_WIDTH/每点像素数）
) (
    input  wire               clk,            // pixel_clk (video_clk)
    input  wire               rst,            // 高有效复位
    input  wire               de,             // 视频数据有效
    input  wire               vs,             // 场同步
    input  wire [9:0]         video_x,        // active 对齐水平坐标 0~639
    input  wire [23:0]        video_in,       // 输入 RGB888

    // 音频采集（与 clk 同域）
    //   pcm_valid 必须是「一个样本被消费」的单拍脉冲，
    //   即 wav_stream_player 的 sample_valid && sample_ready，
    //   ★ 不能直接用 sample_valid —— 它是 out_count != 0 的电平，
    //     队列积压时会连续多拍为高，会把同一个样本重复计入。
    input  wire               enable,         // 有音频文件才显示面板
    input  wire               pcm_valid,
    input  wire signed [15:0] pcm_sample,     // = sample_left（单声道）

    output wire [23:0]        video_out,
    output wire               overlay_active  // 调试：任意面板像素
);

    // ------------------------------------------------------------------
    // 派生常量
    // ------------------------------------------------------------------
    localparam integer SAMPLE_SPACING = WAVE_WIDTH / SAMPLE_COUNT;   // 每点占用像素
    // ★ 中轴比几何中心高 1 像素：8bit 有符号范围是 -128~+127，/4 后是 -32~+31，
    //   正负不对称。若中轴取 WAVE_Y+HEIGHT/2，满幅负样本会算到 WAVE_Y+HEIGHT
    //   那一行，正好落在区域判断之外、被裁掉。上移 1 像素后 -128 精确落在最后一行。
    localparam integer CENTER_Y       = WAVE_Y + WAVE_HEIGHT / 2 - 1;
    localparam integer PEAK_X0        = WAVE_X + WAVE_WIDTH + 8;     // 峰值条左边界
    localparam integer PEAK_X1        = PEAK_X0 + 16;                // 峰值条右边界（宽 16）

    // 坐标类常量提前声明成 10 位：量程本就 0~639 / 0~479，位宽对齐后
    // 比较两端不用隐式扩展，读代码也能一眼看出取值范围。
    // ★ 注：综合器本来就会把 integer 自动推导到必要位宽（实测见第 18.10 节），
    //   这里写成 10 位主要是可读性与"量程承诺"，不是省 LUT 的手段。
    localparam [9:0] WX  = WAVE_X;
    localparam [9:0] WX1 = WAVE_X + WAVE_WIDTH;
    localparam [9:0] WY  = WAVE_Y;
    localparam [9:0] WY1 = WAVE_Y + WAVE_HEIGHT;
    localparam [9:0] CY  = CENTER_Y;
    localparam [9:0] PX0 = PEAK_X0;
    localparam [9:0] PX1 = PEAK_X1;

    // ---- 显示侧窄位宽常量 ----
    // 数组索引位宽：SAMPLE_COUNT=256 时 8 位正好。显式截断成这一宽度再送数组，
    // 既避免综合器对 10 位索引报高位截断告警，也让位宽变化时改动点集中在这里。
    localparam integer IDX_W  = (SAMPLE_COUNT <= 256) ? 8 : 9;
    localparam [9:0]   LAST_I = SAMPLE_COUNT - 1;   // 最后一点的索引
    localparam [9:0]   SPAC10  = SAMPLE_SPACING;    // 每点占几像素（除法常量）
    localparam [9:0]   YTOP   = WAVE_HEIGHT - 1;    // 行偏移上限（64 高 → 63）

    // ------------------------------------------------------------------
    // 行计数（与 osd_overlay 同一套方法：vs 下降沿清零，de 下降沿 +1）
    // ------------------------------------------------------------------
    reg [9:0]  line_y;
    reg        de_d;
    reg        vs_d;
    wire       frame_start = vs_d && !vs;      // 场同步下降沿 = 新一帧开始

    // ------------------------------------------------------------------
    // 双缓冲（Ping-Pong）：写侧写 write_bank，读侧读 display_bank
    //   每帧只交换一次，保证读侧整帧看到的都是同一份完整数据 —— 不撕裂。
    //   存储量 SAMPLE_COUNT×8×2 = 1024bit，组合读，综合器用 FF/LUTRAM 都很小。
    // ------------------------------------------------------------------
    reg signed [7:0] sample_bank0 [0:SAMPLE_COUNT-1];
    reg signed [7:0] sample_bank1 [0:SAMPLE_COUNT-1];

    reg              write_bank;      // 当前采集写入的 bank（初值 0）
    reg              display_bank;    // 当前显示读取的 bank（初值 1，与写侧相反）
    reg              display_valid;   // 至少采满过一帧才允许画波形
    reg [9:0]        capture_count;   // 本帧已采集点数 0 ~ SAMPLE_COUNT（≤512，留足位宽）
    reg [7:0]        decimate_count;  // 抽取计数
    reg [7:0]        peak_bank0;      // bank0 的绝对峰值
    reg [7:0]        peak_bank1;      // bank1 的绝对峰值
    reg [7:0]        display_peak;    // 正在显示的峰值（交换时锁存）

    // 16bit 有符号 → 8bit 绝对值
    //   负数：高 8 位取反加一（补码转绝对值）；正数：直接取高 8 位
    //   ★ 不会溢出：只有高 8 位为 0x00 时 ~x+1 才回绕，而高 8 位为 0 说明
    //     符号位(bit15)=0，不会进入负数分支。
    wire [7:0] sample_abs = pcm_sample[15] ?
                            (~pcm_sample[15:8] + 8'd1) : pcm_sample[15:8];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            line_y         <= 10'd0;
            de_d           <= 1'b0;
            vs_d           <= 1'b0;
            write_bank     <= 1'b0;
            display_bank   <= 1'b1;
            display_valid  <= 1'b0;
            capture_count  <= 7'd0;
            decimate_count <= 8'd0;
            peak_bank0     <= 8'd0;
            peak_bank1     <= 8'd0;
            display_peak   <= 8'd0;
        end else begin
            // ---- 行计数 ----
            de_d <= de;
            vs_d <= vs;
            if (!vs)
                line_y <= 10'd0;              // 场消隐期间，行号归零
            else if (de_d && !de)
                line_y <= line_y + 10'd1;     // 一行结束，行号 +1

            // ---- 采集侧 ----
            if (frame_start) begin
                // ★ 只有采满 SAMPLE_COUNT 才交换。没采满（断流 / 刚开始播）
                //   就保持上一帧画面，避免显示半帧新数据造成跳变。
                if (capture_count >= SAMPLE_COUNT) begin
                    display_bank  <= write_bank;
                    display_valid <= 1'b1;
                    display_peak  <= write_bank ? peak_bank1 : peak_bank0;
                    write_bank    <= ~write_bank;
                end
                capture_count  <= 7'd0;
                decimate_count <= 8'd0;
                // 清「下一帧要写的那个 bank」的峰值；正在写的 bank 不清
                if (write_bank) peak_bank0 <= 8'd0;
                else            peak_bank1 <= 8'd0;
            end else if (pcm_valid) begin
                // ★ 峰值统计不受抽取影响：每个有效样本都比一次，
                //   否则真峰值落在被跳过的 7/8 样本里时会被系统性低估。
                if (write_bank) begin
                    if (sample_abs > peak_bank1) peak_bank1 <= sample_abs;
                end else begin
                    if (sample_abs > peak_bank0) peak_bank0 <= sample_abs;
                end

                if (capture_count < SAMPLE_COUNT) begin
                    if (decimate_count == 8'd0) begin
                        if (write_bank)
                            sample_bank1[capture_count] <= pcm_sample[15:8];
                        else
                            sample_bank0[capture_count] <= pcm_sample[15:8];
                        capture_count  <= capture_count + 7'd1;
                        decimate_count <= DECIMATE - 1;
                    end else begin
                        decimate_count <= decimate_count - 8'd1;
                    end
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // 显示侧（纯组合）：按当前像素坐标算颜色，与输入像素同拍输出
    //   ★ 必须同拍（0 延迟）。下游字幕层 / HDMI 发射器都用顶层共享的
    //     de/vs/video_x，如果这里让 rgb 晚一拍就和它们错位了。
    //     组合读数组天然 0 拍 —— 代价是综合器只能用 FF / 分布式 RAM，
    //     实测走分布式 RAM（`SYN-1027`），没有退化成 FF 阵列。
    //   ★★ 本模块 LUT 的大头在存储，不在逻辑（这里踩过一次坑，记清楚）：
    //     源码只有 sample_bank0/1 共 4096 bit，但显示侧要**同时**读 idx 和 idx+1
    //     （画相邻点连线的竖线），综合器只能把每个 bank 复制一份 → 实际 8192 bit，
    //     被推断成**四个** 256×8 分布式 RAM。所以想继续降 LUT 要减存储或减读口，
    //     压中间变量位宽几乎没用（综合器本来就会自动推导 integer 位宽）。
    //   ★ 除 sel_a/sel_b（必须 8 位有符号，就是样本本身）外，
    //     其余中间量一律窄位宽，见下方类型声明。
    // ------------------------------------------------------------------
    reg  [9:0]       sample_index;     // 当前点索引 0~SAMPLE_COUNT-1
    reg  [9:0]       sample_index_n;   // 下一点（用来把相邻点连成竖线）
    reg  [9:0]       ya, yb;           // 两个相邻点的绝对行号（320~383）
    reg  [9:0]       ylo, yhi;         // 竖线上下沿（各外扩 1 像素）
    reg  [5:0]       peak_height;      // 峰值条高度 0~63（= display_peak×1.25，封顶 63）
    reg  [9:0]       peak_top;         // 峰值条顶行（384-peak_height）
    wire [7:0]       peak_gain = display_peak - {2'b00, display_peak[7:2]};  // ×0.75
    reg  signed [7:0] sel_a;
    reg  signed [7:0] sel_b;
    reg              wave_pixel;
    reg              axis_pixel;
    reg              peak_pixel;
    reg              area_pixel;

    always @* begin
        sample_index   = 10'd0;
        sample_index_n = 10'd0;
        sel_a          = 8'sd0;
        sel_b          = 8'sd0;
        ya             = CY;
        yb             = CY;
        ylo            = CY;
        yhi            = CY;
        // 2026-11-01 修订（w28b）：峰值条"永远不到顶"的真正原因不是缩放系数，
        // 而是音量（top 里 `assign volume = 8'd200;`）先把样本压到 0.78 倍 ⇒
        // 送进来的样本高字节满量程只到 ~87，而不是 128。
        // 处理：×0.75 显示增益（只影响画面，不改声音），并封顶到面板顶部。
        //   MUSIC.PCM 实测：典型段落 51 -> 38 行（约 59%），重音 87 -> 63 行（到顶）。
        peak_height    = (peak_gain > 8'd63) ? 6'd63 : peak_gain[5:0];
        peak_top       = WY1 - {4'd0, peak_height};  // 10 位减法，本帧内不变
        wave_pixel     = 1'b0;
        axis_pixel     = 1'b0;
        peak_pixel     = 1'b0;
        area_pixel     = 1'b0;

        if (enable && de) begin
            // ---------- 波形区 ----------
            if (video_x >= WX && video_x < WX1 &&
                line_y  >= WY && line_y  < WY1) begin
                area_pixel = 1'b1;
                axis_pixel = (line_y == CY);

                sample_index   = (video_x - WX) / SPAC10;
                // 最后一点没有「下一点」，退回自己，避免越界读
                sample_index_n = (sample_index >= LAST_I) ?
                                 LAST_I : (sample_index + 10'd1);

                if (display_valid && sample_index <= LAST_I) begin
                    sel_a = display_bank ? sample_bank1[sample_index[IDX_W-1:0]]
                                         : sample_bank0[sample_index[IDX_W-1:0]];
                    sel_b = display_bank ? sample_bank1[sample_index_n[IDX_W-1:0]]
                                         : sample_bank0[sample_index_n[IDX_W-1:0]];
                    // ★ 有符号样本 → 行偏移，全程无符号：
                    //   `sel ^ 8'h80` 就是补码加 128（正好等于翻转符号位），
                    //   sel = -128 → 0，sel = 0 → 128，sel = +127 → 255。
                    //   再右移 2 位（= floor(u/4)，0~63）得到向上的高度，
                    //   用 YTOP 减掉就变成向下的行偏移：正样本靠上、负样本靠下。
                    //   结果范围 [0,63]，加上 WY 得到绝对行号 [320,383]。
                    ya = WY + (YTOP - {2'd0, (sel_a ^ 8'h80) >> 2});
                    yb = WY + (YTOP - {2'd0, (sel_b ^ 8'h80) >> 2});
                    // ★ 在 y[i] 与 y[i+1] 之间画竖线（再各向外扩 1 像素）：
                    //   这才是波形「连续」的关键。只画每点所在的 2 像素水平段时，
                    //   相邻点高差超过线宽就断开，高频段会碎成散点。
                    ylo = ((ya < yb) ? ya : yb) - 10'd1;
                    yhi = ((ya > yb) ? ya : yb) + 10'd1;
                    wave_pixel = (line_y >= ylo) && (line_y <= yhi);
                end
            end
            // ---------- 峰值条（波形区右侧，宽 16）----------
            else if (video_x >= PX0 && video_x < PX1 &&
                     line_y  >= WY  && line_y  < WY1) begin
                area_pixel = 1'b1;
                // 从面板底部往上长，满幅 = WAVE_HEIGHT
                if (line_y >= peak_top)
                    peak_pixel = 1'b1;
            end
        end
    end

    // 半透明暗底：每通道右移 1 位（亮度减半，Alpha≈128），与字幕带同一做法
    wire [23:0] dim_rgb = {1'b0, video_in[23:17],
                           1'b0, video_in[15:9],
                           1'b0, video_in[7:1]};

    assign overlay_active = area_pixel | wave_pixel | axis_pixel | peak_pixel;
    assign video_out      = wave_pixel ? 24'h20FF90 :   // 波形：亮绿
                            peak_pixel ? 24'hFFB020 :   // 峰值：橙
                            axis_pixel ? 24'h406060 :   // 中轴：深灰
                            area_pixel ? dim_rgb  :     // 面板底：半透明黑
                                         video_in;      // 面板外：原样透传

endmodule
