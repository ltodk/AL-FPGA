
module top(
    input                       clk,
    input                       rst_n,
    output [3:0]                key_row,        // 矩阵键盘 行扫描（低有效）
    input  [3:0]                key_col,        // 矩阵键盘 列读入（低有效）

    output [3:0]                seg_sel,
    output [7:0]                seg_data,

    // UART 日志输出（CH340N TX，第5讲 Step2 串口可观测性）
    output                      uart_tx,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;   // 640*480
// ★ 2026-10-28 噪点排查：四帧整体平移量（字）。
//   ✅ 实验结论（已完成）：平移 +0x10000 后，分层边界从 70.6%→49%、41.3%→20%，
//      与"SDRAM 内部 bank(BA) 边界 0x80000/0x100000"的预测完全一致。
//      ⇒ 噪点只出现在 BA1/BA2（地址 ≥ 0x80000），BA0 永远干净。根因锁定。
//   现改回 0（交付布局）。⚠ 必须与 media_frame_loader 的 BASE_OFFSET 同值。
parameter BUF_BASE_OFFSET = 24'd0;
parameter BUF0_ADDR     = BUF_BASE_OFFSET;
parameter BUF1_ADDR     = BUF_BASE_OFFSET + FRAME_PIXELS;
parameter BUF2_ADDR     = BUF_BASE_OFFSET + FRAME_PIXELS * 2;
parameter BUF3_ADDR     = BUF_BASE_OFFSET + FRAME_PIXELS * 3;
// ★ 第15讲：2×2 缩略图选择页基址（四帧之后的一块独立区域）
//   640×480 页 = 307200 字，正好占满 bank4 的位置（地址 1228800，0x12C000）。
//   必须与 media_frame_reader 的 TB_ADDR 参数同值。
parameter THUMB_ADDR    = BUF_BASE_OFFSET + FRAME_PIXELS * 4;
// 擦洗覆盖范围要盖住缩略图页（5 帧 = 1536000 字），否则页会随存放时间衰减
parameter SCRUB_WORDS_CFG = 24'd1536000;

// =============================================================================
// ★ 诊断开关（实机排查"加载期间随机横条"专用，定位完成后改回 0）
//
//   DIAG_FREEZE_WRITE = 1 : 加载期间完全不向 SDRAM 写（App_wr_en 恒 0），
//       但 SD 读取、BMP 解码、像素计数、SPI 全速翻转全部照常运行。
//       ★ 这个开关的作用是把"SDRAM 写总线"和"SPI 高电流活动"这两件事拆开：
//          - 横条消失 → 根因在 SDRAM 写总线一侧（命令仲裁/时序/数据）
//          - 横条仍在 → 与 SDRAM 写无关，是 SPI 活动引起的电源/EMI/地弹
//
//   DIAG_SPI_SLOW（已退役）：原为 SPI 25MHz -> 16.7MHz 的诊断开关。
//       判别实验完成后已提升为正式设计参数 SD_SPI_HIGH_DIV，见下方说明。
//
//   ★ 2026-10-23 冷启动健壮性结论（SD/SPI 侧）：
//      现象：断电冷启动后，后几张图出现散落随机彩点、布满全图、持久不消；
//            按复位键把 4 张图重写一遍即完全干净；热复位复现不出来。
//      因为复位只是重写数据就好，所以坏的是写进 SDRAM 的数据，不是显示通路。
//      冷态与热态的唯一系统性差别是"加载第 1 张时 HDMI/音频/OSD 还没起来，
//      后面几张时整机已全速运行" → 指向冷启动余量（电源/EMI 或写时序）。
//      已做判别实验：把 SPI 从 25MHz 降到 16.67MHz（负载降约 1/3），
//      冷启动后彩点消失 → 根因在 **SD/SPI 侧信号完整性或上电时序**，
//      不是 SDRAM 写侧时序窗口。（若当时噪点照旧，才需要回头给写侧加 rd_guard。）
//      ★ 为什么"归零"能算证据而不是统计巧合：降频只把写 burst 频率降了约 1/3，
//        若真是随机竞争型错误，应当看到彩点减少约 1/3 而非归零。
//        归零说明这是"要么明显错、要么明显不错"的电气边界问题，不是概率问题。
//      本参数已从诊断开关转为**正式配置**（见下方 SD_SPI_HIGH_DIV）。
//
//   DIAG_SINGLE_BUF = 1 : 关掉双缓冲，写入与显示都固定 BUF0。
//       目的：让 4 张图各自都能完整显示出来，逐张判断好坏。
//       双缓冲下永远只有 BUF0/BUF1 两块内容，切第3、4张时看到的仍是旧内容，
//       无法判断"这张图自己是不是好的"。单缓冲下每按一次键，加载完就显示这张图。
//       副作用：加载期间屏幕被边写边读，会出现上下分界线（撕裂），加载完即正常。
// =============================================================================
localparam DIAG_FREEZE_WRITE = 1'b0;
localparam DIAG_SINGLE_BUF   = 1'b0;
// ★ 2026-10-28 噪点分离实验（只影响读侧，写侧仍是四 bank 一一对应）：
//   1 = 读侧基址永远固定为 bank0（画面一直显示 IMG:1），完全不再换基址。
//   ✅ 实验结果（2026-10-28 21:14）：置 1 后连续读 bank0 **15 分钟零噪点**。
//      ⇒ 读 BA0 是稳的；故障只出现在 BA1/BA2（地址 ≥ 0x80000）。
//   ⇒ 已改回 0，改用 BUF_BASE_OFFSET 平移实验来钉死"BA 边界"。
localparam DIAG_READ_BANK0    = 1'b0;

// =============================================================================
// ★ SD 卡 SPI 高速段分频（正式设计参数，不再是诊断开关）
//   公式（sd_card_sec_read_write.v 第 5 行）：spi_clk = 100MHz / ((DIV+2)*2)
//     DIV = 0 → 25.00 MHz   （4 张图加载约 1.2s）
//     DIV = 1 → 16.67 MHz   （4 张图加载约 1.8s，当前采用）
//   取 1 的原因：25MHz 下断电冷启动时，SD 读回数据偶发错字，导致后几张图
//   出现散落随机彩点（详见上方"冷启动健壮性结论"）。降频后不再复现。
//   代价仅为上电多等约 0.6s，换来的是演示时不必担心第一次上电就带噪点。
//   ★ 若要提速，改回 0 前必须重新做 10 次冷启动浸泡测试确认不再复现。
// =============================================================================
//
// ===== 【噪点排查实验结论 2026-10-28：SD 链路已排除】=====
//   做过 DIV 1→5（SPI 16.67MHz → 7.14MHz）的实机对比：噪点**没有变少，反而更密**。
//   若是 SD 读位错，降频 2.3 倍错误率只会降不会升 ⇒ **问题不在 SD/SPI 链路**。
//   （文件头"冷启动健壮性结论"里"25MHz 出现散落随机彩点、降频后不复现"那条，
//     当时是有其他变量一起变的，不能当证据用；以本次单变量实验为准。）
//   ⇒ 保持 16.67MHz。真正的根因见下方 DIAG_FREEZE_READ_ON_LOAD。
// ★ 2026-10-31 23:40：退回 16.67MHz。25MHz（DIV=0）实测出 ERR:3E + 花屏 + 图片变形，
//   说明这张卡/这条 SPI 通路撑不住 25MHz，读回的扇区数据被破坏（错图就是数据错）。
//   这个参数只有一个使用点（见下方 sd_card_bmp 例化），要再试 25MHz 只改这一行。
localparam integer SD_SPI_HIGH_DIV = 1;

// =============================================================================
// ★ 录制对比素材用：写后静默期拍数（SDRAM tWR/tRP/tRCD 覆盖）
//     GUARD_CYCLES_CFG = 24 : 正常（修复后，横条消失）
//     GUARD_CYCLES_CFG = 0  : 复现"修复前"的现象（加载期间出现横条）
//   改这一个数字 → 重新综合 → 烧录，即可录到"坏"或"好"的画面。
//   录完对比素材务必改回 24。
// =============================================================================
localparam integer GUARD_CYCLES_CFG = 24;

// =============================================================================
// ★ 第10讲 图片转场（糊到灰幕 → 换 bank → 化回原图）的三个可调项
//
//   转场形态 = "fade through grey"（淡到灰），不是压暗：
//       淡出时画面向灰阶收敛（暗部抬升、亮部下降），平均亮度基本不动，
//       最糊那一帧是一整屏中性灰、没有内容 → 在这一帧换 bank，完全看不出跳变。
//   为什么不做"压暗"：alpha 是编码域的乘性系数，显示器是 2.2 伽马，
//       编码值乘 0.25 实际发光只剩 4.8% —— 实机验证 MIN_ALPHA 从 0 抬到 64
//       仍觉得刺眼，因为 4.8% 和 0% 肉眼无差别。详见 fade_alpha_apply.v 头注释。
//
// ① FADE_FRAMES_CFG —— 决定"多快"
//   计时基准：视频 640×480@59.5fps，一帧 ≈ 16.8ms。
//   全程帧数 = 2*FADE_FRAMES + 5（固定 5 帧开销来自：进入淡出那一帧仍全亮、
//   最糊档保持约 3 帧、收尾 1 帧。实测值，见 sim/reg10_*.log 的帧明细）。
//     FADE_FRAMES = 4  → 13 帧 ≈ 0.22s（很快，接近"闪一下"，适合频繁切图）
//     FADE_FRAMES = 8  → 21 帧 ≈ 0.35s（当前采用，干净利落，不拖）
//     FADE_FRAMES = 16 → 37 帧 ≈ 0.62s（原值，主观偏拖沓）
//
// ② MIN_ALPHA_CFG —— 决定"糊到多彻底"
//   它是最糊那一刻保留的**原图权重**（alpha=255 是全原图）：
//     0   → 完全糊成纯灰（`(256-0)/8 = 32`，整除，可用）
//     16  → 保留 6.25% 原图（当前采用，肉眼就是纯灰，但保留极淡的轮廓层次）
//     48  → 保留 18.75%（`(256-48)/8 = 26`，整除）能看出转场，但轮廓会露出来
//     64  → 保留 25%（`(256-64)/8 = 24`，整除）上一版压暗方案的值，灰幕下偏"半透"
//   ★ 换 bank 就发生在 alpha=MIN_ALPHA 那一帧，所以这个值决定"跳变露不露馅"：
//     越小越藏得住，但要留一点点原图层次才不像"凭空变灰"。
//   ★ 约束：(256 - MIN_ALPHA_CFG) 必须能被 FADE_FRAMES_CFG 整除，否则 STEP 取整，
//     最后一级跳不满 255，收尾会"顿一下"。当前 8 帧 + 16 → 240/8 = 30，正好。
//
// ③ FADE_GRAY_CFG —— 决定"灰到什么色阶"（见 fade_alpha_apply.v 端口注释）
//     128 → 中性灰（当前采用）。正好是第14讲把对比度调到 C=0 时的那一屏，
//           白字 OSD / 字幕在灰底上仍然清晰
//     176 → 发雾更柔，但 OSD 白字开始糊
//     224 → 接近"白闪"，最柔，但最糊那一帧 OSD 白字基本看不见
//
//   ★ 三项只影响视觉观感，不影响功能：alpha 仍只在帧边界变化，整帧恒定、不撕裂；
//     换 bank 仍严格发生在最糊档的帧边界（仿真断言守着）。
// =============================================================================
localparam integer FADE_FRAMES_CFG = 8;
localparam integer MIN_ALPHA_CFG   = 16;
localparam integer FADE_GRAY_CFG   = 128;

// =============================================================================
// ★ 第 11 讲 WAV 音频播放（阶段 B）两个开关
//
//   WAV_PLAY_EN = 1 : 卡根目录有 MUSIC.PCM（无头/16bit/小端/单声道/48kHz）就播它，
//                     播完循环；没有该文件、或播放中拔卡断流 1s → 自动回落琶音。
//              = 0 : 完全关掉 WAV，行为与第 10 讲一致（琶音，切图换调）。
//                     ★ 答辩要演示"切图换音调"时把它设 0 重综合，或直接把卡里的
//                       MUSIC.PCM 删掉（运行时切换，不用重新综合）。
//
//   WAV_LOOP_EN = 1 : 单曲循环（当前采用，当背景音乐）
//               = 0 : 播一遍就停，之后回落琶音
//
//   音频素材：D:/FPGA/test/音频脚本/ 下的 wav2pcm.py 生成，输出必须
//             无文件头 / 16bit signed / 小端 / 单声道 / 48kHz。
// =============================================================================
localparam integer WAV_PLAY_EN = 1;
localparam integer WAV_LOOP_EN = 1;

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;
wire [11:0] video_x;          // 水平像素计数（对齐到 hs/vs/de，供 HDMI 数据岛定位）
// 坐标转换：color_bar 的 h_cnt 以 FP 为 0（0~799），参考方案 video_x 以 active 为 0。
// active(h_cnt=160~799) -> ref 0~639；消隐(h_cnt=0~159) -> ref 640~799。
wire [9:0]  video_x_ref;
assign video_x_ref = (video_x >= 12'd160) ? (video_x[9:0] - 10'd160)
                                          : (video_x[9:0] + 10'd640);

wire [23:0] vout_data_raw;
wire [23:0] vout_data;
wire        display_valid;

wire [3:0]  state_code;
wire [3:0]  disp_num;
wire [6:0]  seg_data_0;
wire [7:0]  bmp_err_code;         // BMP 加载/扫描错误码（8位细分，Step3 接数码管/OSD）
wire [2:0]  img_total;            // 已找到图片总数（OSD/UART）
wire        sd_init_done;         // SD 初始化状态（OSD/UART）
wire        load_error;           // 加载错误脉冲（UART）
wire        ld3s_pulse;           // 3s兜底超时脉冲（诊断 UART）
wire        ld3s_src;             // 3s兜底时 source_done_seen 状态
wire        fat_dbg_pulse;        // FAT解析完成脉冲（诊断 UART）
wire        fat_dbg_ok;           // FAT解析合法性
wire [31:0] fat_dbg_val;          // FAT解析出的数据区起始扇区
wire        scan_diag_pulse;      // 扫描结束脉冲（诊断 UART）
wire        scan_diag_zero;       // 1=本轮0图收场
wire [31:0] scan_diag_val;        // 首图扇区/终止扇区
wire [1:0]  recover_state;        // 恢复状态机当前态（Step3可观测）

wire        video_read_req;
wire        video_read_req_ack;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
// 第8讲阶段A：sd_card_write_req_ack 改由 loader_load_start 单拍应答，
// frame_write_finish 不再使用（阶段B 已彻底移除 frame_read_write），故不声明。
wire        frame_write_toggle;      // 由 media_write_commit 在 sd_card_clk(100M) 域产生

wire [1:0]  write_buf_idx;
wire [1:0]  disp_buf_idx;         // 上游请求的 bank（sd_card_bmp，100M 域，零延迟）

// 第10讲：转场控制产生的信号（全部在 video_clk 25M 域内）
//   disp_buf_idx_q —— 真正送给 reader 的 bank（等到全黑那一刻才更新）
//   fade_alpha     —— 0~255，255=原图、0=全黑，整帧恒定
wire [1:0]  disp_buf_idx_q;
wire [7:0]  fade_alpha;
wire        fade_busy;

// =============================================================================
// 第15讲：2×2 缩略图选择页
//   ① sd_card_clk(100M) 域：KEY15 切"缩略图页"视图，KEY12/KEY14 移焦点，
//      KEY13 确认（跳到选中那张图并退出）。全部在 sd 域做，省掉跨域握手。
//   ② video_clk(25M) 域：两个慢变电平（view/focus）两级同步后，在帧边界
//      （vs 下降沿）锁存，读基址 mux 与焦点框叠加都只看锁存值 ⇒ 帧内绝不变化。
// =============================================================================
wire [ADDR_BITS-1:0] rd_base_0;
wire [ADDR_BITS-1:0] rd_base_1;
wire [ADDR_BITS-1:0] rd_base_2;
wire [ADDR_BITS-1:0] rd_base_3;
wire                    tb_wr_en;
wire [ADDR_BITS-1:0]    tb_wr_addr;
wire [MEM_DATA_BITS-1:0] tb_wr_data;
wire                    tb_busy;
wire                    tb_done;

// ★ 2026-10-31（B 方案）：bank 有效位 / 当前写 bank（来自 sd_card_clk 域，video_clk 域两级同步）
wire [3:0]              bmp_bank_valid;
wire [1:0]              bmp_load_bank;

reg         thumb_view_sd;      // 1 = 正在显示缩略图页
reg  [1:0]  focus_sd;           // 焦点象限 {qy,qx}
reg         jump_req_sd;        // KEY13 确认（单拍）
reg  [1:0]  jump_idx_sd;

reg         thumb_view_v;       // video_clk 域（帧边界锁存）
reg  [1:0]  thumb_focus_v;
reg         thumb_view_s1, thumb_view_s2;
reg  [1:0]  focus_s1, focus_s2;
reg         vs_adj_d2;
wire [23:0] vout_data_thumb;

wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

wire hs_0;
wire vs_0;
wire de_0;
wire [11:0] h_cnt_0;

// HDMI 音频发射相关（参考方案：像素时钟驱动，音调由 pcm_media_tone 生成）
wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

// 音频桥接输入（Step3：media_id 接 img_idx 做切图换音调，volume 设非零）
wire [1:0]  media_id;
wire [7:0]  volume;
// 2026-11-01 修订（w28）：128 -> 200。128/256 = 0.5（-6 dB）是把 WAV 与琶音放同一个
// 旋钮时的"保守值"，实际听感偏小；200/256 = 0.781（比原来响约 3.9 dB），
// 而卡上 MUSIC.PCM 峰值 0.867 FS ⇒ 0.867×0.781 = 0.677，仍留 3.4 dB 余量不会削顶。
// 想更响可继续加到 224（0.875，+4.8 dB）或 255（≈1.0）；琶音走同一个旋钮，会一起变大。
assign volume   = 8'd200;       // 音量 200/256 ≈ 0.78 幅度

// ===================== 第 11 讲阶段 B：WAV 音频链路 =====================
// sd 侧（sd_card_clk）与播放侧（video_clk）之间通过异步 FIFO 交接
wire [12:0]        wav_fifo_room;      // 播放侧 → SD 侧：还能灌多少字
wire               wav_fifo_wr_en;     // SD 侧 → 播放侧：写异步 FIFO
wire [15:0]        wav_fifo_wr_data;
wire               wav_play_en;        // SD 侧 → 播放侧：该播了（电平）
wire               wav_audio_found;    // 诊断：扫到 MUSIC.PCM
wire               wav_busy;           // 诊断：音频文件流正在读
wire [7:0]         wav_fs_error;       // 诊断：音频文件读错误码
wire               wav_sample_valid;   // 播放侧 → 音频桥
wire               wav_sample_ready;
wire signed [15:0] wav_sample_left;
wire signed [15:0] wav_sample_right;
wire               wav_active;         // 1 = 用 WAV 音源
wire               wav_underrun;       // 诊断：正在补静音
wire [12:0]        wav_level;          // 诊断：缓冲水位（字）
wire [9:0]         wav_pos_sec;        // 第11讲C：已播秒数（OSD 进度）
wire [9:0]         wav_tot_sec;        // 第11讲C：总秒数

// ===================== 第 16 讲：插播警报音 =====================
//   平时播卡上的 MUSIC.PCM（背景音乐）。在缩略图页里确认跳到"应急预案页"
//   （第 4 张，focus_sd = 2'd3）时，把 FPGA 内部合成的双音警笛**替换**到音源上
//   （mux 握手照常跑、只换 payload，所以数据岛不断流、不会静音），
//   一直响到**操作人员按键解除**（丝印 KEY9 → key_pressed[13]），卡上不需要第二个音频文件。
//   ★ 2026-10-31 修订一：第一版是"独占"（警报期把 WAV 和 mux 全压住），实测只有波形
//     没有声音（音频数据岛断流）⇒ 改成"替换 payload、握手不动"。
//   ★ 2026-10-31 修订二：从"响固定 60 秒"改成"闩锁到人工解除 + 300 秒兜底"。
//     现场语义与真实应急广播一致：警报不会自己停，必须有人确认。
//     想退回定时模式：把 ALARM_LATCH 改成 1'b0（时长看 ALARM_HOLD_S）。
localparam integer ALARM_HOLD_S   = 60;               // 定时模式的时长（ALARM_LATCH=0 时生效）
localparam         ALARM_LATCH    = 1'b1;             // 1 = 一直响到人工解除（默认），0 = 定时自动停
localparam integer ALARM_MAX_S    = 300;              // 闩锁模式兜底上限：5 分钟自动消音（防无人值守长鸣）
// ★ 2026-10-31 修正：这段倒计时跑在 sd_card_clk 域（100MHz），不是 video_clk（25MHz）。
//   上一版写成 25_000_000*60，实际只响 15 秒 —— 这就是"警报时间太短"的直接原因。
//   默认闩锁上限 300s × 100MHz = 3.0e10，超过 32 位，所以计数器必须 36 位。
localparam [35:0]  ALARM_LIMIT = ALARM_LATCH ? (36'd100_000_000 * ALARM_MAX_S)
                                            : (36'd100_000_000 * ALARM_HOLD_S);
localparam [1:0]   ALARM_TRIG_IMG = 2'd3;             // 只有预案页（第4张）触发
localparam         ALARM_ANY_PAGE = 1'b0;             // 1 = 任意页确认都响
// ★ 2026-10-31 新增：两条进入警报的路
//   ①【缩略图选页】KEY15 进缩略图 -> KEY12/14 移焦点到第 4 页 -> 确认退出 => 切页 + 警报
//     由 alarm_retrig / ALARM_ON_THUMB_SEL 控制；
//   ②【应急插播键】KEY10 任意时刻按一下 => 直接切第 4 页 + 警报
//     由 tk_panic_p 控制，与 ① 完全独立。
//   想让 ① 失效、只留 ②（"用插播键取代缩略图选页触发"）就把 ALARM_ON_THUMB_SEL 改 1'b0。
localparam         ALARM_ON_THUMB_SEL = 1'b1;

wire               alarm_valid;        // 保留（合成器现为 step 驱动，此线不再参与握手）
wire               alarm_step;         // 桥回送的"这一拍真的送出一个样本"
wire signed [15:0] alarm_left;
wire signed [15:0] alarm_right;
wire               alarm_on_v;         // video_clk 域的"警报进行中"电平
wire               alarm_active_v;     // 包络非零（含 5ms 收尾），桥用它做 payload 替换
wire               ack_flash_v;        // video_clk 域的"解除键回执"电平（0.7s）

// HDMI TMDS 时钟通道：固定符号 0000011111（5个0+5个1），每像素时钟循环发送
assign tmds_clk_data = 10'b0000011111;

// 调试信号（临时，定位音频问题用）：sequence_error/tone_overflow/blip_active
wire dbg_seq_err;
wire dbg_tone_overflow;
wire dbg_blip_active;

// ===================== 复位结构（第6讲：异步复位同步释放 + PLL锁定参与复位） =====================
// 两个 PLL 的锁定信号（第6讲从 open 引出）
wire sys_pll_lock;
wire video_pll_lock;

// 复位源：外部复位按键 + 两个 PLL 都锁定。任一 PLL 未锁定，系统保持复位。
// PLL 锁定需几十微秒~几毫秒，这段时长足以让电源稳定，故无需额外 POR 计数器。
wire ext_rst_n = rst_n & sys_pll_lock & video_pll_lock;

// 每个时钟域一个复位同步器（异步压下、同步释放）：
//   主时钟域 clk(50M)、sd_card_clk(100M)、ext_mem_clk(125M)、video_clk(25M)
// 输出为低有效复位，子模块（高有效复位）连接时取反。
wire sdcard_rst_n;
wire mem_rst_n;
wire video_rst_n;
wire sys_rst_n;   // 主时钟域（clk 50M）复位，供 key_matrix/seg_scan 等

reset_sync u_sys_reset (
    .clk        (clk),
    .ext_rst_n  (ext_rst_n),
    .rst_n_sync (sys_rst_n)
);
reset_sync u_sdcard_reset (
    .clk        (sd_card_clk),
    .ext_rst_n  (ext_rst_n),
    .rst_n_sync (sdcard_rst_n)
);
reset_sync u_mem_reset (
    .clk        (ext_mem_clk),
    .ext_rst_n  (ext_rst_n),
    .rst_n_sync (mem_rst_n)
);
reset_sync u_video_reset (
    .clk        (video_clk),
    .ext_rst_n  (ext_rst_n),
    .rst_n_sync (video_rst_n)
);

// ===================== 矩阵键盘扫描（KEY12=下一张, KEY13=自动播放, KEY14=上一张） =====================
// ★ 只有 key_pressed[3]（丝印 KEY15）= 答辩现场切换"写后静默期"开/关
//   2026-10-28 起不再响应其它键（原掩码 0xFF0F 会把 KEY12/13/14 也算进来）
wire key_next_n;   // KEY12 按下电平（低有效）
wire key_auto_n;   // KEY13 按下电平（低有效）
wire key_prev_n;   // KEY14 按下电平（低有效）
wire key_spare_n;  // KEY15 = 2×2 缩略图页进出键（低有效）
wire key_ack_n;    // KEY9  = 应急警报"人工解除"键（低有效，第16讲新增）
wire key_panic_n;  // KEY10 = 应急插播键（低有效，第16讲新增）
wire [3:0] key_param_n;  // 第14讲：[0]=亮度+ [1]=亮度- [2]=对比度+ [3]=对比度-（低有效）
key_matrix_4x4 u_key_matrix (
    .clk         (clk),
    .rst         (~sys_rst_n),
    .key_row     (key_row),
    .key_col     (key_col),
    .o_key_next_n(key_next_n),
    .o_key_auto_n(key_auto_n),
    .o_key_prev_n(key_prev_n),
    .o_key_spare_n(key_spare_n),
    .o_key_ack_n (key_ack_n),
    .o_key_panic_n(key_panic_n),
    .o_key_param_n(key_param_n)
);

// =============================================================================
// ★ 2026-10-29 第15讲起：KEY15 改作"切 2×2 缩略图页"，
//   写后静默期不再留演示开关（它是修好后的常态，答辩不需要现场复现 bug）。
//   —— 2026-10-31 定案：这个演示开关**彻底取消**，guard_dis 恒 0（静默期常开）。
//      详细说明见《工程状态文档.md》第 8 讲阶段 A 那条"答辩演示支持"的更正。
// =============================================================================
wire guard_dis = 1'b0;

// =============================================================================
// 第14讲：亮度 / 对比度按键调节
//   键位（低有效）：[0]=亮度+  [1]=亮度-  [2]=对比度+  [3]=对比度-
//   短按步进 16；按住超过 400ms 后每 80ms 连发；两端饱和、不回绕。
//   默认 brightness=255 / contrast=128 → 上电画面与第9讲**完全一致**，
//   可作为回归基线：如果没按任何键但画面偏暗/偏灰，说明参数初值接错了。
// =============================================================================
wire [7:0] adj_bright;
wire [7:0] adj_contrast;
wire       adj_show;
wire       adj_any;

param_adjust_ctrl #(
    .CLKS_PER_MS     (50_000),
    .DEBOUNCE_MS     (20),
    .REPEAT_DELAY_MS (400),
    .REPEAT_RATE_MS  (80),
    .SHOW_MS         (3000),
    .STEP            (16)
) u_param_adjust (
    .clk          (clk),
    .rst_n        (sys_rst_n),
    .key_b_inc_n  (key_param_n[0]),
    .key_b_dec_n  (key_param_n[1]),
    .key_c_inc_n  (key_param_n[2]),
    .key_c_dec_n  (key_param_n[3]),
    .brightness   (adj_bright),
    .contrast     (adj_contrast),
    .show_en      (adj_show),
    .any_pulse    (adj_any)
);

// 保持你原来的 TF / SDRAM / video 时钟；PLL reset 接外部复位（第6讲：系统复位时 PLL 也复位）
sys_pll sys_pll_m0(
    .refclk     (clk),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft),
    .reset      (~rst_n),
    .locked     (sys_pll_lock)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk),
    .reset      (~rst_n),
    .locked     (video_pll_lock)
);

// =============================================================================
// 第8讲 阶段A：四 bank 写通道（media_frame_loader + 跨时钟域搬运）
//
// 数据流：BMP 解码(sd_card_clk 100M) → loader FIFO+burst → CDC(100M→125M)
//         → SDRAM 控制器 App_wr_*（ext_mem_clk 125M）
//
// 与原来的差别：
//   原来 sd_card_bmp 直接输出 write_en/write_data 给 frame_read_write，
//   由 frame_fifo_write 在 125M 域做跨域 + V_FLIP + 地址生成。
//   现在这三件事分别由 loader(V_FLIP+burst计数) + CDC(跨域) 承担。
// =============================================================================
wire        loader_load_start;
wire [1:0]  loader_image_index;
wire        loader_pixel_valid;
wire [23:0] loader_pixel_rgb;
wire        loader_frame_commit;
wire        loader_load_busy;
wire        loader_load_done;
wire [3:0]  loader_image_complete;
wire [7:0]  loader_memory_error;
wire        loader_app_wr_en;
wire [ADDR_BITS-1:0] loader_app_wr_addr;
wire [3:0]  loader_app_wr_mask;
wire [31:0] loader_app_wr_data;

wire        cdc_src_full;
wire        cdc_src_overflow;
wire        cdc_app_wr_en;
wire [ADDR_BITS-1:0] cdc_app_wr_addr;
wire [3:0]  cdc_app_wr_mask;
wire [31:0] cdc_app_wr_data;
wire        cdc_dst_idle;
wire        rd_burst_busy;   // 读侧 burst 进行中（来自 media_frame_reader）
wire        wr_burst_busy;   // 写侧 burst 进行中/即将开始（送往 media_frame_reader）
wire        wr_burst_active; // ★ 写侧"真正在写"的窄窗口（fifo_rd_valid），读侧窄避让用
wire        reader_display_blank; // ★ 读侧给的垂直消隐期标志（写只在消隐期启动）
// ★ 回退开关：置 0 = 取消"写只在消隐期"的分时（恢复 w26 的随时可写行为）
localparam  WR_BLANK_GATE = 1'b0;   // ★ 2026-10-31 22:59 关掉：实测"写只在消隐期"会把加载饿死
                                    //   （消隐期正好是 reader 抢预取的窗口，写拿不到总线 →
                                    //    media_frame_loader 报 ERR_WRITE_STALL=0x46、黑屏、音频 POS 卡 1）。
                                    //   本版保留窄避让（ext_wr_active），取消消隐期门控。

// ---- write_req 上升沿 → loader 的单拍 load_start（保持 sd_card_bmp 握手不变）----
reg  sd_wr_req_d;
always @(posedge sd_card_clk or negedge sdcard_rst_n) begin
    if (!sdcard_rst_n)
        sd_wr_req_d <= 1'b0;
    else
        sd_wr_req_d <= sd_card_write_req;
end
assign loader_load_start  = sd_card_write_req & ~sd_wr_req_d;
// ★ 单缓冲诊断：写侧固定 BUF0（配合下面读侧固定 read_addr_index=0）
assign loader_image_index = DIAG_SINGLE_BUF ? 2'd0 : write_buf_idx;

// ---- 像素流对接：sd_card_bmp 的 write_data 是 {R,G,B,8'b0}，剥出 24 位 RGB ----
// ★ 第13讲：在"解码器输出"与"loader 输入"之间插入写侧最近邻缩放器。
//   sd_card_bmp 吐的是源图原始像素（任意分辨率，行 padding 已剔除），
//   bmp_scale_640x480 把它抽样成恰好 640×480。于是 loader / reader /
//   write_commit / 转场 / 亮度 / 波形全链路都不需要知道源图多大。
wire        src_pixel_valid;
wire [23:0] src_pixel_rgb;
wire [15:0] src_img_width;
wire [15:0] src_img_height;

assign src_pixel_valid = sd_card_write_en;
assign src_pixel_rgb   = sd_card_write_data[31:8];

wire        loader_pixel_ready;   // loader 反压：当前能否再收一个像素
wire        loader_pixel_fire;    // 缩放器实际送出的像素（1:1 旁路时下游无否决权）
wire        scale_overflow;

// ---- 1:1 直通判定（第13讲）----
// 源图恰好 640×480 时整帧旁路缩放器，像素流与第 12 讲逐拍完全一致。
// ★ 必须在这里**打一拍**给出，不能在缩放器内部拿 src_width/src_height 现场比：
//   那样 byp_eff 会变成「解码器宽度寄存器 → 16 位比较 → 2:1 mux」三到四级组合逻辑，
//   直接串进 out_valid → media_frame_loader 的 FIFO 使能链。那条链本就是全工程最紧的
//   （11 级、net 占 66%），实测一加就是 setup 由 +0.107ns 掉到 -0.543ns。
//   打一拍后只剩 1 级 LUT5。安全性：img_width/height 是 BMP 头解析结果，
//   像素区起点（pixel_offset）就由它们算出，所以它们必然早于第一个源像素稳定。
reg src_1to1_r;
always @(posedge sd_card_clk or negedge sdcard_rst_n) begin
    if (!sdcard_rst_n)
        src_1to1_r <= 1'b0;
    else
        src_1to1_r <= (src_img_width  == 16'd640) &&
                      (src_img_height == 16'd480);
end

bmp_scale_640x480 #(
    .DST_W (640),
    .DST_H (480)
) u_bmp_scale (
    .clk        (sd_card_clk),
    .rst        (~sdcard_rst_n),          // 模块内高有效复位
    .start      (loader_load_start),
    .src_width  (src_img_width),
    .src_height (src_img_height),
    .mode_1to1  (src_1to1_r),
    .in_valid   (src_pixel_valid),
    .in_rgb     (src_pixel_rgb),
    .out_ready  (loader_pixel_ready),
    .out_valid  (loader_pixel_valid),
    .out_rgb    (loader_pixel_rgb),
    .out_fire   (loader_pixel_fire),
    .overflow   (scale_overflow)
);

// ---- CDC 排空信号同步回 100M 域（125M → 100M，两级同步）----
reg cdc_idle_s1, cdc_idle_s2;
always @(posedge sd_card_clk or negedge sdcard_rst_n) begin
    if (!sdcard_rst_n) begin
        cdc_idle_s1 <= 1'b0;
        cdc_idle_s2 <= 1'b0;
    end else begin
        cdc_idle_s1 <= cdc_dst_idle;
        cdc_idle_s2 <= cdc_idle_s1;
    end
end

// ---- 整帧提交 + 写完成 toggle ----
// 逻辑已抽到 media_write_commit（像素计数出 frame_commit、等 CDC 排空后翻 toggle）。
// 抽出来的原因：这段曾经在顶层内联时出过"每拍翻转导致 0x38"的 bug，
// 内联写法无法被 tb 直接覆盖。现在 tb_write_two_frames 连跑两帧验证的就是
// 这份 RTL 本身，不再有"仿真一份、综合另一份"的风险。
media_write_commit #(
    .FRAME_PIXELS       (19'd307200),
    .DRAIN_GUARD_CYCLES (8'd255)
) u_write_commit (
    .clk                (sd_card_clk),
    .rst_n              (sdcard_rst_n),
    .load_start         (loader_load_start),
    // ★ 第13讲：必须只数"真正被 loader 收下的像素"，不能直接数 loader_pixel_valid。
    //   第 12 讲之前 pixel_valid 是解码器的单拍脉冲，数电平等价于数脉冲；
    //   现在它来自写侧缩放器，是 valid/ready 握手里的 valid —— **电平**，下游一反压
    //   就连续多拍为高，计数器会把同一个像素数成很多个，frame_commit 提前触发。
    //   ★ 用缩放器给的 out_fire 而不是自己拼 `&& loader_pixel_ready`：
    //     640×480 源图在缩放器里走 1:1 直通旁路，此时下游没有否决权
    //     （第 12 讲本来就不看 ready），若按 && ready 计数就会少算 → frame_commit
    //     永远不来 → 加载卡死。out_fire 内部已经把这个区别处理掉了。
    .pixel_valid        (loader_pixel_fire),
    .load_done          (loader_load_done),
    .memory_error       (loader_memory_error),
    .cdc_idle           (cdc_idle_s2),
    .frame_commit       (loader_frame_commit),
    .frame_write_toggle (frame_write_toggle)
);

// ---- SDRAM 初始化完成的跨域（125M → 100M，慢变化电平，仍做两级同步）----
reg sdr_init_s1, sdr_init_s2;
always @(posedge sd_card_clk or negedge sdcard_rst_n) begin
    if (!sdcard_rst_n) begin
        sdr_init_s1 <= 1'b0;
        sdr_init_s2 <= 1'b0;
    end else begin
        sdr_init_s1 <= Sdr_init_done;
        sdr_init_s2 <= sdr_init_s1;
    end
end

media_frame_loader #(
    .ADDR_WIDTH    (21),
    .IMAGE_WORDS   (307200),
    // ★ 必须与 BUF0..3_ADDR 的平移量一致（见文件头 BUF_BASE_OFFSET）
    .BASE_OFFSET   (BUF_BASE_OFFSET),
    .FIFO_DEPTH    (512),
    .BURST_SIZE    (128),
    .STALL_TIMEOUT (65535),
    .WRITE_V_FLIP  (1),
    .FRAME_WIDTH   (640),
    .FRAME_HEIGHT  (480)
) u_frame_loader (
    .clk                 (sd_card_clk),
    .rst_n               (sdcard_rst_n),
    .load_start          (loader_load_start),
    .image_index         (loader_image_index),
    .pixel_valid         (loader_pixel_valid),
    .pixel_rgb           (loader_pixel_rgb),
    .frame_commit        (loader_frame_commit),
    .decode_error        (8'd0),                 // 解码错误由 sd_card_bmp 自己处理
    .load_busy           (loader_load_busy),
    .load_done           (loader_load_done),
    .image_complete      (loader_image_complete),
    .memory_error        (loader_memory_error),
    .controller_init_done(sdr_init_s2),
    .controller_busy     (1'b0),                 // 控制器内部自己处理 busy
    .refresh_active      (1'b0),
    .app_wr_en           (loader_app_wr_en),
    .app_wr_addr         (loader_app_wr_addr),
    .app_wr_mask         (loader_app_wr_mask),
    .app_wr_data         (loader_app_wr_data),
    // 下游 CDC 命令 FIFO 满就暂停出队，绝不丢像素
    .out_full            (cdc_src_full),
    // 第13讲：反压给写侧缩放器（垂直放大时要重复输出整行，必须按需输出）
    .pixel_ready         (loader_pixel_ready)
);

media_sdram_write_cdc #(
    .ADDR_WIDTH      (21),
    // ★ 2026-10-27：512 → 1024（FIFO_ADDR_WIDTH 9 → 10）
    //   行缓冲把写侧变成「整行 640 连发」，一行的突发量（640）本身就超过
    //   原 FIFO 的 512 深，加深到 1024 后单次突发能整个装下，反压只会发生在
    //   跨行时，不会在行内溢出 —— 这是**降低**溢出概率的一步。
    //   ★ 但它不是 ERR:3A 的根治：3A 的真因是"满"这根线和写使能差一拍
    //     （见 media_sdram_write_cdc 里的 FULL_RESERVE 说明），
    //     所以只加深深度、不修相位差，3A 照样会报。两件事都做了才彻底。
    .FIFO_ADDR_WIDTH (10),
    // ★ 64 字一小块：单次占用总线 0.5μs，配合模块内 256 拍强制让位，
    //   把写通道对读通道的干扰压到最低（写侧需求仅占总线 0.8%，不需要长 burst）
    .BURST_SIZE      (64),
    // ★ 写后静默期：0 = 复现修复前的横条，24 = 正常
    .GUARD_CYCLES    (GUARD_CYCLES_CFG)
) u_wr_cdc (
    .src_clk         (sd_card_clk),
    .src_rst_n       (sdcard_rst_n),
    .src_wr_en       (loader_app_wr_en),
    .src_wr_addr     (loader_app_wr_addr),
    .src_wr_mask     (loader_app_wr_mask),
    .src_wr_data     (loader_app_wr_data),
    .src_full        (cdc_src_full),
    .src_overflow    (cdc_src_overflow),
    // ★ 2026-10-27 修 ERR:0x38「切不了图」永久卡死：
    //   上一帧如果中途少给了字，会留下一个 1~63 字的不满块发不出去，
    //   目的侧 dst_idle 永远等不到 → frame_write_toggle 不翻 → 上层报 0x38，
    //   而 loader 自己一点错都不报（它已经交完 307200 个字了）。
    //   把每帧的启动脉冲送进去，让它在新帧开头补一个块边界把残留放掉。
    .src_frame_start (loader_load_start),
    .dst_clk         (ext_mem_clk),
    .dst_rst_n       (mem_rst_n),
    .dst_app_wr_en   (cdc_app_wr_en),
    .dst_app_wr_addr (cdc_app_wr_addr),
    .dst_app_wr_mask (cdc_app_wr_mask),
    .dst_app_wr_data (cdc_app_wr_data),
    .dst_idle        (cdc_dst_idle),
    // 第8讲：与读侧互斥握手（SDRAM App 端口不支持读写并发）
    .dst_rd_busy     (rd_burst_busy),
    .dst_wr_busy     (wr_burst_busy),
    // ★ 2026-10-28：SDRAM 刷新窗口（官方规则：读写都要避让，写侧原来漏了）
    .dst_ref_vld     (Sdr_init_ref_vld),
    // ★ 答辩演示：KEY8 实时切换写后静默期（0=生效/无横条，1=关闭/复现横条）
    .guard_dis       (guard_dis),
    // ★ 第15讲：缩略图页生成期间让出写通道（1 = 不要发新的写 burst）
    //   引擎自己的写组优先占 App_wr_*，CDC 必须整体停手，否则它的 burst 会被
    //   半路打断（丢了某几个字，那一帧就永久缺字）。
    .dst_wr_hold     (tb_busy),
    // ★ 2026-10-31：加载写只在垂直消隐期启动（活动视频期间总线归显示读）
    .dst_blanking    (WR_BLANK_GATE ? reader_display_blank : 1'b1),
    // ★ 2026-10-31：真正的写命令窗口，给读侧做窄避让
    .dst_wr_active   (wr_burst_active)
);
// =============================================================================
// 第15讲：缩略图页交互（sd_card_clk 域，100MHz）
//   按键丝印 → 功能：
//     KEY15 → 进入/退出 2×2 缩略图页
//     KEY12 → 焦点右移（+1，按 idx = {qy,qx} 的象限序 0=左上 1=右上 2=左下 3=右下）
//     KEY14 → 焦点左移（-1）
//     KEY13 → 确认：跳到焦点那张图，并退出缩略图页
//   非缩略图模式下 KEY12/13/14 仍是原来的"下一张 / 自动播放 / 上一张"。
//   为什么全放在 sd 域做：sd_card_bmp 的 img_idx 与跳转请求都在这个域，
//   焦点/view 只是慢变电平，往 video 域各打两级同步就够，不需要握手。
// =============================================================================
wire tk_next_p, tk_auto_p, tk_prev_p, tk_mode_p, tk_ack_p, tk_panic_p;

// 复用 sd_card_bmp.v 里已有的 key_press_debounce（20ms 消抖 + 单拍 press_pulse），
// 不再重复定义同名模块 —— 重名会让 TD 在综合阶段直接停。
key_press_debounce u_tk_next (
    .clk (sd_card_clk), .rst (~sdcard_rst_n), .button_in (key_next_n),  .press_pulse (tk_next_p));
key_press_debounce u_tk_auto (
    .clk (sd_card_clk), .rst (~sdcard_rst_n), .button_in (key_auto_n),  .press_pulse (tk_auto_p));
key_press_debounce u_tk_prev (
    .clk (sd_card_clk), .rst (~sdcard_rst_n), .button_in (key_prev_n),  .press_pulse (tk_prev_p));
key_press_debounce u_tk_mode (
    .clk (sd_card_clk), .rst (~sdcard_rst_n), .button_in (key_spare_n), .press_pulse (tk_mode_p));
// 第16讲：警报人工解除键（丝印 KEY9 → key_pressed[13]）
key_press_debounce u_tk_ack (
    .clk (sd_card_clk), .rst (~sdcard_rst_n), .button_in (key_ack_n),   .press_pulse (tk_ack_p));
// 第16讲：应急插播键（丝印 KEY10 → key_pressed[14]）
//   按一下 = 立刻跳预案页 + 拉响警报，不依赖缩略图页。
key_press_debounce u_tk_panic (
    .clk (sd_card_clk), .rst (~sdcard_rst_n), .button_in (key_panic_n), .press_pulse (tk_panic_p));

// ---- 第15讲：退出缩略图页 -> 直接切到焦点那张 ----
// 两条路一起铺：
//   ① 退出沿先发 jump_req（sd_card_bmp 里最高优先级的零延迟分支）—— 一帧直达，最理想；
//   ② 10ms 后复核 media_id：如果还停在旧图上（说明 jump 那条路没生效），
//      再退回"注入 key_next/key_prev 电平"的老办法一步步走过去兜底。
// 目标驱动：每走一步都重新比较 media_id 与 focus_sd，到了就停，最多按 4 次。
localparam integer WALK_HOLD = 6_000_000;    // 按下 60ms @100MHz（内部消抖需 20ms）
localparam integer WALK_GAP  = 40_000_000;   // 间隔 400ms，留够第 10 讲淡入淡出转场
localparam integer ARM_WAIT  = 1_000_000;    // 10ms：给 jump_req 留出反应时间

reg        walk_busy;
reg        walk_arm;          // 1 = 已发 jump_req，正在等 10ms 后复核
reg        walk_dir;          // 1 = 注入 key_next，0 = 注入 key_prev
reg        walk_lvl;          // 注入电平：低 = 按下
reg        walk_ph;           // 0 = 按下阶段，1 = 松开/间隔阶段
reg  [1:0] walk_tries;        // 已经按了几次
reg [25:0] walk_tmr;
reg  [3:0] dbg_confirm_cnt;   // 确认次数（调试读数）
reg [26:0] ack_flash;         // 按下解除键后的提示计时（0.7s，OSD 显示 ERR:C9 作回执）

// 第16讲：插播警报计时（sd 域，100MHz）
reg         alarm_en_sd;      // 1 = 警报进行中（闩锁到人工解除）
reg  [35:0] alarm_cnt;        // 计时计数器（闩锁上限 300s@100MHz = 3.0e10，需要 36 位）

wire [1:0] walk_delta = focus_sd - media_id;     // 焦点相对当前图的位置 0..3
// 应急插播键相对当前图的位置（目标恒为预案页）
wire [1:0] panic_delta = ALARM_TRIG_IMG - media_id;
// 解除警报后要回的"第 1 张"相对当前图的位置（1 = 下一张，3 按"上一张"走更近）
wire [1:0] home_delta  = 2'd0 - media_id;
wire       panic_req   = tk_panic_p;             // 单拍：丝印 KEY10 被按下（20ms 消抖后）

reg thumb_view_sd_d;                             // 用于抓"退出缩略图页"的下降沿

// 第16讲：警报重触发条件 —— 每一次"退出缩略图页"且焦点停在预案页都重来一遍，
//   与"这次需不需要切图（walk_delta）"完全无关。用户实测："重新进选图出来后没有警报"
//   就是被 walk_delta != 0 这个条件挡掉的。
//   2026-10-31：外层加 ALARM_ON_THUMB_SEL，方便"只用应急插播键"的版本一键关掉这条。
wire alarm_retrig = ALARM_ON_THUMB_SEL && thumb_view_sd_d && !thumb_view_sd
                 && (ALARM_ANY_PAGE || (focus_sd == ALARM_TRIG_IMG));

always @(posedge sd_card_clk or negedge sdcard_rst_n) begin
    if (!sdcard_rst_n) begin
        thumb_view_sd   <= 1'b0;
        thumb_view_sd_d <= 1'b0;
        focus_sd        <= 2'd0;
        jump_req_sd     <= 1'b0;
        jump_idx_sd     <= 2'd0;
        walk_busy       <= 1'b0;
        walk_arm        <= 1'b0;
        walk_dir        <= 1'b1;
        walk_lvl        <= 1'b1;
        walk_ph         <= 1'b1;
        walk_tries      <= 2'd0;
        walk_tmr        <= 26'd0;
        dbg_confirm_cnt <= 4'd0;
        ack_flash       <= 27'd0;
        alarm_en_sd     <= 1'b0;
        alarm_cnt       <= 36'd0;
    end else begin
        jump_req_sd     <= 1'b0;
        thumb_view_sd_d <= thumb_view_sd;

        // 只有四张都加载完（缩略图页内容才有效）时才允许进入
        if (tk_mode_p && (thumb_view_sd || (loader_image_complete == 4'b1111)))
            thumb_view_sd <= ~thumb_view_sd;

        if (thumb_view_sd) begin
            if (tk_next_p) focus_sd <= focus_sd + 2'd1;
            if (tk_prev_p) focus_sd <= focus_sd - 2'd1;
            if (tk_auto_p) thumb_view_sd <= 1'b0;   // 确认 = 退出（退出沿负责切图）
        end

        // 退出沿：先发 jump_req 直达，并预约 10ms 后的复核
        if (thumb_view_sd_d && !thumb_view_sd && !walk_busy && !walk_arm
            && (walk_delta != 2'd0)) begin
            dbg_confirm_cnt <= dbg_confirm_cnt + 4'd1;
            jump_req_sd <= 1'b1;
            jump_idx_sd <= focus_sd;
            walk_arm    <= 1'b1;
            walk_dir    <= (walk_delta == 2'd3) ? 1'b0 : 1'b1;
            walk_tmr    <= 26'd0;
            walk_tries  <= 2'd0;
        end

        // 第16讲：警报触发 **独立**于"要不要切图"
        //   ★ 2026-10-31 修正：上一版把它嵌在 (walk_delta != 0) 里面。第一次确认到第 4 页后
        //     media_id 已经等于 focus_sd，再选同一张时 walk_delta = 0 ⇒ 整个 if 都不进 ⇒
        //     警报不响，只有先切到别的图再回来才响，现象就是"有冷却、要等一会儿"。
        //   现在只看"退出沿 + 焦点是不是预案页"，判定见上面的 alarm_retrig，
        //   由下面的倒计时块统一把 alarm_en_sd / alarm_cnt 清零重来。

        // 10ms 后复核：jump 生效就什么都不做；没生效才按第一次
        if (walk_arm) begin
            if (walk_tmr >= (ARM_WAIT - 1)) begin
                walk_arm <= 1'b0;
                walk_tmr <= 26'd0;
                if (media_id != focus_sd) begin
                    walk_busy  <= 1'b1;
                    walk_ph    <= 1'b0;
                    walk_lvl   <= 1'b0;
                    walk_tries <= 2'd1;
                end
            end else walk_tmr <= walk_tmr + 26'd1;
        end

        // 走步时序：按下 60ms -> 松开/间隔 400ms -> 复核是否到目标
        if (walk_busy) begin
            if (walk_ph == 1'b0) begin
                if (walk_tmr >= (WALK_HOLD - 1)) begin
                    walk_tmr <= 26'd0;
                    walk_ph  <= 1'b1;
                    walk_lvl <= 1'b1;           // 松开
                end else walk_tmr <= walk_tmr + 26'd1;
            end else begin
                if (walk_tmr >= (WALK_GAP - 1)) begin
                    walk_tmr <= 26'd0;
                    if ((media_id == focus_sd) || (walk_tries >= 2'd4)) begin
                        walk_busy  <= 1'b0;
                        walk_lvl   <= 1'b1;     // 收尾必须松开，别把键一直按着
                        walk_tries <= 2'd0;
                    end else begin
                        walk_tries <= walk_tries + 2'd1;
                        walk_ph    <= 1'b0;
                        walk_lvl   <= 1'b0;     // 再按一次
                    end
                end else walk_tmr <= walk_tmr + 26'd1;
            end
        end

        // ---- 第16讲：应急插播键（丝印 KEY10）----
        //   任意时刻、任意页面（含缩略图页内）按下：
        //     立刻切到预案页（第 4 张）+ 拉响警报，不等缩略图、不等确认。
        //   为什么放在走步逻辑之后：同一组 reg（thumb_view_sd / walk_* / jump_* / focus_sd）
        //   后面写的非阻塞赋值生效，这样"正在走步时按插播键"也能被立刻打断并改道。
        //   现场意义：值班员不需要记得"先按 KEY15 再挑第 4 张"，一颗键就是撤离广播。
        if (panic_req) begin
            thumb_view_sd <= 1'b0;                    // 缩略图页里按也直接退出
            walk_busy     <= 1'b0;                    // 取消正在进行的走步
            walk_lvl      <= 1'b1;                    // 必须松开注入电平
            walk_tries    <= 2'd0;
            focus_sd      <= ALARM_TRIG_IMG;          // 焦点落到预案页
            jump_req_sd   <= 1'b1;                    // 零延迟直达
            jump_idx_sd   <= ALARM_TRIG_IMG;
            walk_arm      <= (panic_delta != 2'd0);   // 已经在这页就不必再兜底走步
            walk_dir      <= (panic_delta == 2'd3) ? 1'b0 : 1'b1;
            walk_tmr      <= 26'd0;
        end

        // 第16讲：警报重触发 + 解除（人工/超时）+ 计时（三者必须在同一个块里写）
        //   ★ 为什么要合成一个块：alarm_en_sd / alarm_cnt 是同一组 reg，
        //     拆成并列的 if 会出现"同一拍既置 1 又清零，最后一条生效"的顺序坑，
        //     上一版就因此出现过"60 秒内重复确认只接着原倒计时走，像有冷却"。
        //   优先级：重触发 > 解除 > 计时
        if (panic_req || alarm_retrig) begin
            alarm_en_sd <= 1'b1;
            alarm_cnt   <= 36'd0;
        end else if (alarm_en_sd && (tk_ack_p || (alarm_cnt >= (ALARM_LIMIT - 36'd1)))) begin
            // ---- 解除警报：人工按丝印 KEY9，或 300s 兜底超时（语义完全相同）----
            //   1) 消音；2) 自动回到第 1 张图（用户 2026-10-31 要求）：
            //      预案页只该在警报期间占屏，解除后终端要自动恢复成常态宣教的第一页，
            //      无人值守时也不会永远停在"紧急疏散"上。
            //   回图仍然走"jump 直达 + 10ms 复核 + 走步兜底"这套已验证的路径，
            //   并把可能还在跑的走步先掐掉（否则它会把画面又推回预案页）。
            alarm_en_sd <= 1'b0;
            alarm_cnt   <= 36'd0;
            walk_busy   <= 1'b0;
            walk_lvl    <= 1'b1;                  // 必须松开注入电平，别把键一直按着
            walk_tries  <= 2'd0;
            focus_sd    <= 2'd0;                  // 焦点复位到第 1 张
            jump_req_sd <= 1'b1;                  // 零延迟直达第 1 张
            jump_idx_sd <= 2'd0;
            walk_arm    <= (home_delta != 2'd0);  // 已经就是第 1 张就不用兜底
            walk_dir    <= (home_delta == 2'd3) ? 1'b0 : 1'b1;
            walk_tmr    <= 26'd0;
        end else if (alarm_en_sd) begin
            alarm_cnt <= alarm_cnt + 36'd1;
        end

        // 解除键回执：按下后 0.7s 内 OSD 第三行显示 ERR:C9，
        //   用来现场确认"我按的那颗键确实接在 ACK_MASK 上"（不改键位也能自检）。
        if (tk_ack_p)
            ack_flash <= 27'd70_000_000;      // 0.7s @100MHz
        else if (ack_flash != 27'd0)
            ack_flash <= ack_flash - 27'd1;
    end
end

// 缩略图模式下把三个功能键从 sd_card_bmp 手里借走，避免"移焦点"同时"切图"；
// 退出后的走步期间再把 key_next/key_prev 借给 walk 注入
wire key_next_bmp = thumb_view_sd ? 1'b1 : ((walk_busy &&  walk_dir) ? walk_lvl : key_next_n);
wire key_auto_bmp = thumb_view_sd ? 1'b1 : key_auto_n;
wire key_prev_bmp = thumb_view_sd ? 1'b1 : ((walk_busy && !walk_dir) ? walk_lvl : key_prev_n);

// ===================== TF 多图扫描与缓存（双缓冲） =====================
sd_card_bmp #(
    .CLK_FREQ_HZ       (100_000_000),
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd131071),
    .SCAN_TARGET_COUNT (3'd4),
    // ★ 冷启动健壮性：16.67MHz（见文件头 SD_SPI_HIGH_DIV 说明），代价上电多等约 0.6s
    .SPI_HIGH_DIV      (SD_SPI_HIGH_DIV),
    // ★ 第 11 讲：MUSIC.PCM 播放开关 / 是否循环
    .WAV_EN            (WAV_PLAY_EN),
    .WAV_LOOP          (WAV_LOOP_EN),
    // ★ 第13讲：加载超时从 3s 放大到 10s
    //   支持任意分辨率后，大图读卡时间显著变长：
    //     640×480  → 0.44s（原设计假设）
    //     1024×768 → 1.13s
    //     1920×1080→ 3.0s  ← 原来的 3s 兜底会把它误杀
    //   10s 仍远小于"一天一充"的忍耐阈值，同时给了慢卡足够余量。
    .LOAD_TIMEOUT_CYCLES (1_000_000_000)
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (~sdcard_rst_n),
    .key_next          (key_next_bmp),
    .key_auto          (key_auto_bmp),
    .key_prev          (key_prev_bmp),
    // ★ 第15讲：缩略图页 KEY13 确认 → 直接跳转到选中那张图（零延迟）
    .jump_req          (jump_req_sd),
    .jump_idx          (jump_idx_sd),
    .state_code        (state_code),
    .recover_state     (recover_state),
    .disp_num          (disp_num),
    .media_id          (media_id),
    .bmp_error_code    (bmp_err_code),
    .img_total         (img_total),
    .sd_init_done_out  (sd_init_done),
    .load_error_out    (load_error),
    .ld3s_pulse        (ld3s_pulse),
    .ld3s_src          (ld3s_src),
    .fat_dbg_pulse     (fat_dbg_pulse),
    .fat_dbg_ok        (fat_dbg_ok),
    .fat_dbg_val       (fat_dbg_val),
    .scan_diag_pulse   (scan_diag_pulse),
    .scan_diag_zero    (scan_diag_zero),
    .scan_diag_val     (scan_diag_val),
    // ★ 第13讲：这两个不再传"期望尺寸"，而是"支持的上限"。
    //   上限受 SPI 读卡时间与加载超时兜底约束：
    //   1920×1080×3 = 6.2MB，16.67MHz 位率约 3.0s，LOAD_TIMEOUT 已放到 10s。
    .max_width         (16'd2048),
    .max_height        (16'd1536),
    // 真实分辨率输出（供写侧缩放器抽样；OSD 也会显示）
    .src_width_out     (src_img_width),
    .src_height_out    (src_img_height),
    .display_valid     (display_valid),

    .write_finish_toggle(frame_write_toggle),   // 第8讲：改为等 CDC 排空后才翻转
    .write_buf_idx     (write_buf_idx),
    .disp_buf_idx      (disp_buf_idx),
    // ★ 2026-10-31（B 方案）：视频通道解锁判据用这两个（bank 有没有内容 / 正在写哪个）
    .bank_valid_o      (bmp_bank_valid),
    .load_bank_o       (bmp_load_bank),

    // ---- 第 11 讲阶段 B：WAV 音频（sd_card_clk 域，接 wav_stream_player 写侧）----
    .wav_fifo_room     (wav_fifo_room),
    .wav_fifo_wr_en    (wav_fifo_wr_en),
    .wav_fifo_wr_data  (wav_fifo_wr_data),
    .wav_play_en       (wav_play_en),
    .wav_audio_found   (wav_audio_found),
    .wav_busy          (wav_busy),
    .wav_fs_error      (wav_fs_error),
    .wav_pos_sec       (wav_pos_sec),
    .wav_tot_sec       (wav_tot_sec),

    .Sdr_init_done     (Sdr_init_done),

    .write_req         (sd_card_write_req),
    .write_req_ack     (loader_load_start),     // 上升沿即 loader 的启动脉冲，单拍应答
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .SD_nCS            (sd_ncs),
    .SD_DCLK           (sd_dclk),
    .SD_MOSI           (sd_mosi),
    .SD_MISO           (sd_miso)
);

seg_decoder seg_decoder_m0(
    .bin_data          (disp_num),
    .seg_data          (seg_data_0)
);
// 第8讲：写通道（loader）错误码优先展示。
// sd_card_bmp 只看到"写完成信号没来"，统一报 0x38(ERR_UPSTREAM)；
// loader 内部的真实错误码（短帧 0x43 / 像素超量 0x44 / 空闲超时 0x47 等）
// 之前没上报，实机只能看到一个笼统的 0x38，无法定位。
// loader 的 memory_error 在每次 load_start 清零、失败帧会保持到下次加载，
// 所以这里直接顶上来给数码管 / OSD / UART 用：显示 0x38 说明确实没收到 toggle，
// 显示 0x4x 说明 loader 自己判了失败。
// =============================================================================
// ★ 第13讲诊断：行缓冲到底有没有丢像素（2026-10-27 加）
//
//   实机上 B 轮（走行缓冲）的竖线呈"整段消失"，而 A 轮（1:1 旁路）干净。
//   竖线整段消失 ≠ 显示器放大能造成的效果（非整数倍放大只会让线宽在 2/3 像素
//   间交替），所以必须先分清是「缩放器 DROP 了像素」还是「SDRAM 读写错」。
//
//   `bmp_scale_640x480.overflow` 本来只在 DROP 分支（行缓冲两块都满、这一份写
//   不进去）置位，且每次 load_start 清零 —— 切图快时一闪就没了，拍不到。
//   这里做一次"曾经发生过"的锁存（只有复位能清），于是：
//     OSD / 数码管出现 ERR:39  → 行缓冲确实丢过像素（问题在缩放器写侧）
//     ERR 一直为空             → 缩放器没丢，问题在 SDRAM 读写或显示链路
//   一次烧录即可定性，不用再猜。
//   注意 0x39 在 bmp_err_def.vh 里未定义（那边只用到 0x31~0x38、0x41~0x47）。
// =============================================================================
reg scale_ovf_seen;
always @(posedge sd_card_clk or negedge sdcard_rst_n) begin
    if (!sdcard_rst_n)
        scale_ovf_seen <= 1'b0;
    else if (scale_overflow)
        scale_ovf_seen <= 1'b1;
end

// ★ 第二个诊断量（2026-10-27 晚）：跨时钟域搬运 FIFO 丢字
//   现象：复位后前两张干净、多切几轮噪点重现 —— 与"每次加载静默丢几个字"吻合。
//   `media_sdram_write_cdc` 的 src 侧 FIFO 只有 512 深（FIFO_ADDR_WIDTH=9），
//   而行缓冲把写侧变成"整行 640 连发"，突发量超过 FIFO 深度；
//   一旦反压链有任何一拍没接住，这里就会丢字且**不报任何错**（像素总数仍够，
//   frame_commit 照常触发），表现出来就是画面零散噪点。
//   这条信号一直是悬空的，现在锁存出来：
//     ERR:3A → CDC FIFO 溢出过（写侧突发把它冲爆了）→ 加深 FIFO 或降突发
reg cdc_ovf_seen;
always @(posedge sd_card_clk or negedge sdcard_rst_n) begin
    if (!sdcard_rst_n)
        cdc_ovf_seen <= 1'b0;
    else if (cdc_src_overflow)
        cdc_ovf_seen <= 1'b1;
end

// =============================================================================
// 构建指纹（第15讲重建版）：OSD 第三行显示 ERR:<MARK>
//   用途：证明"这一版真的进了 FPGA"。上一轮排查吃过亏 —— 照片对不上时无法分辨
//   "没烧进去"和"烧进去了但功能没跑"，靠这个码才把问题切开。
//   每编一版 MARK 加 1；出正式版前必须改回 8'h00。
//   DIAG_SHOW_TB = 1 时，在没有真实错误码的前提下显示生成引擎状态：
//     {2'b00, tb_busy, tb_done, loader_image_complete}
//     0x0F = 四张已加载、引擎还没动 / 0x2F = 正在生成 / 0x1F = 整页生成完成
// =============================================================================
localparam [7:0] DIAG_MARK    = 8'h00;
localparam       DIAG_SHOW_TB = 1'b0;
// DIAG_THUMB = 1 时，没有真实错误码时第三行显示缩略图调试读数：
//   {2'b10, 确认次数[3:0], 焦点[1:0]}  -> 0x80..0xBF
//   0x80 = 这一版已跑起来、还没按过 KEY13 确认；0x81 = 确认过 1 次、焦点在 0
// ★ 正式版必须为 0
localparam       DIAG_THUMB   = 1'b0;
// DIAG_ALARM = 1 时，插播警报期间第三行显示 ERR:C8（平时不显示任何东西）。
//   用途：一秒钟判断"警报状态机到底有没有起来、有没有撑满 ALARM_HOLD_S"。
// ★ 出正式版必须为 0
localparam       DIAG_ALARM   = 1'b0;
// DIAG_ACK = 1 时，按下解除键后 0.7 秒内第三行显示 ERR:C9（回执）。
//   用途：不用改代码就能现场确认"KEY9 到底接没接在 ACK_MASK 上"。
// ★ 出正式版必须为 0
localparam       DIAG_ACK     = 1'b0;

// tb_busy / tb_done 在 ext_mem_clk(125M) 域，video_clk(25M) 域要两级同步再显示
reg tb_busy_s1, tb_busy_v2;
reg tb_done_s1, tb_done_v2;
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n) begin
        tb_busy_s1 <= 1'b0;
        tb_busy_v2 <= 1'b0;
        tb_done_s1 <= 1'b0;
        tb_done_v2 <= 1'b0;
    end else begin
        tb_busy_s1 <= tb_busy;
        tb_busy_v2 <= tb_busy_s1;
        tb_done_s1 <= tb_done;
        tb_done_v2 <= tb_done_s1;
    end
end

wire [7:0] tb_state_dbg = {2'b00, tb_busy_v2, tb_done_v2, loader_image_complete};

// ---------------------------------------------------------------------------
// ★ 2026-10-31「首次出图后抖一下」的一次性诊断（默认开，出正式版前改 0）
//
// 现象：只有第 1 张显示出来、后台还在加载第 2~4 张的约 1 秒内，画面像被
//       左右拉扯/拉伸；只放一张图则不出现。F1（帧边界出图）+F2（丢首帧）都没治好。
//
// 一次上板就能区分两种机理，不用盲改：
//   0xE0 = 两个都没发生（抖动不是 FIFO 欠载、也不是读请求被冻结挡掉）
//   0xE1 = 读侧 FIFO 欠载（reader_underflow 置过位）
//          → 说明加载写流量把显示读饿死了，要改读写分时（写只在消隐期）
//   0xE2 = 帧头读请求被 read_freeze 挡过（video_read_req 与 freeze 同拍）
//          → 说明冻结解除时机会让 reader 丢掉整帧重装，画面靠"绕回"继续
//   0xE3 = 两者都发生
// 两个标志都是粘滞的（只复位清零），所以抖动过去之后仍能在 OSD 第三行读到。
localparam       DIAG_UFLOW = 1'b0;   // 2026-10-31 23:42 关：诊断已完成（E3），
                                      // 而且现在加载期整段冻结、请求必然被掩，E2 会常亮误导
reg  uflow_seen, req_msk_seen;
wire [7:0] diag_uflow_code;

// 按键实时电平（1 = 按下），用来核对"我按的那颗键到底接在哪根线上"。
// 按住任意一颗：ERR = 0xF0 | 位图。0xF8=KEY12(next) 0xF4=KEY13(auto)
//               0xF2=KEY14(prev) 0xF1=KEY15(mode)
// 松开时 ERR = 0x80 + 确认次数*4 + 焦点  （0x80..0xBF）
wire [3:0] key_lv_w   = {~key_next_n, ~key_auto_n, ~key_prev_n, ~key_spare_n};
wire       key_any_lv = |key_lv_w;

// 构建指纹：上电后 ERR 先显示 DIAG_MARK，生成完成后显示 |0x10，
// 约 2.7 秒后自动隐去（演示时看不到）。DIAG_MARK=0 则完全不显示。
reg [26:0] mark_hold;
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n)        mark_hold <= 27'd0;
    else if (!tb_done_v2)    mark_hold <= 27'd0;
    else if (!mark_hold[26]) mark_hold <= mark_hold + 27'd1;
end
wire       mark_on   = (DIAG_MARK != 8'd0) && !mark_hold[26];
wire [7:0] mark_code = tb_done_v2 ? (DIAG_MARK | 8'h10) : DIAG_MARK;

wire [7:0] err_code_show = (DIAG_ALARM != 1'b0 && alarm_on_v) ? 8'hC8
                         : (DIAG_ACK   != 1'b0 && ack_flash_v) ? 8'hC9
                         : (loader_memory_error != 8'd0)   ? loader_memory_error
                         : (bmp_err_code         != 8'd0)   ? bmp_err_code
                         : (cdc_ovf_seen)                   ? 8'h3A
                         : (scale_ovf_seen)                 ? 8'h39
                         : (diag_uflow_code != 8'd0)        ? diag_uflow_code
                         : (DIAG_THUMB != 1'b0)             ? (key_any_lv
                                                              ? {4'b1111, key_lv_w}
                                                              : {2'b10, dbg_confirm_cnt, focus_sd})
                         : (mark_on)                        ? mark_code
                         : (DIAG_SHOW_TB)                   ? tb_state_dbg
                                                            : 8'd0;

// 真实错误码（不含诊断码）：数码管只认这个，诊断码 C8/C9 只上 OSD，
//   否则插播警报时数码管会跟着显示 "8E"，看着像真出错了。
wire [7:0] err_code_real = (loader_memory_error != 8'd0) ? loader_memory_error
                         : (bmp_err_code         != 8'd0) ? bmp_err_code
                         : (cdc_ovf_seen)                 ? 8'h3A
                         : (scale_ovf_seen)               ? 8'h39
                                                          : 8'd0;

// 出错时图号位熄灭，避免 "08E" 被误读成错误码的一部分（只留 8E = 0x38）
wire [6:0] seg_num_show = (err_code_real != 8'd0) ? 7'b000_0000 : seg_data_0;

// 第5讲 Step3：数码管第1位显示错误码分类数字(1~8)，第2位显示 'E' 表示错误态
// 8位错误码 0x31~0x38 的低4位即分类 1~8；0=无错(两位都不显示)
wire [6:0] seg_err_code;
seg_decoder seg_decoder_err(
    .bin_data          ({1'b0, err_code_real[3:0]}),
    .seg_data          (seg_err_code)
);
// 'E' 的七段码（共阴，a,b,c,d,e,f,g 位序，E = a,d,e,f,g 点亮）
localparam [6:0] SEG_E = 7'b111_1001;
// 无错时第1位也熄灭（避免常显"0"被误读为错误0）
wire [6:0] seg_err_code_show = (err_code_real != 8'd0) ? seg_err_code : 7'b000_0000;
wire [6:0] seg_err_flag = (err_code_real != 8'd0) ? SEG_E : 7'b000_0000;

// 第5讲 Step3：恢复状态机可观测。数码管第3位显示恢复态 0~3，第4位 'r' 标志。
// recover_state: 0=OK 1=RETRY 2=SKIP 3=RESCAN；仅在非 OK 态时点亮（避免常显"0"）。
wire [6:0] seg_recover_code;
seg_decoder seg_decoder_rec(
    .bin_data          ({2'b0, recover_state}),
    .seg_data          (seg_recover_code)
);
localparam [6:0] SEG_R = 7'b101_0000;   // 'r' 七段码（共阴，r = e,g 段点亮）
wire [6:0] seg_recover_show = (recover_state != 2'b00) ? seg_recover_code : 7'b000_0000;
wire [6:0] seg_recover_flag = (recover_state != 2'b00) ? SEG_R : 7'b000_0000;

seg_scan seg_scan_m0(
    .clk               (clk),
    .rst_n             (sys_rst_n),
    .seg_sel           (seg_sel),
    .seg_data          (seg_data),
    .seg_data_0        ({1'b0,seg_num_show}),
    .seg_data_1        ({1'b0,seg_err_code_show}),
    .seg_data_2        ({1'b0,seg_err_flag}),
    .seg_data_3        ({1'b0,seg_recover_show}),
    .seg_data_4        ({1'b0,seg_recover_flag}),
    .seg_data_5        (8'h00)
);

// ===================== 原图像时序与帧缓存 =====================
video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (~video_rst_n),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .h_cnt_out         (h_cnt_0)
);

// =============================================================================
// 第8讲阶段B：读侧改由 media_frame_reader（SDRAM 读 burst + 异步 FIFO + 两级预取）
// + rgb_video_adapter（整组寄存 1 拍 + 消隐清零）接管，删除原 video_delay 的
// 21 拍移位。pixel_rgb 由 prefetch 提前 1 拍顶出，与 de_0 同拍对齐；
// adapter 再把 hs_0/vs_0/de_0/rgb 整组寄存 1 拍输出 hs/vs/de/vout_data_raw，
// 三者严格同拍，video_x 也对应打 1 拍对齐。
// =============================================================================
wire [23:0] reader_pixel_rgb;
wire        reader_pixel_valid;
wire        reader_underflow;

// =============================================================================
// ★★★ 第13讲 噪点根因修复：加载期间冻结读侧（2026-10-28）
//
// 【实测事实链】
//   1. 四张诊断图**逐字节完全相同**，但每张出噪点的位置不一样
//      ⇒ 像素流内容相同而结果不同 ⇒ 错的**不是显示通路**，是 SDRAM 里存的内容。
//   2. IMG:1 永远干净，IMG:2/3/4 一张比一张花。
//      区别只有一个：IMG:1 写入时 display_valid 还是 0，**读侧没在跑**；
//      IMG:2/3/4 写入时画面已经在播，**读 burst 和写 burst 在抢同一条 SDRAM 总线**。
//   3. 把 SPI 从 16.67MHz 降到 7.14MHz（加载时间 ×2.3）噪点反而更密
//      ⇒ 错误总量正比于"加载窗口有多长" ⇒ 每个单位时间都在丢，且与 SD 无关。
//
// 【机理】
//   写侧 dst_app_wr_en = fifo_rd_valid，是**单拍脉冲**，而且 SDRAM 控制器
//   没有任何"我收下了"的应答。一旦这一拍恰好落在控制器不接命令的时刻
//   （读 burst 还在收尾 / 内部刷新窗口 / 正在换行），这条写命令就被**静默丢弃**：
//     · 地址是随数据一起打包进 FIFO 的（dst_app_wr_addr 取自 fifo_rd_data），
//       所以丢一条只会在原地留一个"洞"，后面的像素不会错位
//       —— 这正是"棋盘格和数字都完整、只有灰底散落彩点"的原因；
//     · 洞里留着的是这块地址上一次的内容（未初始化 / 上一张图），
//       所以颜色随机、位置随机、每加载一次重掷一次；
//     · 没有任何计数器看得见这条丢命令 ⇒ 永远零错误码。
//
// 【修法】
//   加载期间把读侧请求整体按住（loader_load_busy 从 load_start 一直保持到整帧写完），
//   让写 burst 独占总线。代价是加载那 0.44s 屏幕是黑的 —— 这正是普通播放器
//   "换图时闪一下"的行为，演示完全可接受，而且比"边写边读撕裂 + 彩点"好得多。
//
//   为什么不在写侧加 Sdr_busy 门控就够：那只能挡住"控制器忙"这一类丢命令，
//   挡不住"读 burst 正在流水、控制器暂时不收新命令"这一类；而且改 Sdr_busy
//   门控会改变写通路时序，属于另一项改动。先做这个单变量实验验证机理。
// =============================================================================
// 【2026-10-31 B+A 方案】把"加载期整段冻结"改成"按 bank 冻结 + 解锁缓冲"
// -----------------------------------------------------------------------------
// 老做法（10-28 的单变量实验，一直留到 w26 正式版）：
//     load_freeze = load_busy;               // 四张图连着加载 ⇒ 0.44×4 ≈ 1.6s 全黑
//     vout_data   = (display_valid && !load_freeze) ? raw : 0;
// 三个后果（实机 2026-10-31 量出来）：
//     ① 首图其实早就写完了，可屏幕一直黑到第四张写完（实测 2.60s 才出图）；
//     ② 解锁那一刻 vout_data 从 0 硬切到 raw，而读侧刚重新拉起 read_req 会清 FIFO，
//        预取还没填满 ⇒ 头几十个像素是陈旧值/未初始化内容 = "黑屏闪花一下"；
//     ③ 加载中途 OSD/字幕/波形照常显示，视觉上"字先出来、图后出来"。
// 新做法：
//     bank_ok_v : 当前显示的那个 bank 是否已有完整一帧（bank_valid 同步过来）
//     read_freeze = load_busy && (!bank_ok_v || 正在写的就是正在看的那一 bank)
//         —— 只在"要看的那一 bank 空着"或"正在写它"时冻结读侧；
//            加载 bank1..3 时 bank0 已经在显示，读写并发由 reader 的
//            app_rd_busy / ext_wr_busy 握手 + CDC 的 rd_quiet 门控负责（第 8 讲机制）。
//     vid_ready : bank_ok 之后再过 REL_BLANK_CYC 拍才解锁（≈2ms@25MHz），
//         —— 给 frame_fifo_read 足够时间重新预取，彻底消掉解锁瞬间的彩点。
//     缩略图页例外：它的内容不是 bank 里加载出来的图，用 tb_done（生成本完成）做判据。
// =============================================================================
reg load_busy_v1, load_busy_v2;
reg [3:0] bv_s1, bv_s2;             // bank_valid 两级同步
reg [1:0] lb_s1, lb_s2;             // load_bank  两级同步
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n) begin
        load_busy_v1 <= 1'b0;
        load_busy_v2 <= 1'b0;
        bv_s1        <= 4'b0000;
        bv_s2        <= 4'b0000;
        lb_s1        <= 2'd0;
        lb_s2        <= 2'd0;
    end else begin
        load_busy_v1 <= loader_load_busy;   // 来自 sd_card_clk 域，两级同步
        load_busy_v2 <= load_busy_v1;
        bv_s1        <= bmp_bank_valid;
        bv_s2        <= bv_s1;
        lb_s1        <= bmp_load_bank;
        lb_s2        <= lb_s1;
    end
end

// 当前真正在显示的 bank（与 reader 例化的 read_addr_index 用同一个表达式，别分叉）
//   DIAG_READ_BANK0：噪点排查实验用的"读侧钉死 bank0"，正式版为 0
wire [1:0] rd_bank_idx = DIAG_READ_BANK0 ? 2'd0 :
                         (DIAG_SINGLE_BUF ? 2'd0 : disp_buf_idx_q);

wire       bank_ok_v   = bv_s2[rd_bank_idx];              // 这一 bank 有完整一帧
wire       load_same_v = load_busy_v2 && (lb_s2 == rd_bank_idx);  // 正在写这一 bank
// ★ 回退开关：1 = 整个"四张加载期"都冻结读侧（w26 的稳定显示时序）
//   0 = 按 bank 冻结（首图一写完就能显示，出图约 1.2s；用户 2026-10-31 22:03 验证过的那版行为）
//   ★ 2026-11-01 00:2x：定回 0。用户实测 1 的版本"开机慢、图音不同步"，已否决。
localparam LOAD_FREEZE_ALL = 1'b0;

// ★ 2026-10-31 23:40：冻结判据从 load_busy（**按张**置位）改成"四张没全加载完"。
//   load_busy 在每张之间会有空档，空档里显示会瞬间解锁，正好露出"半张已写、半张还是
//   旧内容"的 bank —— 这就是 w26 之后残留抖动/花屏的来源（图片看着变大变花同理）。
//   3 秒超时兜底：卡里图不足 4 张（或读卡出错）时不能永远冻结。
reg  [31:0] load_phase_cnt;
wire        load_phase_all = (loader_image_complete != 4'b1111);
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n)                       load_phase_cnt <= 32'd0;
    else if (!load_phase_all)               load_phase_cnt <= 32'd0;
    else if (load_phase_cnt != 32'd75_000_000) load_phase_cnt <= load_phase_cnt + 32'd1;
end
wire       load_phase_to = (load_phase_cnt == 32'd75_000_000);   // 3s @25MHz

wire       read_freeze = LOAD_FREEZE_ALL
                       ? (load_phase_all && !load_phase_to)
                       : (load_busy_v2 && (!bank_ok_v || load_same_v));

// ★ A 方案的关键：读请求只能在**帧边界**放行，否则 reader 会从第 0 行开始读，
//   而显示已经走到第 N 行 ⇒ 那一帧上下错位（老版本"解锁瞬间闪一下"的另一半原因）。
//   read_req 在 vs 沿置 1、被 ack 后清零；冻结期间一直是 1，所以用"vs 后的一小段窗口"
//   与之相与，就能保证解锁一定落在帧头。
reg  vs_0_d;
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n) vs_0_d <= 1'b1;
    else              vs_0_d <= vs_0;
end
wire vs_edge_f = vs_0_d & ~vs_0;      // 与 video_timing_data 判帧头的判据完全一致
reg [10:0] fs_win;
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n)                        fs_win <= 11'h7FF;
    else if (vs_edge_f)                      fs_win <= 11'd0;
    else if (fs_win != 11'h7FF)              fs_win <= fs_win + 11'd1;
end
wire at_frame_start = (fs_win != 11'h7FF);   // vs 之后约 2048 拍（82us@25MHz）内有效

// ★ 2026-10-31 22:59：不再与 at_frame_start 相与。
//   read_req 是"电平 + 等 ack"的，被窗口掩掉之后虽然还是 1，但没有新的上升沿，
//   reader 的取帧状态机可能就再也等不到一次请求 ⇒ 整屏黑（本版 ERR:46 黑屏的另一半原因）。
//   帧边界对齐交给下面的 F1（img_on_v 只在 vs 沿切换）。
// ★ 2026-10-31 23:25：请求通路上彻底不再做任何掩码。
//   read_req 是"vs 沿置位、等 ack 清零"的电平；只要被掩掉一次，它就不再产生新的
//   上升沿，reader 的取帧状态机等不到请求 ⇒ 之后几帧的 FIFO 与屏幕行号错开，
//   表现就是"图片先左移一下、再右移一下、还带拉伸"（ERR:E2 就是抓到过这种被掩的请求）。
//   想看的那一 bank 有没有内容、能不能显示，统一交给下面的 img_on_v / vid_ready 判断。
// ★ 2026-11-01 00:3x：把 at_frame_start 掩码**加回来**，恢复 2026-10-31 22:03 用户验证过
//   的那版行为（当时出图 1.17 s、图/字幕/音频同帧）。read_req 是"vs 沿置位、等 ack 清零"的
//   电平，与"vs 后 82us 窗口"相与 ⇒ 解锁一定落在帧头。若上板再出现黑屏/等不到请求，
//   把本行的 `& at_frame_start` 去掉即可（F1 的 img_on_v 已经能保证帧边界切换）。
wire       video_read_req_g = video_read_req & ~read_freeze & at_frame_start;

// 解锁缓冲：bank 一旦 ok 就从头计时，连续 REL_BLANK_CYC 拍才允许出图
localparam [17:0] REL_BLANK_CYC = 18'd50_000;   // 2ms @ 25MHz
reg [17:0] rel_cnt;
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n)                    rel_cnt <= 18'd0;
    else if (!bank_ok_v)                 rel_cnt <= 18'd0;
    else if (rel_cnt != REL_BLANK_CYC)   rel_cnt <= rel_cnt + 18'd1;
end
wire vid_ready = bank_ok_v && (rel_cnt >= REL_BLANK_CYC);

// ★ F1：出图使能只在 vs 边沿切换，永不出现"上半黑下半有图"的半帧
//   原来是组合出来的（vid_ready 直接进 img_ok_v），解锁时刻落在帧中间时，
//   那一帧就只显示下部 ⇒ 人眼看到"图片出现的瞬间抖一下"。
// ★ F2：解锁后再多等一个 vs 才允许出图（丢首帧）。
//   理由：read_req 的上升沿会清 frame_fifo_read 的 FIFO 重新预取，
//   首帧顶部若干行可能还是陈旧/重复像素。丢一帧只多黑 16.7ms（肉眼几乎无感），
//   但能保证第一帧可见画面是从第 0 行开始、数据已经就位的完整帧。
reg        img_on_v;
reg        rd_seen;
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n) begin
        img_on_v <= 1'b0;
        rd_seen  <= 1'b0;
    end else if (!vid_ready) begin
        img_on_v <= 1'b0;      // bank 失效（重扫/换图）立即熄屏，不显示半帧垃圾
        rd_seen  <= 1'b0;
    end else if (vs_edge_f) begin
        img_on_v <= rd_seen;   // 第一个 vs 时 rd_seen 还是 0 ⇒ 该帧仍黑
        rd_seen  <= 1'b1;      // 从第二个 vs 起才真正出图
    end
end

// ★ 一次性诊断的采样与编码（读侧两个"粘滞标志"，见前面 DIAG_UFLOW 的说明）
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n) begin
        uflow_seen   <= 1'b0;
        req_msk_seen <= 1'b0;
    end else begin
        if (reader_underflow)              uflow_seen   <= 1'b1;
        if (video_read_req && read_freeze) req_msk_seen <= 1'b1;
    end
end
assign diag_uflow_code =
    (DIAG_UFLOW && (uflow_seen || req_msk_seen))
        ? (8'hE0 | {6'b000000, uflow_seen, req_msk_seen}) : 8'd0;

// ★ 第15讲：读基址 mux —— 缩略图视图下四个 bank 全部指向缩略图页
//   （read_addr_index 照旧随 disp_buf_idx_q 变，但四路同址，等于无效化）
//   注意：缩略图页生成引擎**不能**用这四个信号当源图基址（否则会读自己正在写的页），
//   它走 reader 的 src_addr_0..3 专用端口，恒接 BUF0..3。
assign rd_base_0 = thumb_view_v ? THUMB_ADDR[ADDR_BITS-1:0] : BUF0_ADDR[ADDR_BITS-1:0];
assign rd_base_1 = thumb_view_v ? THUMB_ADDR[ADDR_BITS-1:0] : BUF1_ADDR[ADDR_BITS-1:0];
assign rd_base_2 = thumb_view_v ? THUMB_ADDR[ADDR_BITS-1:0] : BUF2_ADDR[ADDR_BITS-1:0];
assign rd_base_3 = thumb_view_v ? THUMB_ADDR[ADDR_BITS-1:0] : BUF3_ADDR[ADDR_BITS-1:0];

media_frame_reader #(
    .MEM_DATA_BITS   (MEM_DATA_BITS),
    .ADDR_BITS       (ADDR_BITS),
    // ★ 第15讲：擦洗范围扩到 5 帧（含缩略图页 1228800..1535999），
    //   否则页内容会重演 10-28 的"高地址衰减"（实机验证过：噪点随时间增多）
    .SCRUB_WORDS     (SCRUB_WORDS_CFG[ADDR_BITS-1:0]),
    // ---- 第15讲：2×2 缩略图页生成引擎 ----
    .TB_EN           (1),
    .TB_TEST_EN      (0),
    .TB_PASSES       (2),
    .TB_ADDR         (THUMB_ADDR[ADDR_BITS-1:0]),
    .TB_SRC_W        (640),
    .TB_W            (320),
    .TB_H            (240)
) u_media_frame_reader (
    .mem_clk          (ext_mem_clk),
    .rst              (~mem_rst_n),
    .Sdr_init_done    (Sdr_init_done),
    .Sdr_init_ref_vld (Sdr_init_ref_vld),
    .Sdr_busy         (Sdr_busy),
    .Sdr_rd_en        (Sdr_rd_en),
    .Sdr_rd_dout      (Sdr_rd_dout),
    .App_rd_en        (App_rd_en),
    .App_rd_addr      (App_rd_addr),

    // 读写互斥握手：读侧靠 ext_wr_busy 避让写 burst，app_rd_busy 送回 CDC
    .ext_wr_busy      (wr_burst_busy),
    // ★ 2026-10-31：预取避让改用"真正在写"的窄窗口（wr_burst_active）
    .ext_wr_active    (wr_burst_active),
    .app_rd_busy      (rd_burst_busy),
    // ★ 2026-10-31：把消隐期标志送回去给 CDC 做"写只在消隐期"的分时
    .display_blank    (reader_display_blank),

    .pix_clk          (video_clk),
    .pix_rst          (~video_rst_n),
    .read_req         (video_read_req_g),
    .read_req_ack     (video_read_req_ack),
    .read_addr_0      (rd_base_0),
    .read_addr_1      (rd_base_1),
    .read_addr_2      (rd_base_2),
    .read_addr_3      (rd_base_3),
    .read_addr_index  (rd_bank_idx),
    .read_len         (FRAME_PIXELS),
    // ★ 2026-10-28 已回滚：曾试过 .pixel_consume(de_0 & ~load_freeze)，
    //   想让"加载期间零读活动"。结果整屏变纯灰（实机验证）。
    //   原因：pixel_consume 是读侧唯一的节流信号。停掉消费 → FIFO 填满 →
    //   frame_fifo_read 靠 wrusedw 判断发不发 burst → 一个 burst 都发不出 →
    //   read_cnt 卡住，而 de 还在走 → 读侧与显示位置永久错位；
    //   load_freeze 结束后 read_req 重新拉起又会 fifo_aclr 重置 FIFO，
    //   帧内容从头开始、显示已在中途 → 无法恢复。
    //   正确做法：若真要冻结读侧，必须同时冻结显示侧的帧同步（de/vs），
    //   不能只掐消费。留作后续方案，不在第13讲硬做。
    .pixel_consume    (de_0),
    .pixel_rgb        (reader_pixel_rgb),
    .pixel_valid      (reader_pixel_valid),
    .underflow        (reader_underflow),

    // ---- 第15讲：缩略图页生成引擎 ----
    .tb_image_complete(loader_image_complete),
    .src_addr_0       (BUF0_ADDR[ADDR_BITS-1:0]),
    .src_addr_1       (BUF1_ADDR[ADDR_BITS-1:0]),
    .src_addr_2       (BUF2_ADDR[ADDR_BITS-1:0]),
    .src_addr_3       (BUF3_ADDR[ADDR_BITS-1:0]),
    .tb_wr_en         (tb_wr_en),
    .tb_wr_addr       (tb_wr_addr),
    .tb_wr_data       (tb_wr_data),
    .tb_busy          (tb_busy),
    .tb_done          (tb_done)
);

// 整组寄存 1 拍：hs_0/vs_0/de_0/reader_pixel_rgb 同拍进入，同拍输出 hs/vs/de/vout_data_raw
rgb_video_adapter u_rgb_video_adapter (
    .pclk             (video_clk),
    .rst_n            (video_rst_n),
    .hs_in            (hs_0),
    .vs_in            (vs_0),
    .de_in            (de_0),
    .rgb_in           (reader_pixel_rgb),
    .video_clk        (),
    .video_hs         (hs),
    .video_vs         (vs),
    .video_de         (de),
    .video_rgb        (vout_data_raw)
);

// =============================================================================
// 第10讲：图片转场（淡出 → 全黑 → 换 bank → 淡入）
//
//   为什么不需要讲义那套跨时钟域握手：
//   四 bank 架构下 sd_card_bmp 切图只是改 disp_buf_idx（零延迟），
//   而 frame_fifo_read 只在每帧 S_ACK 锁存一次 bank 基址，帧内只做地址递增
//   —— "切哪个 bank"本来就只在帧边界生效。
//   所以只要把送给 reader 的索引换成 disp_buf_idx_q，等全黑时再更新它即可，
//   整个转场是单时钟域（video_clk），无 toggle 握手、无 commit 回传。
//
//   enable 接 display_valid：无图（首图未提交 / 拔卡）时不介入，
//   与第 9 讲的上电行为完全一致。
// =============================================================================
media_fade_transition #(
    .FADE_FRAMES (FADE_FRAMES_CFG),   // 见文件头 ①②③ 三项说明
    .MIN_ALPHA   (MIN_ALPHA_CFG),     // 最糊档保留的原图权重（16 ≈ 纯灰）
    .HOLD_FRAMES (1)                  // 最糊档多停 1 帧，确保读侧已锁到新 bank
) u_media_fade_transition (
    .clk            (video_clk),
    .rst            (~video_rst_n),
    .vs             (vs),
    .disp_buf_idx   (disp_buf_idx),
    .enable         (display_valid),
    .disp_buf_idx_q (disp_buf_idx_q),
    .alpha          (fade_alpha),
    .fade_busy      (fade_busy)
);

// video_x 对齐到 adapter 输出：h_cnt_0 打 1 拍，与 de(=video_de) 严格同拍
reg [11:0] video_x_r;
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n)
        video_x_r <= 12'd0;
    else
        video_x_r <= h_cnt_0;
end
assign video_x = video_x_r;

// 图像通道出图条件（2026-10-31 B+A + F1/F2，23:25 再改）：
//   display_valid          已显示过至少一张
//   img_on_v               vid_ready（bank 有效后再等 ~2ms 让 FIFO 预取）在**vs 边沿**锁存，
//                          并丢掉解锁后的第一帧（F1/F2：杜绝半帧）
//   load_same_v            正在写的就是正在看的那个 bank 时才熄屏（唯一的同 bank 保护）
//   thumb_view_v           缩略图页：内容由生成引擎写出，判据换成 tb_done（生成完成）
wire img_ok_v = thumb_view_v ? tb_done_v2
                             : (display_valid && !read_freeze && img_on_v);
assign vout_data = img_ok_v ? vout_data_raw : 24'd0;

// =============================================================================
// 第14讲：亮度 / 对比度调节（插在 OSD **之前**）
//   为什么放在 OSD 之前：调暗画面时，状态文字和滚动字幕仍是纯白，
//   不会跟着一起变暗 —— 参数回显和状态提示在任何亮度下都可读。
//   本模块打 2 拍，de/vs/video_x 同步延迟 2 拍，下游 OSD 一律用 *_adj 信号，
//   保证叠加坐标与像素严格同拍（否则文字会整体右移 2 像素）。
// =============================================================================
wire [23:0] vout_data_adj;
wire [9:0]  video_x_adj;
wire        de_adj, vs_adj;
wire [7:0]  bright_eff, contrast_eff;

brightness_contrast_adjust u_bc_adjust (
    .clk           (video_clk),
    .rst_n         (video_rst_n),
    .de            (de),
    .vs            (vs),
    .video_x       (video_x_ref),
    .rgb_in        (vout_data),
    .brightness    (adj_bright),
    .contrast      (adj_contrast),
    .out_de        (de_adj),
    .out_vs        (vs_adj),
    .out_x         (video_x_adj),
    .rgb_out       (vout_data_adj),
    .bright_eff    (bright_eff),
    .contrast_eff  (contrast_eff)
);

// =============================================================================
// 第15讲：把 sd 域的两个慢变量（view / focus）搬进 video 域，并在**帧边界**锁存
//   为什么必须锁存：读基址 mux 一旦帧内变化，就会"上半帧旧图 / 下半帧新图"撕裂；
//   焦点框同理。两者都只在 vs 下降沿更新，与帧起始严格对齐。
//   注意 vs 在本视频通路里是**低有效**（高电平覆盖整个有效画面，见
//   rgb_video_adapter 的 VS_OUT_ACTIVE_HIGH 与上游 vs_0 的叠加）⇒ 帧边界取
//   下降沿，与 osd_overlay / audio_waveform_overlay 的判据完全一致。
//   （第15讲第一版就是在这里把判据写成 `if (vs)`，导致行计数被死按在 0，
//    整个左半屏被焦点框涂成青色 —— 这个坑已记进工程状态文档第二十二章。）
// =============================================================================
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n) begin
        thumb_view_s1 <= 1'b0;
        thumb_view_s2 <= 1'b0;
        focus_s1      <= 2'd0;
        focus_s2      <= 2'd0;
        vs_adj_d2     <= 1'b0;
        thumb_view_v  <= 1'b0;
        thumb_focus_v <= 2'd0;
    end else begin
        thumb_view_s1 <= thumb_view_sd;
        thumb_view_s2 <= thumb_view_s1;
        focus_s1      <= focus_sd;
        focus_s2      <= focus_s1;
        vs_adj_d2     <= vs_adj;
        if (vs_adj_d2 && !vs_adj) begin
            thumb_view_v  <= thumb_view_s2;
            thumb_focus_v <= focus_s2;
        end
    end
end

// =============================================================================
// 第10讲：转场 alpha 乘法（插在 亮度对比度 之后、OSD 之前）
//   ① 放在亮度/对比度之后：对比度<128 时会把黑色抬成灰（y=(x-128)*c/128+128），
//      若 alpha 在它之前，淡出到"黑"其实是灰的。放在后面才是真黑。
//   ② 放在 OSD/字幕之前：转场时状态文字和滚动字幕**不跟着变暗**，
//      全程可读 —— 这是刻意的，转场属于画面层，UI 层不参与。
//   ③ 纯组合、零延迟：de_adj/vs_adj/video_x_adj 保持原样，OSD 坐标不用改。
// =============================================================================
wire [23:0] vout_data_fade;

fade_alpha_apply #(
    .FADE_GRAY (FADE_GRAY_CFG)        // 转场最糊时的目标灰阶
) u_fade_alpha_apply (
    .alpha   (fade_alpha),
    .rgb_in  (vout_data_adj),
    .rgb_out (vout_data_fade)
);

// ===================== OSD 屏幕叠加（第5讲 Step2）=====================
// 在 vout_data 上叠加三行状态文字。阶段B后 vout_data 与 hs/vs/de/video_x 全部由
// rgb_video_adapter 整组寄存 1 拍输出，四路严格同拍，OSD 直接用 de/vs/video_x_ref
// 与 vout_data 同域即可，行计数由 de 下降沿生成。
wire [23:0] vout_data_osd;
osd_overlay u_osd_overlay (
    .clk           (video_clk),
    .rst           (~video_rst_n),
    .de            (de_adj),
    .vs            (vs_adj),
    .video_x       (video_x_adj),
    .video_in      (vout_data_fade),
    .sd_init_done  (sd_init_done),
    .img_idx       (media_id),
    .img_total     (img_total),
    .bmp_err_code  (err_code_show),   // 第8讲：优先显示 loader 的真实错误码
    .param_bright  (adj_bright),      // 第14讲：亮度回显
    .param_contrast(adj_contrast),    // 第14讲：对比度回显
    .param_show    (adj_show),        // 第14讲：调节后 3 秒内显示
    // 第11讲阶段C：音频状态四态进 OSD（行5）
    //   wav_audio_found 来自 sd_card_clk(100MHz) 域的慢变化电平，OSD 内部两级同步；
    //   wav_active / wav_underrun 本来就在 video_clk 域，多打两拍无影响
    .audio_found   (wav_audio_found),
    .audio_active  (wav_active),
    .audio_underrun(wav_underrun),
    .audio_pos_sec (wav_pos_sec),
    .audio_tot_sec (wav_tot_sec),
    .video_out     (vout_data_osd)
);

// ===================== 音频波形叠加（第 12 讲）=====================
// 图层顺序变成：BMP 图片 → 状态 OSD(左上六行) → 音频波形面板 → 滚动字幕 → HDMI
//   ★ 波形模块与像素时钟同域：wav_stream_player 本来就跑在 video_clk，
//     所以 pcm 样本零跨域，不需要任何异步 FIFO / 握手。
//   ★ pcm_valid 取的是「一个样本被消费」的单拍脉冲（sample_valid && sample_ready），
//     不能直接接 sample_valid —— 那是 out_count != 0 的电平，队列积压时
//     会连续多拍为高，同一个样本会被重复计入采集。
//   ★ enable 用 wav_audio_found（卡上扫到 MUSIC.PCM 才显示面板）。
//     它在 sd_card_clk(100MHz) 域，这里做两级同步；与 sd_init_done 相与，
//     拔卡后面板自动收起，和 OSD 行5 的 AU:NO FILE 保持一致。
reg wav_found_s0, wav_found_s1;
reg sd_ok_s0,     sd_ok_s1;
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n) begin
        wav_found_s0 <= 1'b0;
        wav_found_s1 <= 1'b0;
        sd_ok_s0     <= 1'b0;
        sd_ok_s1     <= 1'b0;
    end else begin
        wav_found_s0 <= wav_audio_found;
        wav_found_s1 <= wav_found_s0;
        sd_ok_s0     <= sd_init_done;
        sd_ok_s1     <= sd_ok_s0;
    end
end

wire wav_sample_pop = wav_sample_valid && wav_sample_ready;   // 真正消费掉一个样本
// 第16讲：警报期间把警报样本也喂进波形面板，否则插播时波形会"冻住不动"
wire alarm_sample_pop = alarm_step;
wire wave_pcm_valid   = alarm_active_v ? alarm_sample_pop : wav_sample_pop;
wire signed [15:0] wave_pcm_sample = alarm_active_v ? alarm_left : wav_sample_left;

wire [23:0] vout_data_wave;

audio_waveform_overlay #(
    .WAVE_X       (64),        // 面板左边界（右侧留 64 给峰值条与边距）
    .WAVE_Y       (320),       // 面板上边界：OSD 六行到 114 结束，字幕带 436 起，
    .WAVE_WIDTH   (512),       //   320+64=384，上下都不打架
    .WAVE_HEIGHT  (64),
    // ★ 256 点 × 抽取 2 = 512 样本/帧，一帧只到 800 个，余量 1.56 倍；
    //   抽取率不能放大到 8 —— 那会把显示采样率压到 6kHz，3200Hz 素材直接混叠成假低频。
    .DECIMATE     (2),
    .SAMPLE_COUNT (256)        // 每点 2 像素
) u_audio_waveform (
    .clk        (video_clk),
    .rst        (~video_rst_n),
    .de         (de_adj),
    .vs         (vs_adj),
    .video_x    (video_x_adj),
    .video_in   (vout_data_osd),
    .enable     (wav_found_s1 & sd_ok_s1),
    .pcm_valid  (wave_pcm_valid),
    .pcm_sample (wave_pcm_sample),
    .video_out  (vout_data_wave),
    .overlay_active ()
);

// ===================== 滚动字幕叠加（第9讲 OSD 字幕叠加）=====================
// 图层顺序：BMP 图片 → 状态 OSD(左上六行) → 音频波形面板 → 滚动字幕(底部) → HDMI
//   5x7 点阵 2 倍放大、每帧左移 2 像素平滑滚动、半透明黑底带（移位近似 Alpha）
//   多条字幕自动循环；display_valid 作为总开关，首图提交前不显示字幕。
wire [23:0] vout_data_sub;

subtitle_scroll_overlay #(
    .H_ACTIVE    (640),
    .BAND_Y      (436),       // 原 452 贴底太近，上移 16 行留安全区
    .SCROLL_STEP (1)          // 原 2（120 px/s）偏快，改 1（60 px/s）
) u_subtitle_scroll (
    .clk            (video_clk),
    .rst            (~video_rst_n),
    .de             (de_adj),
    .vs             (vs_adj),
    .video_x        (video_x_adj),
    .video_in       (vout_data_wave),
    .enable         (display_valid),
    .video_out      (vout_data_sub),
    .overlay_active ()
);

// ===================== 第15讲：2×2 缩略图页焦点框（最上层）=====================
//   图层顺序变成：BMP 图片 → OSD → 波形 → 字幕 → **缩略图十字隔条+焦点角标** → HDMI
//   放在最上层是刻意的：缩略图页是个"选择界面"，隔条与焦点标记必须压过所有叠加层，
//   否则字幕带会横穿十字隔条、左上 OSD 会盖住焦点角标的左上臂。
//   enable = thumb_view_v（帧边界锁存），非缩略图视图下本模块零影响。
wire [23:0] vout_data_brd;
wire        brd_over;

thumb_focus_border #(
    .DW (640), .DH (480), .HW (320), .HH (240),
    .CW (52),  .BT (8),  .OL (2)          // 明黄 + 2px 深灰描边；臂长 52 / 臂宽 8（2026-11-01 定稿）
) u_thumb_focus_border (
    .clk      (video_clk),
    .rst      (~video_rst_n),
    .de       (de_adj),
    .video_x  (video_x_adj),
    .enable   (thumb_view_v),
    .focus    (thumb_focus_v),
    .rgb_in   (vout_data_sub),
    .rgb_out  (vout_data_brd),
    .over     (brd_over)
);

// ===================== UART 串口日志（第5讲 Step2）=====================
// 运行在 sd_card_clk(100MHz) 域，115200 波特率，TX 输出到 CH340N(C13)。
// 事件：SD初始化/拔卡/切图/加载错误。
uart_logger #(
    .CLK_FREQ_HZ (100_000_000),
    .BAUD_RATE   (115200)
) u_uart_logger (
    .clk          (sd_card_clk),
    .rst          (~sdcard_rst_n),
    .sd_init_done (sd_init_done),
    .img_idx      (media_id),
    .img_total    (img_total),
    .load_error   (load_error),
    .ld3s_pulse   (ld3s_pulse),
    .ld3s_src     (ld3s_src),
    .fat_dbg_pulse(fat_dbg_pulse),
    .fat_dbg_ok   (fat_dbg_ok),
    .fat_dbg_val  (fat_dbg_val),
    .scan_diag_pulse(scan_diag_pulse),
    .scan_diag_zero (scan_diag_zero),
    .scan_diag_val  (scan_diag_val),
    .bmp_err_code (err_code_real),   // 串口只记真实错误码，不记诊断码 C8/C9
    .uart_tx      (uart_tx)
);

// =============================================================================
// 第8讲阶段A：SDRAM 写端口改由 CDC 驱动（原来由 frame_read_write 驱动）
// 阶段B：读侧已由 media_frame_reader 接管，frame_read_write 实例已彻底移除。
// =============================================================================
// ★ 诊断开关 DIAG_FREEZE_WRITE=1 时写使能恒 0：
//   写地址/数据总线照常由 CDC 驱动（不改变时序关系），只是不再产生写命令，
//   这样"SPI + 解码 + 计数"的活动强度与正常加载完全一致，只去掉 SDRAM 写这一件事。
assign App_wr_en   = DIAG_FREEZE_WRITE ? 1'b0 : (tb_wr_en ? 1'b1 : cdc_app_wr_en);
assign App_wr_addr = tb_wr_en ? tb_wr_addr : cdc_app_wr_addr;
assign App_wr_din  = tb_wr_en ? tb_wr_data : cdc_app_wr_data;
// DM 高电平 = 屏蔽该字节；缩略图页要整字写，用 4'b0000
assign App_wr_dm   = tb_wr_en ? 4'b0000 : cdc_app_wr_mask;

sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (~mem_rst_n),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

// ===================== 第 11 讲阶段 B：WAV 音频播放器 =====================
// 跨两个时钟域：
//   ▸ sd 侧（sd_card_clk 100MHz）：接 sd_card_bmp 的音频文件流（写异步 FIFO）
//   ▸ 播放侧（video_clk 25MHz）：48kHz 节拍、音量、valid/ready 样本输出
// 播放侧输出直接进音频桥的 mux；wav_active=1 时 mux 选 WAV，否则回落琶音。
// （这些 wire 在上方"音频桥接输入"处统一声明，这里只做例化）
wav_stream_player #(
    .CLOCK_HZ     (25_000_000),        // video_clk
    .SAMPLE_HZ    (48_000),
    .FIFO_AW      (10),                // 1024 字 = 2048 字节 ≈ 21ms 缓冲
    .STALL_CYCLES (25_000_000),        // 断流 1s 才放弃（回落琶音）
    .REFILL_WORDS (128)                // 攒到 128 字（≈2.7ms）才开声
) u_wav_player (
    .sd_clk          (sd_card_clk),
    .sd_rst_n        (sdcard_rst_n),
    .sd_play_en      (wav_play_en),
    .sd_fifo_wr_en   (wav_fifo_wr_en),
    .sd_fifo_wr_data (wav_fifo_wr_data),
    .sd_fifo_room    (wav_fifo_room),

    .pix_clk         (video_clk),
    .pix_rst_n       (video_rst_n),
    .volume          (volume),
    .sample_valid    (wav_sample_valid),
    .sample_ready    (wav_sample_ready),
    .sample_left     (wav_sample_left),
    .sample_right    (wav_sample_right),
    .active          (wav_active),
    .underrun        (wav_underrun),
    .level_dbg       (wav_level)
);

// ---- 第16讲：警报电平跨到 video 域（sd 域 100MHz → video 域 25MHz，慢变电平两级同步）----
reg alarm_s0, alarm_s1;
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n) begin
        alarm_s0 <= 1'b0;
        alarm_s1 <= 1'b0;
    end else begin
        alarm_s0 <= alarm_en_sd;
        alarm_s1 <= alarm_s0;
    end
end
assign alarm_on_v = alarm_s1;

// ---- 第16讲：解除键回执电平跨到 video 域（同样两级同步）----
reg ack_s0, ack_s1;
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n) begin
        ack_s0 <= 1'b0;
        ack_s1 <= 1'b0;
    end else begin
        ack_s0 <= (ack_flash != 27'd0);
        ack_s1 <= ack_s0;
    end
end
assign ack_flash_v = ack_s1;

// FPGA 内合成的双音警笛（2026-10-31 起已不是"双短鸣"）：1000Hz 与 750Hz 方波
// 每 0.35s 交替一次、无静音段，持续到 alarm_en_sd 掉下去（KEY9 解除 / ALARM_MAX_S 兜底）。
// 节拍由 sink 握手回送的 alarm_step 驱动（不自己产生 48kHz tick），详见 §23.12。
// 不占卡上文件、不吃 LUT（约 40 个触发器）。
alarm_tone_gen #(
    .CLOCK_HZ  (25_000_000),
    .SAMPLE_HZ (48_000)
) u_alarm_tone (
    .clk            (video_clk),
    .rst_n          (video_rst_n),
    .run            (alarm_on_v),
    .step           (alarm_step),
    .sample_left    (alarm_left),
    .sample_right   (alarm_right),
    .active         (alarm_active_v)
);

// ===================== HDMI 音频桥接（替换加密 HDMI IP） =====================
// 参考方案：像素时钟驱动，pcm_media_tone 生成音调，内部完成音频包封装 + TMDS 编码
// Step3：media_id 接图片编号 img_idx，切图换音调（哔提示音 + 琶音循环）
media_hdmi_audio_bridge u_media_hdmi_audio_bridge (
    .pixel_clk          (video_clk),
    .pixel_rst_n        (video_rst_n),
    .video_hsync        (hs),
    .video_vsync        (vs),
    .video_de           (de),
    .video_x            (video_x_ref),
    .video_rgb          (vout_data_brd),
    .media_id           (media_id),
    .volume             (volume),
    // ---- 第 11 讲阶段 B：WAV 播放器接进来，wav_active=1 时 mux 选 WAV ----
    // 没有 MUSIC.PCM / 播放中拔卡断流 / WAV_PLAY_EN=0 时 wav_active=0，
    // 桥内 mux 自动回落琶音 —— 不会哑掉。
    .wav_valid          (wav_sample_valid),
    .wav_ready          (wav_sample_ready),
    .wav_left           (wav_sample_left),
    .wav_right          (wav_sample_right),
    .wav_active         (wav_active),
    // ---- 第16讲：警报音（优先级 警报 > WAV > 琶音）----
    .alarm_valid        (alarm_valid),
    .alarm_step         (alarm_step),
    .alarm_left         (alarm_left),
    .alarm_right        (alarm_right),
    .alarm_active       (alarm_active_v),
    .channel0_symbol    (tmds_ch0_data),
    .channel1_symbol    (tmds_ch1_data),
    .channel2_symbol    (tmds_ch2_data),
    .sequence_error     (dbg_seq_err),
    .tone_overflow      (dbg_tone_overflow),
    .pcm_contract_error (),
    .blip_active        (dbg_blip_active),
    .pcm_tap_valid      (),
    .pcm_tap_left       ()
);

// HDMI DDC（参考方案不读 EDID，引脚释放）
assign HDMI_DDC_SCL = 1'b1;
assign HDMI_DDC_SDA = 1'bz;

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (~video_rst_n),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule

// =====================================================================
// key_matrix_4x4: 核心板 4x4 矩阵键盘扫描（合并进本文件，便于工程管理）
//  行扫描: key_row[3:0] 输出低电平轮流扫描; 列读入: key_col[3:0] 10k上拉, 按下为低
//  输出低有效电平, 交给 sd_card_bmp 内部 key_press_debounce 消抖
//
//  丝印编号与 key_pressed 索引的实测映射（2026-10-15 板上验证）：
//     key_pressed[0] -> 丝印 KEY12（下一张）
//     key_pressed[1] -> 丝印 KEY13（自动播放开/关）
//     key_pressed[2] -> 丝印 KEY14（上一张）
//     key_pressed[3] -> 丝印 KEY15（未用）
//  注：丝印与矩阵行列存在错位，此处按实测固定映射，勿凭“KEY0=索引0”直觉改。
// =====================================================================
module key_matrix_4x4 (
    input  wire       clk,
    input  wire       rst,
    output wire [3:0] key_row,
    input  wire [3:0] key_col,
    output reg        o_key_next_n,
    output reg        o_key_auto_n,
    output reg        o_key_prev_n,
    // ★ 备用键：只有 key_pressed[3]（丝印 KEY15）按下才为低。
    //   现在专用于第15讲的"进出 2×2 缩略图页"。
    output reg        o_key_spare_n,
    // ★ 第16讲：应急警报"人工解除"键（低有效），取自 key_pressed[13] = 丝印 KEY9。
    //   若上板发现按 KEY9 没反应（矩阵行列与丝印不完全对应），把下面的 ACK_MASK
    //   改成 16'hE000，让 KEY9/KEY10/KEY11 任意一颗都能解除，即可零成本兜底（KEY11 现空闲）。
    output reg        o_key_ack_n,
    // ★ 第16讲：应急插播键（低有效），取自 key_pressed[14] = 丝印 KEY10。
    //   按一下 = 立刻跳到预案页（第 4 张）+ 拉响警报，不管当前在第几页、在不在缩略图页。
    //   与"缩略图选第 4 张退出"那条路并存，两条路都通向同一个状态（第 4 页 + 警报）。
    output reg        o_key_panic_n,
    // ★ 第14讲：亮度/对比度调节键（低有效），取自 key_pressed[7:4]
    //   [0]=亮度+  [1]=亮度-  [2]=对比度+  [3]=对比度-
    //   注：这三个丝印号需上板实测确认（矩阵扫描顺序与丝印不完全对应），
    //       按一次看哪一个有反应即可，接反了在顶层对调即可。
    output reg  [3:0] o_key_param_n
);

parameter integer SCAN_DIV = 5000;   // 每行保持约100us@50MHz, 16键全扫周期约400us

// 解除键掩码：bit13 = key_pressed[13] = 丝印 KEY9
localparam [15:0] ACK_MASK = 16'h2000;
// 应急插播键掩码：bit14 = 丝印 KEY10（唯一）
//   ★ 2026-10-31 用户要求：KEY11 不再兼任插播，留空给以后的功能。
//     当初把 bit15（KEY11）也收进来，是因为"KEY10 到底落在哪一位"还没实测；
//     现在板上已确认 KEY10 = bit14，双保险没意义了，收窄成一颗键更好讲。
localparam [15:0] PANIC_MASK = 16'h4000;

// ---------------- 扫描节拍 ----------------
reg [15:0] scan_cnt;
wire scan_tick = (scan_cnt == SCAN_DIV - 1);

always @(posedge clk or posedge rst) begin
    if (rst)
        scan_cnt <= 16'd0;
    else if (scan_tick)
        scan_cnt <= 16'd0;
    else
        scan_cnt <= scan_cnt + 16'd1;
end

// ---------------- 行扫描 0..3 轮流 ----------------
reg [1:0] row_idx;
always @(posedge clk or posedge rst) begin
    if (rst)
        row_idx <= 2'd0;
    else if (scan_tick)
        row_idx <= row_idx + 2'd1;
end

// 当前行拉低, 其余拉高
assign key_row = ~(4'b0001 << row_idx);

// ---------------- 列两级同步 + 行切换时刻采样 ----------------
reg [3:0] col_sync0;
reg [3:0] col_sync1;
reg [3:0] col_stable;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        col_sync0  <= 4'hF;
        col_sync1  <= 4'hF;
        col_stable <= 4'hF;
    end else begin
        col_sync0 <= key_col;
        col_sync1 <= col_sync0;
        if (scan_tick)
            col_stable <= col_sync1;
    end
end

// ---------------- 键值解码: key_pressed[i]=1 表示 KEYi 按下 ----------------
reg [15:0] key_pressed;
integer i;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        key_pressed <= 16'd0;
    end else if (scan_tick) begin
        for (i = 0; i < 4; i = i + 1)
            key_pressed[row_idx*4 + i] <= ~col_stable[i];
    end
end

// ---------------- 输出低有效电平 ----------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        o_key_next_n <= 1'b1;
        o_key_auto_n <= 1'b1;
        o_key_prev_n <= 1'b1;
        o_key_spare_n <= 1'b1;
        o_key_ack_n   <= 1'b1;
        o_key_panic_n <= 1'b1;
        o_key_param_n <= 4'hF;
    end else begin
        o_key_next_n <= ~key_pressed[0];   // KEY12 按下 -> 0
        o_key_auto_n <= ~key_pressed[1];   // KEY13 按下 -> 0
        o_key_prev_n <= ~key_pressed[2];   // KEY14 按下 -> 0
        // KEY15（bit3）= 第15讲：切 2×2 缩略图页。
        // ★ 2026-10-28 修正记录：原掩码写作 16'hFF0F，实际把 bit0~3 全部包含进来，
        //   于是按 KEY12/13/14 也会翻转当时挂在这个键上的开关，
        //   使"每一次加载的质量"随按键次数随机变化 —— 排查噪点时的游离变量。
        //   现为 16'h0008：只留 bit3；bit8~15 留给后续功能键，不再有副作用。
        o_key_spare_n <= ~(|(key_pressed & 16'h0008));
        // 第16讲：应急警报人工解除键（丝印 KEY9 → bit13）。低有效。
        o_key_ack_n <= ~(|(key_pressed & ACK_MASK));
        // 第16讲：应急插播键（丝印 KEY10 → bit14）。低有效。
        o_key_panic_n <= ~(|(key_pressed & PANIC_MASK));
        // 第14讲：[0]=亮度+ [1]=亮度- [2]=对比度+ [3]=对比度-
        o_key_param_n <= ~key_pressed[7:4];
    end
end

endmodule

// =====================================================================
// thumb_focus_border：2×2 缩略图页的十字隔条 + 焦点四角 L 形角标
//
//   画什么：
//     · 十字隔条：x=318..321、y=238..241 四条 4 像素黑带（把 640×480 分成 4 个 320×240）
//     · 焦点角标：选中象限的四个角画 L 形青色短臂（CW=24 长、BT=3 宽）
//
//   为什么用"四角 L 形"而不是"整圈边框"（第15讲第一版就是整圈）：
//     整圈 5px 青框在实机上看着像"图片没铺满、露出蓝底"，而且会压住左上角
//     的 OSD 状态文字。改成四角角标后既明确表示"当前选中"，又不遮挡内容。
//
//   ★ 行号为什么不用 vs（这是第15讲最大的坑，务必保留）：
//     本视频通路的 vs 是**低有效**（高电平覆盖整个有效画面），
//     第一版写 `if (vs) y <= 0;` ⇒ 有效区内 y 被死死按在 0 ⇒ qy 恒 0、ly 恒 0
//     ⇒ `ly < 6` 恒成立 ⇒ **整个左半屏被涂成 24'h00E5FF 青色**，
//     而且永远画不到 y=238 的横向隔条（实机照片：只有竖条没有横条）。
//     现在改成"de 下降沿做 mod-DH 计数"，完全不依赖 vs 极性：
//     每帧有效行数恒等于 DH，计数自然相位对齐，抗极性错误。
// =====================================================================
module thumb_focus_border #(
    parameter integer DW = 640,
    parameter integer DH = 480,
    parameter integer HW = 320,     // 半宽（象限宽）
    parameter integer HH = 240,     // 半高（象限高）
    parameter integer CW = 52,      // 角标臂长（24 → 40 → 52，2026-11-01 定稿）
    parameter integer BT = 8,       // 角标线宽（3 → 6 → 8，2026-11-01 定稿）
    parameter integer OL = 2        // 深色描边宽度（外圈，保证浅色页上也看得见）
)(
    input  wire        clk,
    input  wire        rst,         // 高有效
    input  wire        de,
    input  wire [9:0]  video_x,     // 有效区内 0..639
    input  wire        enable,      // 1 = 正在显示缩略图页
    input  wire [1:0]  focus,       // 0=左上 1=右上 2=左下 3=右下
    input  wire [23:0] rgb_in,
    output reg  [23:0] rgb_out,
    output reg         over
);
    // 角标配色（2026-11-01 用户要求"明黄、更醒目"）
    //
    // 只换颜色是不够的，先算对比度（相对亮度按 sRGB gamma 2.2 近似）：
    //   白色页底(245) vs 青 00E5FF   → 1.41 : 1   （原来那版，用户觉得"不明显"）
    //   白色页底(245) vs 明黄 FFD400 → 1.21 : 1   ← 纯黄在白底上比青还淡！
    //   白色页底(245) vs 深灰 202020 → 15.99 : 1
    //   红色页底(203,2,2) vs 明黄    → 4.48 : 1
    // 所以做法是**明黄 + 2px 深灰描边**：白底上那道深灰圈负责"看得见"，
    // 红底（第 4 页预案）上明黄本身对比 4.48:1 已经很跳。结论：任何页都醒目。
    localparam [23:0] CLR_ARM = 24'hFFD400;   // 明黄
    localparam [23:0] CLR_EDG = 24'h202020;   // 深灰描边

    // 行号：de 下降沿做 mod-DH 计数
    reg [11:0] y;
    reg        de_d;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            y    <= 12'd0;
            de_d <= 1'b0;
        end else begin
            de_d <= de;
            if (de_d && !de)
                y <= (y >= (DH - 1)) ? 12'd0 : (y + 12'd1);
        end
    end

    wire [11:0] x   = {2'b00, video_x};
    wire        qx  = (x >= HW);
    wire        qy  = (y >= HH);
    wire [1:0]  idx = {qy, qx};
    wire [11:0] lx  = qx ? (x - HW) : x;
    wire [11:0] ly  = qy ? (y - HH) : y;

    // 十字隔条（4 像素黑）
    wire gutter = ((x >= (HW - 2)) && (x <= (HW + 1))) ||
                  ((y >= (HH - 2)) && (y <= (HH + 1)));

    // 焦点象限的四角 L 形角标：外圈(描边) 与 内芯(明黄) 两套几何
    wire focused = (idx == focus);

    wire near_l_o = (lx < BT);
    wire near_r_o = (lx > (HW - 1 - BT));
    wire near_t_o = (ly < BT);
    wire near_b_o = (ly > (HH - 1 - BT));
    wire arm_v_o  = (near_l_o || near_r_o) && ((ly < CW) || (ly > (HH - 1 - CW)));
    wire arm_h_o  = (near_t_o || near_b_o) && ((lx < CW) || (lx > (HW - 1 - CW)));

    // 内芯向内缩 OL（从页边与臂端各缩 OL）⇒ 外圈自动形成 OL 宽的描边
    wire near_l_i = (lx < (BT - OL));
    wire near_r_i = (lx > (HW - 1 - (BT - OL)));
    wire near_t_i = (ly < (BT - OL));
    wire near_b_i = (ly > (HH - 1 - (BT - OL)));
    wire arm_v_i  = (near_l_i || near_r_i) && ((ly < (CW - OL)) || (ly > (HH - 1 - (CW - OL))));
    wire arm_h_i  = (near_t_i || near_b_i) && ((lx < (CW - OL)) || (lx > (HW - 1 - (CW - OL))));

    wire ring_o = focused & (arm_v_o || arm_h_o) & enable;
    wire core_i = focused & (arm_v_i || arm_h_i) & enable;

    wire gt = gutter & enable;
    wire bx = ring_o | core_i;

    always @(*) begin
        over    = de & (gt | bx);
        rgb_out = gt     ? 24'h000000 :
                  core_i ? CLR_ARM    :
                  ring_o ? CLR_EDG    : rgb_in;
    end
endmodule

// =====================================================================
// alarm_tone_gen: 插播警报音合成器（第16讲新增，合并进本文件便于工程管理）
//
// 为什么用 FPGA 合成而不是卡上放第二个音频文件：
//   卡上目前只有一条音频通道（fat32_volume_scanner 只认一个 8.3 短名
//   "MUSIC" + "PCM"），再加一个 ALARM.PCM 就要改扫描器 —— 而扫描器是整条
//   图片加载链的公共部件（第 5~7 讲的核心），改它的收益远小于风险。
//   警报音只有 1000Hz 方波 + 四段固定时长，用 ~40 个触发器就能合成，
//   顺便还证明"声音也能在片内算出来"。
//
// 波形：方波而不是正弦 —— 方波谐波丰富，喇叭听起来更"刺"，报警器就是这味儿；
//       而且只要一个计数器取反，不需要查表 ROM。
//   1000Hz @48kHz = 每周期 48 个样本，前半 24 个 +A、后半 24 个 -A。
//    750Hz @48kHz = 每周期 64 个样本，前半 32 个 +A、后半 32 个 -A。
//
// 节奏（不循环计时，插播期间一直响）：
//   1000Hz 响 0.35 s（16800 样本）→ 750Hz 响 0.35 s（16800 样本）→ 再换回 1000Hz …
//   两段之间没有静音，是连续的"双音警笛"，听感比"双短鸣"更像疏散警报。
//
// 握手：与 wav_stream_player 同一套"48kHz 节拍 + 单样本 pending"结构，
//   下游 ready=0 时 sample_valid 保持，不丢样本；run=0 时立刻 valid=0 且
//   计数器归零，保证下一次插播从"第一声"开始响。
// =====================================================================
module alarm_tone_gen #(
    parameter integer CLOCK_HZ  = 25_000_000,
    parameter integer SAMPLE_HZ = 48_000,
    parameter signed [15:0] AMP = 16'sd16000      // ≈0.49 FS：音乐峰值 0.68 FS 时警笛仍压得住
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               run,               // 1 = 正在插播（目标值）
    input  wire               step,              // 1 = 这一拍真的送出一个样本（由 sink 握手产生）
    output reg  signed [15:0] sample_left,
    output wire signed [15:0] sample_right,
    output wire               active              // 1 = 包络非零（桥用这个做 payload 替换）
);

    // ---- 节奏：连续双音警笛（1000Hz 与 750Hz 轮流，每段 0.35s，无静音）----
    //   2026-10-31 修订：原来是"双短鸣 + 0.5s 静音"的 0.9s 循环，听感偏"提示音"，
    //   用户反馈"太短"。现在改成不间断的双音警笛：听觉上更像疏散警报，
    //   而且没有静音段，插播期间一直响到 ALARM_HOLD_S 到点为止。
    localparam integer SEG_LEN  = SAMPLE_HZ * 35 / 100;   // 0.35 s 一段 = 16800
    localparam integer SQH_A    = SAMPLE_HZ / 2000;       // 1000Hz 半周期 = 24
    localparam integer SQH_B    = SAMPLE_HZ / 1500;       //  750Hz 半周期 = 32

    // ---- 节拍：由 sink 的真实握手（bridge 的 alarm_step）驱动 ----
    //   2026-10-31 三修：原来这里自带一个 48kHz 相位累加器（tick），
    //   但 sink 实际取样节拍是 mux_valid && source_ready，两者不完全一致：
    //   WAV 换曲空档 / FIFO 出水节奏变化时，同一个样本会被重复送出或跳过，
    //   方波边沿随之抖动 —— 用户听到的就是"密集打火声"。
    //   现在彻底不给合成器自己的时钟：送出一个样本才走一步，天然跟 sink 锁死。
    reg [15:0] seg_cnt;      // 段内第几个样本
    reg        seg_b;        // 0 = 1000Hz, 1 = 750Hz
    reg        seg_pend;     // 1 = 该换音高了，等当前半周期走完再换（避免波形被截断出"咔"）
    reg [5:0]  sq_cnt;       // 方波相位

    // ---- 起落包络：0 = 静音 / 1 = AMP/4 / 2 = AMP/2 / 3 = AMP ----
    //   2026-10-31 二修：原来是硬切（run 一掉就把 sample_left 清 0、一起就把
    //   方波从 0 跳到 +AMP），插播开始/结束各有一记"咔"。现在软起软落。
    localparam integer RAMP_TICKS = SAMPLE_HZ / 200;   // 5 ms = 240 个样本
    reg [1:0] ramp;
    reg [8:0] ramp_tmr;

    wire [5:0] sq_half = seg_b ? SQH_B[5:0] : SQH_A[5:0];
    wire [5:0] sq_top  = seg_b ? 6'd63     : 6'd47;

    wire signed [15:0] lvl = (ramp == 2'd3) ? AMP
                           : (ramp == 2'd2) ? (AMP >>> 1)
                           : (ramp == 2'd1) ? (AMP >>> 2)
                                            : 16'sd0;

    assign sample_right = sample_left;     // 单声道，左右同值
    assign active       = (ramp != 2'd0);  // 包络一起来就替换，收尾阶段仍在替换

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seg_cnt     <= 16'd0;
            seg_b       <= 1'b0;
            seg_pend    <= 1'b0;
            sq_cnt      <= 6'd0;
            ramp        <= 2'd0;
            ramp_tmr    <= 9'd0;
            sample_left <= 16'sd0;
        end else if (step) begin
            // ---- ① 包络：每 5 ms 升/降一档（run 只当"目标"用）----
            if (!run) begin
                if (ramp != 2'd0) begin
                    if (ramp_tmr >= (RAMP_TICKS - 1)) begin
                        ramp_tmr <= 9'd0;
                        ramp     <= ramp - 2'd1;
                    end else ramp_tmr <= ramp_tmr + 9'd1;
                end
            end else begin
                if (ramp != 2'd3) begin
                    if (ramp_tmr >= (RAMP_TICKS - 1)) begin
                        ramp_tmr <= 9'd0;
                        ramp     <= ramp + 2'd1;
                    end else ramp_tmr <= ramp_tmr + 9'd1;
                end
            end

            // ---- ② 相位：每个送出的样本走一步 ----
            if (!run && (ramp == 2'd0)) begin
                // 已经完全静音：相位归零，下次插播从第一声开始
                seg_cnt     <= 16'd0;
                seg_b       <= 1'b0;
                seg_pend    <= 1'b0;
                sq_cnt      <= 6'd0;
                sample_left <= 16'sd0;
            end else begin
                // 段长度只用来"提出换音高请求"，真正的切换放在半周期边界上，
                // 否则方波会被从中间截断，每 0.35s 出一记轻微的"咔"。
                if (seg_cnt >= (SEG_LEN - 1)) begin
                    seg_cnt  <= 16'd0;
                    seg_pend <= 1'b1;
                end else begin
                    seg_cnt <= seg_cnt + 16'd1;
                end

                if (sq_cnt >= sq_top) begin
                    sq_cnt <= 6'd0;
                    if (seg_pend) begin
                        seg_b    <= ~seg_b;      // 换音高（在过零点上换）
                        seg_pend <= 1'b0;
                    end
                end else begin
                    sq_cnt <= sq_cnt + 6'd1;
                end

                sample_left <= (sq_cnt < sq_half) ? lvl : -lvl;
            end
        end
    end

endmodule
