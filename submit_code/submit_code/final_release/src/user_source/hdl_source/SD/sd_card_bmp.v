
`include "bmp_err_def.vh"

module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4,
    // 2026-11-01 修订（w28）：500 → 2000。转场本身 21 帧 ≈ 0.35 s，500 ms 的间隔
    // 意味着每张图"完全静止"只有约 0.15 s，现场看着像抽帧；2000 ms ⇒ 每张静止约 1.65 s，
    // 4 张一轮 8 s（评委看得清、也不至于等太久）。要更慢改 3000、要更快改 1000。
    parameter integer AUTO_PLAY_MS      = 2000,      // 自动轮播"等待"间隔（毫秒）；实际切图总间隔≈等待+转场 0.35s
    // 2026-11-01 修订（w30）：日常轮播只在**前 N 张**里循环，第 4 张（应急预案页）不参与。
    //   原因：预案页是"事故状态"的画面，混在轮播里闪一下会让人误以为真出事了。
    //   注意：这只是缩小"自动轮播的候选集合"——手动 KEY12/KEY14 翻页、2×2 缩略图选图、
    //   KEY10 应急插播走的都是完整 bank_valid，仍然能正常到第 4 张。
    //   卡里只放 3 张图时自然退化成 3 张循环；改回 4 就恢复原来"四张全轮播"的行为。
    parameter integer AUTO_PLAY_BANKS   = 3,
    // 停在"预案页"时暂停轮播：否则 KEY10 插播后 2.35 s，轮播会把画面从预案页带回去
    //   （next_valid(3) 的下一个就是 0），而警笛还在响 —— 画面与声音对不上。
    //   插播或缩略图选中该页 ⇒ 暂停；用任何方式离开该页 ⇒ 自动恢复。
    parameter [1:0]   AUTO_HOLD_IDX     = 2'd3,
    // ===== 【拔插卡死锁修复】超时/看门狗参数 =====
    // 默认值即原硬件行为（100MHz 时钟），仿真时由 testbench 缩小以加速。
    parameter integer READ_WAIT_CYCLES    = 50_000_000,  // 500ms：源未读完且无数据
    parameter integer WRITE_WAIT_CYCLES   = 50_000_000,  // 500ms：源读完但整帧未写完
    parameter integer LOAD_TIMEOUT_CYCLES = 300_000_000, // 3s 兜底
    parameter integer EMPTY_SCAN_CYCLES   = 100_000_000, // 1s：0 图扫描后重试间隔
    parameter integer SCAN_WD_CYCLES      = 300_000_000, // 3s：扫描看门狗（扫描卡死则重扫）
    parameter integer SD_TIMEOUT_CYCLES   = 10_000_000,  // 100ms：适配层单扇区读超时
    parameter integer FAT_WD_CYCLES       = 20_000_000,  // 200ms：FAT32 客户端看门狗
    parameter integer SPI_LOW_DIV         = 248,         // SD 卡初始化阶段 SPI 分频
    parameter integer SPI_HIGH_DIV        = 0,           // SD 卡读写阶段 SPI 分频
    parameter [15:0]  CMD_TIMEOUT_BYTES   = 16'hFFFF,    // SD 命令响应超时（字节数）
    // 【2026-10-21 实机回归后关闭空闲探活】
    // 现象：插卡空闲/轮播时反复闪一瞬 "NO CARD"、画面跳回第 1 张。
    // 原因：S_PROBE 用 CMD13 探活，判据要求 R1 精确等于 0x00；而 CMD13 在 SPI
    //       模式返回的是 R2（R1+状态共 2 字节），R1 只要带状态位就被判"卡不在"
    //       → sd_init_done 被清 0 → OSD 闪 NO CARD → 50ms 后重新初始化又变 1
    //       → 重扫 → first_image_committed 清零 → 自动加载首图（画面回跳）。
    //       周期≈600ms（500ms 探活 + 50ms 稳定延时），与观察到的闪烁频率一致。
    // 影响：本开关只关"空闲主动发现拔卡"。拔卡后的黑屏/死锁修复不依赖它：
    //       整栈软复位 sd_flush + 100ms/200ms/500ms/3s 四层超时兜底全部保留。
    // 代价：静止画面下拔卡再插回，需按一次切图（或开着轮播）才重新扫卡恢复。
    // 如需重开：置 1 并先修好 CMD13 判据（放宽 R1 匹配 + 读走 R2 第二字节）。
    parameter integer PROBE_EN            = 0,           // 空闲保活探测（CMD13），默认关闭
    parameter integer PROBE_GAP_CYCLES    = 50_000_000,  // 探测间隔 500ms
    parameter integer PWR_WAIT_CYCLES     = 5_000_000,   // 重新初始化前稳定延时 50ms
    // ===== 第 11 讲阶段 B：WAV 音频播放 =====
    parameter integer WAV_EN          = 1,      // 1 = 识别到 MUSIC.PCM 就播放
    parameter integer WAV_LOOP        = 1,      // 1 = 单曲循环；0 = 播一遍回到琶音
    parameter integer WAV_LOOP_GAP    = 5_000,  // 循环接缝空档（100MHz 周期数，50μs）
    parameter integer WAV_RETRY_GAP   = 10_000_000, // 读失败后重试间隔（100ms）
    // ===== 【噪点排查 2026-10-27】错误码锁存开关 =====
    // 为什么：0x38(等不到写完成) / 0x45(帧长不足) 这类加载失败发生后，代码会
    //   ①在"下一次成功提交"处清成 ERR_NONE；
    //   ②扫描完成时也清一次。
    // 结果是一次失败只在屏幕上闪不到 1 秒，事后完全无法回答"这次上电期间到底
    // 失败过没有"。而"重扫→重新加载→写到正在显示的 bank"正是当前噪点最可疑
    // 的链路，必须先能看见它是否发生。
    // 置 1 = 错误码只在复位时清零（诊断态）；定版前改回 0 恢复原行为。
    // ★ 2026-10-28：噪点排查已进入"复位前后重拍比对"阶段，不再需要常驻错误码，
    //   改回 0（诊断任务由 N1 的"全程 ERR:---"已记录完成）。
    parameter integer DIAG_LATCH_ERR  = 0
)(
    input                       clk,
    input                       rst,
    input                       key_next,           // 下一张
    input                       key_auto,           // 自动播放 开/关
    input                       key_prev,           // 上一张
    // ★ 第15讲：缩略图页 KEY13 确认 → 直接跳到指定那张图（零延迟，不改缓冲策略）
    input                       jump_req,           // 单拍
    input  [1:0]                jump_idx,           // 目标图片索引 0~3
    output [3:0]                state_code,         // 状态机状态码（调试）
    output [1:0]                recover_state,      // 恢复状态机当前态（第5讲Step3可观测）0=OK 1=RETRY 2=SKIP 3=RESCAN
    output [3:0]                disp_num,           // 数码管显示：当前图片编号 1~4
    output [1:0]                media_id,           // 当前图片编号 0~3（供 HDMI 音频切图换音调）
    output reg [7:0]            bmp_error_code,     // 最近一次错误码（粘滞，成功提交后清零）
    output [2:0]                img_total,          // 已找到图片总数 1~4（供 OSD/UART）
    output                      sd_init_done_out,   // SD 初始化状态（供 OSD/UART）
    output                      load_error_out,     // 加载错误脉冲（供 UART 日志）
    output reg                  ld3s_pulse,         // 3s兜底超时触发脉冲（诊断用）
    output reg                  ld3s_src,           // 3s兜底触发时 source_done_seen 状态（诊断用：1=读完了写没完）
    output                      fat_dbg_pulse,      // FAT解析完成脉冲（透传自bmp_read，诊断用）
    output                      fat_dbg_ok,         // FAT解析合法性
    output [31:0]               fat_dbg_val,        // FAT解析出的数据区起始扇区
    output                      scan_diag_pulse,    // 扫描结束脉冲（透传，诊断用）
    output                      scan_diag_zero,     // 1=本轮0图收场
    output [31:0]               scan_diag_val,      // 首图扇区/终止扇区
    // ★ 第13讲：语义变更（原名 bmp_width/bmp_height，原来同时被当作"期望值"
    //   送进 decoder 做"必须等于"校验）。现在改为"支持的上限"，
    //   因为任意分辨率的图都要能过。
    input  [15:0]               max_width,          // 支持的最大宽（如 2048）
    input  [15:0]               max_height,         // 支持的最大高（如 1536）
    // ★ 第13讲新增：当前图片真实分辨率（来自 BMP 头），供写侧缩放器与 OSD 用
    output [15:0]               src_width_out,
    output [15:0]               src_height_out,
    output reg                  display_valid,

    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,

    // ★ 2026-10-31（B 方案）：把"哪个 bank 里有完整一帧"和"正在写哪个 bank"暴露给顶层。
    //   目的：视频通道的黑屏判据从"正在加载吗"(load_busy，要等四张全完)
    //         改成"要看的那一 bank 有内容吗"(bank_valid)，首图一写完就能显示。
    //   这两个量在 clk=sd_card_clk(100M) 域，顶层 video_clk 域用两级同步后使用。
    output wire [3:0]           bank_valid_o,       // 每位=该 bank 已有一帧完整内容
    output wire [1:0]           load_bank_o,        // 当前正在往哪个 bank 写（未加载时为 0）

    // ===== 第 11 讲阶段 B：WAV 音频文件流（全部在 clk = sd_card_clk 域）=====
    //   wav_fifo_room    ← 播放侧跨时钟域 FIFO 的剩余空间（扇区级流控用）
    //   wav_fifo_wr_*    → 把 16bit 小端样本写进那个 FIFO
    //   wav_play_en      → 电平：应该正在播（播放侧据此拉起/放下 wav_active）
    //   wav_audio_found  → 扫描到 MUSIC.PCM（OSD/UART 用）
    input  [12:0]               wav_fifo_room,
    output                      wav_fifo_wr_en,
    output [15:0]               wav_fifo_wr_data,
    output                      wav_play_en,
    output                      wav_audio_found,
    output                      wav_busy,
    output [7:0]                wav_fs_error,
    // ★ 第11讲阶段C：播放进度（秒），供 OSD 显示 "POS 3/6s"
    //   由 feeder 的 file_offset（已读字节数）经 wav_progress 一次性除法换算，
    //   仍是 clk = sd_card_clk 域；OSD 侧做两级同步。
    output [9:0]                wav_pos_sec,
    output [9:0]                wav_tot_sec,

    // 第6讲：SDRAM 初始化完成信号（ext_mem_clk 域），经内部 2FF 同步后
    // 门控上电扫描，确保 SDRAM 就绪后才开始读写，避免启动时写数据丢失
    input                       Sdr_init_done,

    output reg                  write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO
);

wire key_next_press;
wire key_auto_press;
wire key_prev_press;

wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire             sd_init_done;
// FAT32 卷扫描器输出（第七讲替换 bmp_read 的物理扇区扫描）
wire             scanner_scan_done;    // 扫描完成（单拍脉冲）
wire             scanner_scan_busy;    // 扫描中
reg              scan_done_latched;    // 扫描完成锁存（电平，供按键/轮播/首图判断）
wire [7:0]       scanner_fs_error;
wire [7:0]       scanner_spc;          // 每簇扇区数
wire [2:0]       scanner_spc_shift;    // log2(每簇扇区数)，供 streamer 移位代替乘法
wire [31:0]      scanner_fat_start;    // FAT 表起始扇区
wire [31:0]      scanner_data_start;   // 数据区起始扇区
wire [31:0]      scanner_max_cluster;  // 最大簇号
wire [3:0]       scan_resource_valid;  // 4 个文件有效标志
wire [31:0]      img_cluster0, img_cluster1, img_cluster2, img_cluster3;
wire [31:0]      img_size0, img_size1, img_size2, img_size3;
// 文件流读取器输出
wire             streamer_file_busy;
wire             streamer_file_done;
wire             streamer_file_valid;
wire [7:0]       streamer_file_byte;
wire [31:0]      streamer_file_offset;
wire [7:0]       streamer_fs_error;
// 第 11 讲：扫描器识别到的音频文件（MUSIC.PCM）
wire             scanner_audio_valid;
wire [31:0]      scanner_audio_cluster;
wire [31:0]      scanner_audio_size;
// 第11讲阶段C：feeder 文件内偏移 → 换算成秒
wire [31:0]      wav_file_offset;
wire [9:0]       wav_pos_sec_w;
wire [9:0]       wav_tot_sec_w;
// ===== 扇区读仲裁（第 11 讲：从"按阶段选 owner"改成"逐扇区三路仲裁"）=====
// 三个消费者共用同一个 sd_sector_adapter，每个扇区重新判一次归属。
// 历史实现是 use_streamer 三目：扫描阶段整段归 scanner，加载阶段整段归
// streamer。对图片够用，但音频需要每 5.33ms 一个扇区，会被图片突发饿死。
wire             scanner_sector_req;
wire [31:0]      scanner_sector_lba;
wire             streamer_sector_req;
wire [31:0]      streamer_sector_lba;
wire             wav_sector_req;        // 音频（最高优先级）
wire [31:0]      wav_sector_lba;
// 仲裁器主口（对接 sd_sector_adapter）
wire             arb_sector_req;
wire [31:0]      arb_sector_lba;
wire             arb_sector_busy;
wire             arb_sector_valid;
wire [7:0]       arb_sector_byte;
wire [8:0]       arb_sector_byte_index;
wire             arb_sector_done;
wire [7:0]       arb_sector_error;
wire [2:0]       arb_grant_dbg;         // 诊断：0=扫描器 1=图片 2=音频 7=空闲

reg              scan_start_pulse;
reg              load_start_pulse;
// load_kick：每次"发起一帧加载"时打一拍的单周期脉冲。
// 与 load_start_pulse（电平保持，供 streamer 采样）分开：
//   load_start_pulse → streamer 的 file_start（电平）
//   load_kick        → 顶层 SDRAM 写通道的启动请求（单拍）
reg              load_kick;
reg [31:0]       load_cluster;         // 加载的目标文件起始簇号
reg [31:0]       load_file_size;       // 加载的目标文件大小
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg [31:0]       auto_cnt;
reg [2:0]        img_found_count;
reg [1:0]        img_idx;              // 当前真正显示中的图片编号
reg [1:0]        load_idx;             // 当前正在写入的图片编号
reg [1:0]        pending_buf_idx;      // 当前正在写入的目标缓冲区
reg              next_req_pending;
reg              prev_req_pending;
reg              load_busy;

// ===== 第8讲阶段C：四 bank 上电预加载 =====
// preload_next：预加载队列"下一张要加载的图编号"。上电扫描完成后，按
// 0→1→2→3 依次把 4 张图加载进 bank0/1/2/3；每张加载完成提交后自增。
// all_loaded：4 张（或实际找到的 img_found_count 张）全部加载完成（粘滞）。
// 未 all_loaded 前切图请求排队（next/prev_req_pending 照常记录），
// 全部加载完后切图零延迟——只改 disp_buf_idx/img_idx 指针，不再碰 SD。
reg [1:0]        preload_next;
reg              all_loaded;

// ===== 第16讲：bank 有效性（拔卡后仍可切图的关键）=====
// 背景：原来"能不能切图"被绑在 all_loaded / img_found_count 上，而这两个量在
// 卡被拔掉（读卡超时 → 整栈软复位）时会被清零 ⇒ 已缓存进 SDRAM 的四张图还在，
// 却再也切不动，屏幕永远停在最后一张，现场看起来像死机。
//
// 现在把两件事拆开：
//   all_loaded / img_found_count  = "加载流程的状态"（卡在不在、扫到几张）——照旧会被清；
//   bank_valid[3:0]               = "这个 bank 里是不是一帧完整内容"——只由
//                                   「加载提交成功」置位、「扫描到图、准备重载」清零，
//                                   拔卡 / 读卡失败**不动它**。
// 切图判据从 all_loaded 换成 bank_valid ⇒ 卡拔掉后照样能翻页、轮播、缩略图选图、
// 走淡入淡出转场；只是不能再加载新内容（这正是终端该有的语义）。
reg  [3:0]       bank_valid;
wire [2:0]       bank_cnt = {2'd0, bank_valid[0]} + {2'd0, bank_valid[1]} +
                            {2'd0, bank_valid[2]} + {2'd0, bank_valid[3]};
// 注（w28）：原来的 cnt_eff（"有效张数，空时给 1"）已随 next_index_limited 一起废弃 ——
// 相邻张改成按 bank_valid 位图找（见文件末尾 next_valid / prev_valid），不再需要取模。

// ===== 【拔插卡死锁修复】整栈软复位脉冲 sd_flush =====
// 一拍内同时复位 accumulator 侧两个 FAT32 客户端（scanner / streamer）和
// sd_sector_adapter 的在途请求。原来的 bug：卡被拔掉时若 streamer 正卡在
// DATA_WAIT 等一个永远不会来的 sector_done，它就永远停在非 IDLE 态，之后
// 所有 file_start 被忽略 → 每张图都 500ms 超时 → 黑屏 + ERR:0x38，只能复位。
reg              sd_flush;
reg [31:0]       scan_wd_cnt;       // 扫描看门狗：扫描进行中累计
reg              zero_scan_latch;   // 本轮扫描 0 图（电平锁存，供"空扫描重试"计时）
// 找到的图片张数（resource_valid 的 1 的个数）
wire [2:0]       res_valid_cnt = {2'd0, scan_resource_valid[0]} +
                                 {2'd0, scan_resource_valid[1]} +
                                 {2'd0, scan_resource_valid[2]} +
                                 {2'd0, scan_resource_valid[3]};

// ===== 第5讲错误恢复：重试1次 → 跳过 → 保留上一帧 =====
wire [7:0]       decoder_bmp_error;    // bmp24_decoder 校验错误码（0x31~0x37）
wire [7:0]       bmp_error_w;          // 加载错误码（streamer 错→ERR_UPSTREAM；decoder 错→细分码）
wire             load_error_w;         // 加载错误脉冲（streamer 或 decoder 出错）
wire             fat_dbg_pulse_w;      // FAT 诊断（scanner 扫描完成脉冲）
wire             fat_dbg_ok_w;         // FAT 解析成功（scanner 无错误）
wire [31:0]      fat_dbg_val_w;        // 数据区起始扇区（scanner data_start_lba）
wire             scan_diag_pulse_w;    // 扫描结束诊断（scanner scan_done）
wire             scan_diag_zero_w;     // 0 图收场（resource_valid == 0）
wire [31:0]      scan_diag_val_w;      // 首图簇号（resource0_cluster）
reg              load_error_seen;      // 本次加载已重试过标志
reg [1:0]        retry_count;          // 重试计数（最多1次）
reg              scan_pending;         // 扫描进行中（用于扫描超时自动重扫）
reg [31:0]       empty_scan_cnt;       // 空扫描重试计数（扫描完成却0张图时延时重扫）

// bmp_ready 先表示"源文件读取/送 FIFO 完成"；真正切显示要等 write_finish_toggle 同步后
reg              source_done_seen;
// 锁存"整帧写完"脉冲，避免写完成早于源结束时脉冲丢失导致卡死
reg              write_done_seen;
// 加载超时保护：load_busy 持续过久则强制中止，恢复按键响应
reg [31:0]       load_timeout_cnt;
// 写完成快速超时：源读完(source_done_seen=1)但写完成(write_done_seen)迟迟不来时，
// 单独快速计时，把"写完成信号丢失"的恢复等待从 3s 压缩到 ~0.5s（正常写完成<100ms）
reg [31:0]       write_wait_cnt;
// 读完成快速超时：源没读完(source_done_seen=0，streamer 读文件卡住)时单独快速计时，
// 500ms 即重扫。拔卡后 SD 底层可能卡死(等 0xFE 令牌)，21ms 超时不一定能生效，
// 这里在 sd_card_bmp 层做兜底，保证 0.5s 内自恢复（不依赖 SD 底层是否卡死）。
reg [31:0]       read_wait_cnt;
// 加载活动标记：streamer 真正开始读文件（file_busy 上升沿）后才置位，
// 用于消除"启动当拍 file_busy 仍为低"导致的 source_done_seen 竞态
reg              load_active;
reg              file_busy_d;
// BMP 解析适配：文件流 → bmp24_decoder（像素组装字节序号 0=B 1=G 2=R）
reg [1:0]        px_byte_cnt;
// ---- 第13讲：BMP 行 stride（4 字节对齐 padding）处理 ----
//   BMP 每行字节数 = ceil(W*3 / 4) * 4，即 W*3 之后补 1~3 个 padding 字节。
//   640×480 时 640*3 = 1920，正好是 4 的倍数 → padding = 0，
//   所以老代码"每字节 0→1→2 循环、offset>=pixel_offset 就组装"能跑通。
//   换成任意分辨率后（如 850 宽 → 2550 → padding 2 字节），
//   不显式跳过就会把 padding 当像素字节，整幅错位。
reg [1:0]        row_pad_left;      // 当前行末还剩几个 padding 字节要吞掉
reg [15:0]       src_col;           // 当前行内列号 0..W-1

// 同步 mem_clk 域的 write_finish_toggle
reg [2:0]        wrfin_tgl_sync;
// 每帧加载发起时采样的 toggle 基准：写完成 = 同步值相对基准发生翻转（电平对比，
// 不依赖边沿脉冲，彻底消除拔插卡后跨帧残留翻转导致的写完成漏检 → 3s 兜底）
reg              wrfin_base;
wire             file_busy_rise;
// bmp24_decoder 输出（文件流 → 头解析/校验/像素组装）
wire             header_match_w;       // 11 项校验结果
wire [31:0]      pixel_offset_w;       // 像素数据偏移（用于判断像素区起点）
wire [31:0]      file_len_w;           // 文件长度（用于判断像素区终点）
wire [15:0]      img_width_w;          // 第13讲：图片真实宽（BMP 头解析结果）
wire [15:0]      img_height_w;         // 第13讲：图片真实高

assign src_width_out  = img_width_w;
assign src_height_out = img_height_w;

// 第13讲：行内字节数 W*3（24bpp）与行末 padding 字节数
//   padding = (4 - (W*3) % 4) % 4，W*3 已对齐时结果为 0
wire [17:0]      row_bytes_w = {2'd0, img_width_w} * 3'd3;
wire [1:0]       row_pad_w   = (2'd4 - row_bytes_w[1:0]) & 2'b11;

// 第6讲：Sdr_init_done（ext_mem_clk 域）跨时钟域同步到 sd_card_clk 域。
// 慢变电平信号，用 2FF 同步即可（同源 PLL，频率不同但相位固定，2FF 足够）。
reg [1:0]        sdram_init_sync;
wire             sdram_init_done_sd;
assign sdram_init_done_sd = sdram_init_sync[1];

wire auto_tick;
wire [1:0] next_from_current;
wire [1:0] prev_from_current;

// 自动轮播计时阈值：AUTO_PLAY_MS 毫秒（CLK_FREQ_HZ 为 Hz）
localparam [31:0] AUTO_CNT_MAX = CLK_FREQ_HZ / 1000 * AUTO_PLAY_MS - 1;

assign write_en   = bmp_data_wr_en;
assign write_data = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0};
// 自动轮播计时：AUTO_PLAY_MS 毫秒（CLK_FREQ_HZ 为 Hz）
assign auto_tick  = (auto_cnt == AUTO_CNT_MAX);
// 第16讲：不再用会被清零的 img_found_count；按"哪些 bank 真的有完整一帧"来找相邻张。
//   ★ 2026-11-01 修订（w28）：原来用 next_index_limited(img_idx, cnt_eff)，函数假设
//     有效 bank 就是 0..cnt-1 的连续前缀。一旦出现空洞（例如 PIC2 解码失败，
//     bank_valid = 4'b1011），从 bank0 出发 next 落 1(有效)但 prev 落 2(无效)，
//     从 bank1 出发 next 落 2(无效) ⇒ 按键被门控挡掉、看起来"卡住"，
//     bank3 永远到不了，自动轮播走同一路径也会卡。
//     改成沿环向前/向后最多探 3 步，取第一个 valid 的 bank；全无效则返回原位置。
assign next_from_current = next_valid(img_idx, bank_valid);
assign prev_from_current = prev_valid(img_idx, bank_valid);

// 第16讲修订（w30）：自动轮播用**受限的**候选集合（只在 bank_valid 与掩码的交集里找下一张）。
//   手动翻页 / 缩略图选图 / 应急插播仍然用上面的完整 bank_valid，行为不变。
wire [3:0] auto_valid = bank_valid & mask_n(AUTO_PLAY_BANKS[2:0]);
wire [1:0] auto_next  = next_valid(img_idx, auto_valid);

// 停在预案页时暂停轮播（见 AUTO_HOLD_IDX 注释）。
//   置位：插播 / 缩略图选中的目标就是该页
//   清除：画面已经不在该页（任何方式离开，包括 KEY9 解除回首页）
reg auto_hold;
always @(posedge clk or posedge rst) begin
    if (rst)                                              auto_hold <= 1'b0;
    else if (jump_req && (jump_idx == AUTO_HOLD_IDX))      auto_hold <= 1'b1;
    else if (img_idx != AUTO_HOLD_IDX)                     auto_hold <= 1'b0;
end
// streamer 开始读文件（file_busy 上升沿）：真正开始读数据
assign file_busy_rise = ~file_busy_d & streamer_file_busy;

// 数码管显示当前图片编号（1~4；尚未显示任何图时为 0）
assign disp_num = display_valid ? {2'b0, img_idx} + 4'd1 : 4'd0;
// 媒体编号输出（0~3），供 HDMI 音频桥接做切图换音调
assign media_id = img_idx;
// 图片总数（供 OSD/UART 显示 IMG:n/m）
assign img_total = img_found_count;
// ★ 2026-10-31（B 方案）：bank 有效位 + 当前写 bank，供顶层决定视频通道什么时候可以出图
assign bank_valid_o = bank_valid;
// 注意：这里必须用 write_buf_idx（真正正在写的那个 bank），不能用 pending_buf_idx
//   （pending_buf_idx 是"待写缓冲序号"，与 loader 送出的写地址差一，用错会导致
//    "正在写显示中的 bank 却没冻结读侧" ⇒ 半帧撕裂）
assign load_bank_o  = write_buf_idx;
// SD 初始化状态（供 OSD/UART 判断有无卡）
assign sd_init_done_out = sd_init_done;
// 加载错误脉冲（供 UART 日志）
assign load_error_out = load_error_w;
// FAT32 诊断重映射（scanner 输出 → 原 bmp_read 诊断端口，语义相近）
assign fat_dbg_pulse_w = scanner_scan_done;
assign fat_dbg_ok_w    = (scanner_fs_error == 8'd0);
assign fat_dbg_val_w   = scanner_data_start;
assign scan_diag_pulse_w = scanner_scan_done;
assign scan_diag_zero_w  = (scan_resource_valid == 4'd0);
assign scan_diag_val_w   = img_cluster0;
// 透传到顶层 UART（保持顶层接口不变）
assign fat_dbg_pulse = fat_dbg_pulse_w;
assign fat_dbg_ok    = fat_dbg_ok_w;
assign fat_dbg_val   = fat_dbg_val_w;
assign scan_diag_pulse = scan_diag_pulse_w;
assign scan_diag_zero  = scan_diag_zero_w;
assign scan_diag_val   = scan_diag_val_w;
// 加载错误码：streamer 文件系统错误 → ERR_UPSTREAM（触发重扫自恢复），
// decoder 11 项校验失败 → 细分错误码（坏图，走重试/跳过）。
assign bmp_error_w = (streamer_fs_error != 8'd0) ? ERR_UPSTREAM : decoder_bmp_error;
// 加载错误：streamer 文件读取错误，或 bmp24_decoder 11 项校验失败
assign load_error_w = (streamer_file_done && (streamer_fs_error != 8'd0)) ||
                      (streamer_file_done && !header_match_w);

// ===================== 复位副本（降扇出）=====================
// 2026-10-21 综合报告暴露的问题：顶层 u_sdcard_reset 的输出 sdcard_rst_n
// 单条网驱动 1113 个负载，走普通布线（非全局网络），光走线就 12.991ns，
// 在 100MHz（10ns）下有 310 个 recovery/removal/hold 违例端点，全部落在
// 异步复位端（.sr 引脚）——逻辑级数为 0，纯布线延迟。
// 做法：把 rst 复制成 4 份触发器输出，每份只驱动一组子模块，单网扇出降到约 1/4。
// 说明：① 副本比 rst 晚 1 拍释放。各子模块之间没有"必须同拍离开复位"的约束，
//          上电/按键复位场景下晚 1 拍无影响。
//       ② 本模块自身的 always 块仍用原始 rst，保证状态机第一时间离开复位。
//       ③ syn_keep 防止综合器把 4 个逻辑等价的寄存器合并回 1 个。
(* syn_keep = "true" *) reg rst_g0;   // 组0：3 个按键消抖
(* syn_keep = "true" *) reg rst_g1;   // 组1：sd_card_top（内部 3 个子模块）
(* syn_keep = "true" *) reg rst_g2;   // 组2：FAT32 卷扫描器 + 文件流读取器
(* syn_keep = "true" *) reg rst_g3;   // 组3：扇区适配层 + BMP 解码器

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rst_g0 <= 1'b1;
        rst_g1 <= 1'b1;
        rst_g2 <= 1'b1;
        rst_g3 <= 1'b1;
    end else begin
        rst_g0 <= 1'b0;
        rst_g1 <= 1'b0;
        rst_g2 <= 1'b0;
        rst_g3 <= 1'b0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        ld3s_pulse <= 1'b0;
        ld3s_src   <= 1'b0;
    end else begin
        ld3s_pulse <= 1'b0;
        // 兜底超时触发时打一拍脉冲，并记录 source_done_seen 状态：
        //   1 = SD 读已完成、SDRAM 写完成信号丢失（问题在写通道）
        //   0 = SD 读本身就没完成（问题在读通道）
        // 【2026-10-21】原来只覆盖 3s 兜底，而 500ms 快速超时路径完全不打日志，
        // 实机报 ERR:0x38 时串口只有 DS:xxxxxx，无法判断是读还是写超时。
        // 这里把三条超时路径统一纳入，串口看到 WR:3S 就是写通道、RD:3S 就是读通道。
        if (load_busy && ((load_timeout_cnt > LOAD_TIMEOUT_CYCLES) ||
                          (source_done_seen && !write_done_seen &&
                           (write_wait_cnt > WRITE_WAIT_CYCLES)) ||
                          (!source_done_seen && (read_wait_cnt > READ_WAIT_CYCLES)))) begin
            ld3s_pulse <= 1'b1;
            ld3s_src   <= source_done_seen;
        end
    end
end

// ============================================================================
// 恢复状态机（第5讲 Step3 架构化：状态机显式化 + 可观测）
// 本状态机为"观察者"：不干预上方主控制流，仅把恢复策略的当前阶段
// 编码成 2 位 recover_state，供数码管/OSD 实时显示，便于答辩演示与定位。
//
//   2'b00 ST_OK     正常/无恢复动作（含成功提交、空闲）
//   2'b01 ST_RETRY   第一次失败，正在自动重试（retry_count 已 +1）
//   2'b10 ST_SKIP    重试后仍失败，跳过坏图，保持上一帧
//   2'b11 ST_RESCAN  上游错误/超时，正在完整重扫自恢复
//
// 判定依据（均为主控制流当前一拍状态）：
//   - RESCAN：load_error_w 且 bmp_error_w==ERR_UPSTREAM（读卡无响应/拔卡）
//             或 3s 兜底超时（load_busy 且 load_timeout_cnt 超限）
//   - RETRY ：load_busy 且 load_error_w 且非上游错 且 尚未重试过（retry_count==0）
//   - SKIP  ：load_busy 且 load_error_w 且非上游错 且 已重试过（retry_count!=0）
//   - OK    ：其余
// ============================================================================
reg [1:0] recover_state_r;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        recover_state_r <= 2'b00;
    end else begin
        if ((load_error_w && (bmp_error_w == ERR_UPSTREAM)) ||
            (load_busy && (load_timeout_cnt > LOAD_TIMEOUT_CYCLES))) begin
            recover_state_r <= 2'b11;                     // ST_RESCAN
        end else if (load_busy && load_error_w && (bmp_error_w != ERR_UPSTREAM)) begin
            recover_state_r <= (retry_count == 2'd0) ? 2'b01 : 2'b10;  // RETRY / SKIP
        end else if (!load_busy) begin
            recover_state_r <= 2'b00;                     // ST_OK
        end
        // 其余情况（load_busy 正常推进中）保持上一状态
    end
end
assign recover_state = recover_state_r;

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_next (
    .clk        (clk),
    .rst        (rst_g0),
    .button_in  (key_next),
    .press_pulse(key_next_press)
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_auto (
    .clk        (clk),
    .rst        (rst_g0),
    .button_in  (key_auto),
    .press_pulse(key_auto_press)
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_prev (
    .clk        (clk),
    .rst        (rst_g0),
    .button_in  (key_prev),
    .press_pulse(key_prev_press)
);

function [1:0] next_index_limited;
    input [1:0] cur;
    input [2:0] count;
    begin
        case (count)
            3'd0: next_index_limited = 2'd0;
            3'd1: next_index_limited = 2'd0;
            3'd2: next_index_limited = (cur == 2'd1) ? 2'd0 : (cur + 2'd1);
            3'd3: next_index_limited = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);
            default: next_index_limited = (cur == 2'd3) ? 2'd0 : (cur + 2'd1);
        endcase
    end
endfunction

function [1:0] prev_index_limited;
    input [1:0] cur;
    input [2:0] count;
    begin
        case (count)
            3'd0: prev_index_limited = 2'd0;
            3'd1: prev_index_limited = 2'd0;
            3'd2: prev_index_limited = (cur == 2'd0) ? 2'd1 : (cur - 2'd1);
            3'd3: prev_index_limited = (cur == 2'd0) ? 2'd2 : (cur - 2'd1);
            default: prev_index_limited = (cur == 2'd0) ? 2'd3 : (cur - 2'd1);
        endcase
    end
endfunction

// 第16讲修订（w28）：按"有效位"找下一个 / 上一个 bank，能跳过空洞。
//   只在 4 个 bank 的环上探 3 步（第 4 步回到自身，没必要探），组合逻辑、无状态。
function [1:0] next_valid;
    input [1:0] cur;
    input [3:0] valid;
    reg   [1:0] s1, s2, s3;
    begin
        s1 = (cur == 2'd3) ? 2'd0 : (cur + 2'd1);          // +1
        s2 = (cur <= 2'd1) ? (cur + 2'd2) : (cur - 2'd2);  // +2
        s3 = (cur == 2'd0) ? 2'd3 : (cur - 2'd1);          // +3 (即 -1)
        if      (valid[s1]) next_valid = s1;
        else if (valid[s2]) next_valid = s2;
        else if (valid[s3]) next_valid = s3;
        else                next_valid = cur;
    end
endfunction

function [1:0] prev_valid;
    input [1:0] cur;
    input [3:0] valid;
    reg   [1:0] s1, s2, s3;
    begin
        s1 = (cur == 2'd0) ? 2'd3 : (cur - 2'd1);          // -1
        s2 = (cur >= 2'd2) ? (cur - 2'd2) : (cur + 2'd2);  // -2
        s3 = (cur == 2'd3) ? 2'd0 : (cur + 2'd1);          // -3 (即 +1)
        if      (valid[s1]) prev_valid = s1;
        else if (valid[s2]) prev_valid = s2;
        else if (valid[s3]) prev_valid = s3;
        else                prev_valid = cur;
    end
endfunction

// 第16讲修订（w30）：自动轮播的候选掩码 —— 低 n 位置 1。
//   n = AUTO_PLAY_BANKS，所以 n=3 时得到 4'b0111（只在前 3 张里循环）。
function [3:0] mask_n;
    input [2:0] n;
    begin
        mask_n = 4'b0000;
        if (n > 3'd0) mask_n[0] = 1'b1;
        if (n > 3'd1) mask_n[1] = 1'b1;
        if (n > 3'd2) mask_n[2] = 1'b1;
        if (n > 3'd3) mask_n[3] = 1'b1;
    end
endfunction

function [31:0] cluster_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: cluster_lut = img_cluster0;
            2'd1: cluster_lut = img_cluster1;
            2'd2: cluster_lut = img_cluster2;
            default: cluster_lut = img_cluster3;
        endcase
    end
endfunction

function [31:0] size_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: size_lut = img_size0;
            2'd1: size_lut = img_size1;
            2'd2: size_lut = img_size2;
            default: size_lut = img_size3;
        endcase
    end
endfunction

// 第8讲阶段C：四 bank 模式下不再需要 next_buf_lut 双缓冲倒腾，
// 写目标 bank 直接 = load_idx（图编号 → bank 编号一一对应）。

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        wrfin_base            <= 1'b0;
        sdram_init_sync       <= 2'b00;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_kick             <= 1'b0;
        load_cluster          <= 32'd0;
        load_file_size        <= 32'd0;
        scan_done_latched     <= 1'b0;
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 32'd0;
        img_found_count       <= 3'd0;
        img_idx               <= 2'd0;
        load_idx              <= 2'd0;
        pending_buf_idx       <= 2'd0;
        write_buf_idx         <= 2'd0;
        disp_buf_idx          <= 2'd0;
        preload_next          <= 2'd0;
        all_loaded            <= 1'b0;
        bank_valid            <= 4'b0000;   // 第16讲：复位时没有任何有效画面
        next_req_pending      <= 1'b0;
        prev_req_pending      <= 1'b0;
        load_busy             <= 1'b0;
        source_done_seen      <= 1'b0;
        write_done_seen       <= 1'b0;
        load_timeout_cnt      <= 32'd0;
        write_wait_cnt        <= 32'd0;
        read_wait_cnt         <= 32'd0;
        load_active           <= 1'b0;
        file_busy_d           <= 1'b0;
        px_byte_cnt           <= 2'd0;
        write_req             <= 1'b0;
        display_valid         <= 1'b0;
        load_error_seen       <= 1'b0;
        retry_count           <= 2'd0;
        scan_pending          <= 1'b0;
        empty_scan_cnt        <= 32'd0;
        bmp_error_code        <= ERR_NONE;
        sd_flush              <= 1'b0;
        scan_wd_cnt           <= 32'd0;
        zero_scan_latch       <= 1'b0;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        // 第6讲：Sdr_init_done 跨域同步（2FF）
        sdram_init_sync  <= {sdram_init_sync[0], Sdr_init_done};
        scan_start_pulse <= 1'b0;
        // load_kick 单拍默认撤销（发起分支里再置 1；非阻塞赋值"后写者胜"）
        load_kick        <= 1'b0;
        // 整栈软复位默认撤销（需要的分支里再置 1 拍；非阻塞赋值"后写者胜"）
        sd_flush         <= 1'b0;
        // load_start 电平保持：只在 streamer 真正开始读(上升沿)接收后才清除
        if (file_busy_rise)
            load_start_pulse <= 1'b0;
        file_busy_d      <= streamer_file_busy;
        // 扫描完成锁存：scanner 的单拍 scan_done → 电平，供按键/轮播/首图判断；
        // 下一次扫描发起时清 0。
        if (scanner_scan_done)
            scan_done_latched <= 1'b1;
        else if (scan_start_pulse)
            scan_done_latched <= 1'b0;

        // ===== 空扫描锁存（拔插卡死锁修复）=====
        // scan_done 是单拍脉冲，原实现直接拿它当计时门控，计数器每个周期被清零，
        // 1s 的空扫描看门狗永远计不满 —— 一旦扫到 0 张图就永久黑屏、只能按复位。
        // 这里锁存成电平，计时才真正有效。
        if (scanner_scan_done)
            zero_scan_latch <= (scan_resource_valid == 4'd0);
        else if (scan_start_pulse)
            zero_scan_latch <= 1'b0;

        // ===== 扫描看门狗（拔插卡死锁修复）=====
        // 扫描进行中却迟迟不返回 scan_done（扫描器被底层读吊死），强制作废本轮
        // 扫描并重新 kick，同时整栈软复位。没有这条，扫描一旦卡住就永久黑屏。
        if (scan_pending && !scan_done_latched) begin
            if (scan_wd_cnt >= SCAN_WD_CYCLES) begin
                scan_wd_cnt <= 32'd0;
                scan_kicked <= 1'b0;
                scan_pending<= 1'b0;
                sd_flush    <= 1'b1;
            end else begin
                scan_wd_cnt <= scan_wd_cnt + 32'd1;
            end
        end else begin
            scan_wd_cnt <= 32'd0;
        end

        // write_req 握手：加载发起时置 1（提前通知 SDRAM 写侧准备），
        // write_req_ack 到来后清 0。
        //
        // 【2026-10-21 三次修正】这里原来写的是 `if (load_start_pulse) write_req<=1;
        // else if (write_req_ack) write_req<=0;`，导致**上电只能加载第一张图**：
        //   · load_start_pulse 是"电平保持"（要等 streamer 真正开始读、file_busy
        //     上升沿才撤，见下面第 486 行），不是单拍脉冲；
        //   · write_req_ack 是顶层对 write_req 上升沿做边沿检测得到的单拍脉冲，
        //     恰好落在 load_start_pulse 仍然为 1 的那个周期；
        //   · 两个 if 分支里 load_start_pulse 优先 —— ack 清不掉 write_req。
        // 结果 write_req 置 1 后永久停在 1，后续每次加载都没有上升沿：
        //   loader 收不到 load_start → 像素到来时 !active → ERR 0x41；
        //   写完信号永远不来 → 500ms 写等待超时 → ERR 0x38（WR:3S），图切不动。
        // 现在用 load_kick（每个加载发起分支打一拍）来置请求，与电平保持解耦：
        // 每次加载必定产生且只产生一个 write_req 脉冲。
        if (load_kick)
            write_req <= 1'b1;
        else if (write_req_ack)
            write_req <= 1'b0;

        // 像素字节序号：加载发起时清 0，像素区内按行推进（第13讲）
        //   ★ 原来是"每字节 0→1→2→0 无脑循环"，只在 W*3 是 4 的倍数时成立。
        //     现在显式维护 行内列号 src_col 与 行末 padding 计数 row_pad_left：
        //       · padding 期：只递减 row_pad_left，不组装像素
        //       · 正常期：每 3 字节一个像素，第 W 个像素结束进入 padding 期
        //   这样 decoder 收到的像素流恰好是 W×H 个、顺序正确、无填充字节。
        if (load_start_pulse) begin
            px_byte_cnt  <= 2'd0;
            row_pad_left <= 2'd0;
            src_col      <= 16'd0;
        end else if (streamer_file_valid && (streamer_file_offset >= pixel_offset_w)) begin
            if (row_pad_left != 2'd0) begin
                row_pad_left <= row_pad_left - 2'd1;      // 吞掉行末 padding
            end else if (px_byte_cnt == 2'd2) begin
                px_byte_cnt <= 2'd0;                      // R 字节收完，回到 B
                if (src_col == (img_width_w - 16'd1)) begin
                    src_col      <= 16'd0;                // 行末：回到第 0 列
                    row_pad_left <= row_pad_w;            // 并进入 padding 期
                end else begin
                    src_col      <= src_col + 16'd1;
                end
            end else begin
                px_byte_cnt <= px_byte_cnt + 2'd1;
            end
        end

        // 第16讲：卡不在位 **且** 一个有效 bank 都没有时才走"整栈复位"；
        // 只要 bank 里还有完整画面，就继续走下面的正常流程，
        // 让切图/轮播/缩略图选图在拔卡后依然可用（只额外把 FAT32 栈按住）。
        if (!sd_init_done && (bank_cnt == 3'd0)) begin
            // 卡不在位（被拔掉 / 正在重新初始化）：
            // 【修复】保留当前显示画面（display_valid / disp_buf_idx 不清零），
            // 不再动不动黑屏；同时整栈软复位，保证 FAT32 客户端干净。
            scan_kicked           <= 1'b0;
            first_image_committed <= 1'b0;
            // 第16讲：只有"一个有效 bank 都没有"时才把轮播开关关掉；
            // 已经缓存过画面的情况下保持用户设的轮播状态，拔卡后继续轮播缓存图。
            if (bank_cnt == 3'd0) auto_play_en <= 1'b0;
            auto_cnt              <= 32'd0;
            img_found_count       <= 3'd0;
            load_idx              <= 2'd0;
            pending_buf_idx       <= 2'd0;
            write_buf_idx         <= 2'd0;
            next_req_pending      <= 1'b0;
            prev_req_pending      <= 1'b0;
            preload_next          <= 2'd0;
            all_loaded            <= 1'b0;
            load_busy             <= 1'b0;
            source_done_seen      <= 1'b0;
            write_done_seen       <= 1'b0;
            wrfin_base            <= 1'b0;
            load_timeout_cnt      <= 32'd0;
            write_wait_cnt        <= 32'd0;
            read_wait_cnt         <= 32'd0;
            load_active           <= 1'b0;
            load_error_seen       <= 1'b0;
            retry_count           <= 2'd0;
            scan_pending          <= 1'b0;
            // 扫描完成：DIAG_LATCH_ERR=1 时保留上一次的错误码，便于事后判读
            bmp_error_code        <= DIAG_LATCH_ERR ? bmp_error_code : ERR_NONE;
            write_req             <= 1'b0;   // 不能留下悬挂的写请求
            sd_flush              <= 1'b1;   // 整栈软复位：scanner/streamer/adapter
        end else begin
            // 第16讲：走到这里说明"卡不在位但 bank 里有缓存"（normal 流程继续跑）。
            // 只做两件事：把 FAT32 整栈按住（abort 是电平，栈停在 IDLE 不会死等），
            // 并把 scan_kicked 放开 —— 卡插回来时能立刻重新扫描。
            // 切图相关的状态（img_idx / disp_buf_idx / bank_valid / auto_play_en）一律不动。
            if (!sd_init_done) begin
                sd_flush    <= 1'b1;
                scan_kicked <= 1'b0;
            end
            // 扫描完成：缓存找到的图片总数（resource_valid 的 1 的个数）。
            // 第 11 讲起不再需要"切换 SD 归属"——仲裁器逐扇区自动判，
            // 扫描器扫完就不再发请求，SD 自然让给图片/音频。
            if (scanner_scan_done) begin
                img_found_count <= {2'd0, scan_resource_valid[0]} + {2'd0, scan_resource_valid[1]} +
                                   {2'd0, scan_resource_valid[2]} + {2'd0, scan_resource_valid[3]};
                // 第16讲：这次扫描确实找到了图 ⇒ 马上要重载 4 个 bank，
                // 先把"有效"清零（避免重载期间切到写了一半的 bank）；
                // 每张提交成功时再逐个置位。扫描没找到图（比如卡已拔出）就**不动**，
                // 让已缓存的画面继续可切 —— 这是"拔卡后还能翻页"的关键。
                if (scan_resource_valid != 4'b0000)
                    bank_valid <= 4'b0000;
            end

            // 检测加载真正开始：streamer 开始读文件（file_busy 上升沿）
            if (load_busy && file_busy_rise)
                load_active <= 1'b1;

            // 记住 streamer 已经把源文件送完（file_done），但还不能切显示，得等整帧写完
            if (load_active && streamer_file_done)
                source_done_seen <= 1'b1;

            // 锁存"整帧写完"：写完成 = 同步后的 toggle 电平相对本帧基准发生翻转。
            // 用电平对比替代边沿脉冲检测——write_finish_toggle 是跨帧累积的 toggle，
            // 拔插卡后上一帧残留的翻转会导致边沿在 load_active 窗口外被漏掉；
            // 电平对比则只要本帧写操作产生新翻转（奇数次）就能捕获，彻底消除 3s 兜底。
            if (load_active && (wrfin_tgl_sync[2] != wrfin_base))
                write_done_seen <= 1'b1;

            // 扫描结束标记：清除"扫描进行中"（用于扫描超时自动重扫）
            if (scanner_scan_done)
                scan_pending <= 1'b0;

            if (load_busy && load_error_w) begin
                // ===== 加载失败（第5讲恢复策略：重试1次→跳过→保留上一帧）=====
                load_timeout_cnt <= 32'd0;
                load_active      <= 1'b0;
                source_done_seen <= 1'b0;
                write_done_seen  <= 1'b0;
                bmp_error_code   <= bmp_error_w;
                if (bmp_error_w == ERR_UPSTREAM) begin
                    // 超时错误（读卡无响应/卡拔出）：不是普通坏图，触发完整重扫，
                    // 插回后自动恢复。释放 busy 并重置扫描状态。
                    // 【修复】保留当前显示画面不黑屏；整栈软复位（sd_flush）把
                    // 可能卡在等 sector_done 的 streamer 一并拉回 IDLE。
                    load_busy             <= 1'b0;
                    load_error_seen       <= 1'b0;
                    retry_count           <= 2'd0;
                    scan_kicked           <= 1'b0;
                    scan_pending          <= 1'b1;
                    first_image_committed <= 1'b0;
                    auto_play_en          <= 1'b0;
                    auto_cnt              <= 32'd0;
                    img_found_count       <= 3'd0;
                    load_idx              <= 2'd0;
                    pending_buf_idx       <= 2'd0;
                    write_buf_idx         <= 2'd0;
                    next_req_pending      <= 1'b0;
                    prev_req_pending      <= 1'b0;
                    preload_next          <= 2'd0;
                    all_loaded            <= 1'b0;
                    source_done_seen      <= 1'b0;
                    write_done_seen       <= 1'b0;
                    load_active           <= 1'b0;
                    load_timeout_cnt      <= 32'd0;
                    write_req             <= 1'b0;
                    sd_flush              <= 1'b1;
                end else if (!load_error_seen && (retry_count < 2'd1)) begin
                    // 第一次失败：自动重试1次（复用电平保持的 load_start_pulse）
                    load_error_seen  <= 1'b1;
                    retry_count      <= retry_count + 2'd1;
                    load_start_pulse <= 1'b1;
                    load_kick        <= 1'b1;
                end else begin
                    // 重试仍失败：跳过这张坏图，保持当前显示（不花屏）。
                    // 第8讲阶段C：预加载阶段坏图也要续发下一张，否则预加载
                    // 卡在坏图这一张，后续图永远不加载，all_loaded 置不上、
                    // 切图（依赖 all_loaded）失效。续发逻辑与提交分支一致，
                    // 唯一区别是坏图**不切换显示**（这张没加载成功）。
                    load_busy       <= 1'b0;
                    load_error_seen <= 1'b0;
                    retry_count     <= 2'd0;
                    if (!all_loaded && ({1'b0, preload_next} + 3'd1 < img_found_count)) begin
                        preload_next <= preload_next + 2'd1;
                        load_idx         <= preload_next + 2'd1;
                        load_cluster     <= cluster_lut(preload_next + 2'd1);
                        load_file_size   <= size_lut(preload_next + 2'd1);
                        write_buf_idx    <= preload_next + 2'd1;
                        load_start_pulse <= 1'b1;
                        load_kick        <= 1'b1;
                        load_busy        <= 1'b1;
                        source_done_seen <= 1'b0;
                        write_done_seen  <= 1'b0;
                        wrfin_base       <= wrfin_tgl_sync[2];
                        load_active      <= 1'b0;
                    end else begin
                        // 坏图是最后一张：预加载结束，但当前仍无图可显示时
                        // 保持现状（display_valid 已由首图/前几张决定）。
                        all_loaded   <= 1'b1;
                        preload_next <= 2'd0;
                    end
                end
            end else if (load_busy && source_done_seen && write_done_seen) begin
                // 只有"源图送完 + 整帧写完"都满足，才提交新图并切换显示缓冲区。
                // 第8讲阶段C：四 bank 下写目标 = load_idx（bank 编号），显示直接
                // 切到刚写完的 bank；预加载阶段提交后判断是否续发下一张。
                load_busy             <= 1'b0;
                load_timeout_cnt      <= 32'd0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                load_active           <= 1'b0;
                load_error_seen       <= 1'b0;
                retry_count           <= 2'd0;
                // 提交成功：DIAG_LATCH_ERR=1 时不抹掉错误码（见参数说明）
                bmp_error_code        <= DIAG_LATCH_ERR ? bmp_error_code : ERR_NONE;
                // 第8讲阶段C：四 bank 预加载期间，只在"首图提交"（还没提交过任何
                // 图）时切显示到 img0；后续第 1/2/3 张静默加载进 bank1/2/3，
                // 不打扰显示。这样预加载完成后屏幕停在首图，符合直觉。
                if (!first_image_committed) begin
                    disp_buf_idx <= load_idx;   // 首图切显示到 bank0
                    img_idx      <= load_idx;
                    display_valid<= 1'b1;
                end
                // 第16讲：这一帧已完整写进 bank(load_idx) ⇒ 标记该 bank 有内容
                bank_valid[load_idx] <= 1'b1;
                first_image_committed <= 1'b1;
                // 预加载续发：还没加载完 img_found_count 张就继续加载下一张。
                // preload_next 是"刚提交这张"的编号，下一张 = preload_next + 1。
                if (!all_loaded && ({1'b0, preload_next} + 3'd1 < img_found_count)) begin
                    preload_next <= preload_next + 2'd1;
                    // 续发下一张：写目标 bank = 下一张编号（四 bank 一一对应）
                    load_idx         <= preload_next + 2'd1;
                    load_cluster     <= cluster_lut(preload_next + 2'd1);
                    load_file_size   <= size_lut(preload_next + 2'd1);
                    write_buf_idx    <= preload_next + 2'd1;
                    load_start_pulse <= 1'b1;
                    load_kick        <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    wrfin_base       <= wrfin_tgl_sync[2];
                    load_active      <= 1'b0;
                end else begin
                    all_loaded   <= 1'b1;
                    preload_next <= 2'd0;
                end
            end else if (load_busy) begin
                // 快速超时兜底（把 3s 黑屏压缩到 ~0.5s）：
                //  - 写完成快速超时：源读完(source_done_seen=1)但写完成迟迟不来（500ms）
                //  - 读完成快速超时：源没读完(source_done_seen=0，streamer 读文件卡住，500ms）
                //    —— 拔卡后 SD 底层可能卡死(等 0xFE)，21ms 超时不一定生效，这里兜底
                if (load_timeout_cnt > LOAD_TIMEOUT_CYCLES ||
                    (source_done_seen && !write_done_seen && (write_wait_cnt > WRITE_WAIT_CYCLES)) ||
                    (!source_done_seen && (read_wait_cnt > READ_WAIT_CYCLES))) begin
                    // 超时（读坏图/截断文件导致死锁，或读/写完成信号丢失）强制释放 busy，恢复按键响应
                    // 【修复】保留当前显示画面不黑屏；整栈软复位，避免卡死的客户端残留
                    load_busy             <= 1'b0;
                    load_timeout_cnt      <= 32'd0;
                    write_wait_cnt        <= 32'd0;
                    read_wait_cnt         <= 32'd0;
                    source_done_seen      <= 1'b0;
                    write_done_seen       <= 1'b0;
                    load_active           <= 1'b0;
                    bmp_error_code        <= ERR_UPSTREAM;   // 上游兜底超时
                    // 触发完整重扫（与 ERR_UPSTREAM 错误路径对齐）：
                    // 兜底超时意味着读卡/写链路已异常，必须重扫自恢复，
                    // 否则 display_valid 保留旧图但扫描状态悬空，插回卡后无人 kick
                    scan_kicked           <= 1'b0;
                    scan_pending          <= 1'b1;
                    first_image_committed <= 1'b0;
                    auto_play_en          <= 1'b0;
                    auto_cnt              <= 32'd0;
                    img_found_count       <= 3'd0;
                    load_idx              <= 2'd0;
                    pending_buf_idx       <= 2'd0;
                    write_buf_idx         <= 2'd0;
                    next_req_pending      <= 1'b0;
                    prev_req_pending      <= 1'b0;
                    preload_next          <= 2'd0;
                    all_loaded            <= 1'b0;
                    write_req             <= 1'b0;
                    sd_flush              <= 1'b1;
                end else begin
                    load_timeout_cnt <= load_timeout_cnt + 32'd1;
                    // 写完成快速计时：只在"源读完但写未完成"时累计，其余清零。
                    if (source_done_seen && !write_done_seen)
                        write_wait_cnt <= write_wait_cnt + 32'd1;
                    else
                        write_wait_cnt <= 32'd0;
                    // 读完成快速计时（读进度看门狗）：源没读完时，
                    // streamer 有数据输出(file_valid)就清零，连续 500ms 无数据才累计。
                    // 这样正常加载（持续有数据，几百 ms）不会误判，只有真卡死
                    // （拔卡后 streamer 等不到扇区，无数据）才快速重扫。
                    if (!source_done_seen) begin
                        if (streamer_file_valid)
                            read_wait_cnt <= 32'd0;
                        else
                            read_wait_cnt <= read_wait_cnt + 32'd1;
                    end else
                        read_wait_cnt <= 32'd0;
                end
            end else begin
                load_timeout_cnt <= 32'd0;
                write_wait_cnt   <= 32'd0;
                read_wait_cnt    <= 32'd0;
                // 超时错误（读卡无响应）触发完整重扫：无论发生在扫描还是加载阶段，
                // 都重置扫描状态并重新扫前 4 张图。卡拔出/接触不良时持续重试，
                // 插回后自动恢复，无需按复位。
                if (load_error_w && (bmp_error_w == ERR_UPSTREAM)) begin
                    scan_kicked           <= 1'b0;   // 允许重新发起扫描
                    scan_pending          <= 1'b1;
                    first_image_committed <= 1'b0;
                    // 第16讲：已缓存过画面就保留用户的轮播开关
                    if (bank_cnt == 3'd0) auto_play_en <= 1'b0;
                    auto_cnt              <= 32'd0;
                    img_found_count       <= 3'd0;
                    load_idx              <= 2'd0;
                    pending_buf_idx       <= 2'd0;
                    write_buf_idx         <= 2'd0;
                    next_req_pending      <= 1'b0;
                    prev_req_pending      <= 1'b0;
                    preload_next          <= 2'd0;
                    all_loaded            <= 1'b0;
                    load_busy             <= 1'b0;
                    source_done_seen      <= 1'b0;
                    write_done_seen       <= 1'b0;
                    load_active           <= 1'b0;
                    load_timeout_cnt      <= 32'd0;
                    bmp_error_code        <= bmp_error_w;
                    write_req             <= 1'b0;
                    sd_flush              <= 1'b1;   // 整栈软复位，清掉可能的残留客户端
                end
            end

            // 上电后自动发起一次"扫描前 4 张 BMP"（FAT32 按文件名匹配）
            // 第6讲：等 SDRAM 初始化完成（sdram_init_done_sd）+ SD 卡初始化完成
            // （sd_init_done）后才启动扫描，否则未就绪就读会失败
            if (!scan_kicked && sd_init_done && sdram_init_done_sd) begin
                scan_start_pulse      <= 1'b1;
                scan_kicked           <= 1'b1;
                scan_pending          <= 1'b1;
                first_image_committed <= 1'b0;
                // 第16讲：重扫期间别把轮播开关关掉（缓存图还在，扫完能接着轮播）
                if (bank_cnt == 3'd0) auto_play_en <= 1'b0;
                auto_cnt              <= 32'd0;
                img_found_count       <= 3'd0;
                load_idx              <= 2'd0;
                pending_buf_idx       <= 2'd0;
                write_buf_idx         <= 2'd0;
                next_req_pending      <= 1'b0;
                prev_req_pending      <= 1'b0;
                preload_next          <= 2'd0;
                all_loaded            <= 1'b0;
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                load_active           <= 1'b0;
                load_timeout_cnt      <= 32'd0;
                // 注意：不清 display_valid / disp_buf_idx / img_idx。
                // 拔插卡恢复期间继续显示上一帧，避免黑屏；新图提交时再切缓冲。
            end else begin
                // ===== 空扫描重试（第5讲稳定性增强，拔插卡死锁修复）=====
                // 卡在位(sd_init_done=1)但扫描完成却一张图都没找到：
                // 典型场景是拔插过程中卡接触不良，扫描读到全垃圾扇区。
                // 【修复】原实现用单拍 scanner_scan_done 当计时门控，计数器每拍被
                // 清零，1s 永远计不满 —— 扫描到 0 张图后系统永久停摆（黑屏 + 只能
                // 复位）。这里改用电平锁存的 zero_scan_latch 计时，到点重新扫描。
                if (zero_scan_latch && scan_done_latched && !load_busy) begin
                    if (empty_scan_cnt >= EMPTY_SCAN_CYCLES) begin
                        empty_scan_cnt  <= 32'd0;
                        zero_scan_latch <= 1'b0;
                        scan_kicked     <= 1'b0;   // 释放，让自动重扫逻辑重新 kick
                        sd_flush        <= 1'b1;   // 清掉本轮扫描可能残留的客户端状态
                    end else begin
                        empty_scan_cnt <= empty_scan_cnt + 32'd1;
                    end
                end else begin
                    empty_scan_cnt <= 32'd0;
                end

                // 忙的时候也只记 1 次“下一张”请求，不会累积成连跳两张
                if (key_next_press && (bank_cnt > 3'd0))
                    next_req_pending <= 1'b1;

                // 上一张（KEY14）
                if (key_prev_press && (bank_cnt > 3'd1))
                    prev_req_pending <= 1'b1;

                // 自动播放开/关
                if (key_auto_press && (bank_cnt > 3'd1)) begin
                    auto_play_en <= ~auto_play_en;
                    auto_cnt     <= 32'd0;
                    // 开启轮播时立即请求切一次图（复用“手动下一张”通路），
                    // 避免按键后干等一个 AUTO_PLAY_MS 间隔才看到第一次切换。
                    if (!auto_play_en)
                        next_req_pending <= 1'b1;
                end

                // 自动播放 1s 计数：只有当前没有写图任务时才计时
                if (auto_play_en && display_valid && !load_busy && (bank_cnt > 3'd1)) begin
                    if (auto_tick)
                        auto_cnt <= 32'd0;
                    else
                        auto_cnt <= auto_cnt + 32'd1;
                end else begin
                    auto_cnt <= 32'd0;
                end

                // 首图自动加载到 bank0（第8讲阶段C：四 bank 上电预加载的起点）
                if (scan_done_latched && !first_image_committed && !load_busy && (img_found_count != 3'd0)) begin
                    load_idx         <= 2'd0;
                    load_cluster     <= img_cluster0;
                    load_file_size   <= img_size0;
                    // 四 bank：第 0 张固定写 bank0，后续 1/2/3 张由提交分支按
                    // preload_next 顺序续发到 bank1/2/3。不再用双缓冲倒腾。
                    pending_buf_idx  <= 2'd0;
                    write_buf_idx    <= 2'd0;
                    preload_next     <= 2'd0;
                    all_loaded       <= 1'b0;
                    load_start_pulse <= 1'b1;
                    load_kick        <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    wrfin_base       <= wrfin_tgl_sync[2];   // 采样本帧写完成基准
                    load_active      <= 1'b0;
                    next_req_pending <= 1'b0;
                    prev_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
                // ★ 第15讲：缩略图页 KEY13 确认 → 跳到选中那张（最高优先级，零延迟）
                //   与"下一张/上一张"同一条零延迟路径：只改 img_idx / disp_buf_idx，
                //   不重新加载 —— 四 bank 里内容早已就位。
                else if (jump_req && display_valid && !load_busy &&
                         bank_valid[jump_idx] && (jump_idx != img_idx)) begin
                    img_idx          <= jump_idx;
                    disp_buf_idx     <= jump_idx;
                    next_req_pending <= 1'b0;
                    prev_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
                // 手动下一张（第8讲阶段C：四 bank 全部加载完后切图零延迟，只改指针）
                else if (display_valid && !load_busy && next_req_pending && (bank_cnt > 3'd0) && bank_valid[next_from_current]) begin
                    img_idx         <= next_from_current;
                    disp_buf_idx    <= next_from_current;
                    next_req_pending <= 1'b0;
                    prev_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
                // 手动上一张：零延迟切指针
                else if (display_valid && !load_busy && prev_req_pending && (bank_cnt > 3'd1) && bank_valid[prev_from_current]) begin
                    img_idx         <= prev_from_current;
                    disp_buf_idx    <= prev_from_current;
                    next_req_pending <= 1'b0;
                    prev_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
                // 自动播放下一张：零延迟切指针
                //   w30：候选集合限于 auto_valid（默认前 3 张），且停在预案页时 auto_hold 挡住，
                //        避免"插播后 2.35 s 又被轮播带离预案页"。
                else if (display_valid && !load_busy && auto_play_en && auto_tick &&
                         (auto_valid != 4'b0000) && !auto_hold && (auto_next != img_idx)) begin
                    img_idx         <= auto_next;
                    disp_buf_idx    <= auto_next;
                    next_req_pending <= 1'b0;
                    prev_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
            end
        end
    end
end

// ============================================================================
// 第七讲：FAT32 文件系统（替换 bmp_read 的物理扇区扫描）
//   scanner 识别文件系统 + 按文件名匹配，streamer 按 FAT 链读文件，
//   adapter 把 sector_* 接口转成官方 sd_sec_* 接口，bmp24_decoder 保留 11 项校验。
// ============================================================================

// ---------- 扇区读仲裁：三路逐扇区仲裁（第 11 讲阶段 B）----------
// 每个扇区读完后重新判归属，优先级：音频 > 图片 > 扫描器。
// 三个客户端各自看到独立的 busy/valid/done 和字节流，互不串扰。
wire             scanner_sector_busy;
wire             scanner_sector_valid;
wire [7:0]       scanner_sector_byte;
wire [8:0]       scanner_sector_byte_index;
wire             scanner_sector_done;
wire [7:0]       scanner_sector_error;

wire             streamer_sector_busy;
wire             streamer_sector_valid;
wire [7:0]       streamer_sector_byte;
wire [8:0]       streamer_sector_byte_index;
wire             streamer_sector_done;
wire [7:0]       streamer_sector_error;

wire             wav_sector_busy;
wire             wav_sector_valid;
wire [7:0]       wav_sector_byte;
wire [8:0]       wav_sector_byte_index;
wire             wav_sector_done;
wire [7:0]       wav_sector_error;

sd_sector_arbiter u_arbiter (
    .clk            (clk),
    .rst_n          (~rst_g2),
    .abort          (sd_flush),

    .c0_req         (scanner_sector_req),
    .c0_lba         (scanner_sector_lba),
    .c0_busy        (scanner_sector_busy),
    .c0_valid       (scanner_sector_valid),
    .c0_byte        (scanner_sector_byte),
    .c0_byte_index  (scanner_sector_byte_index),
    .c0_done        (scanner_sector_done),
    .c0_error       (scanner_sector_error),

    .c1_req         (streamer_sector_req),
    .c1_lba         (streamer_sector_lba),
    .c1_busy        (streamer_sector_busy),
    .c1_valid       (streamer_sector_valid),
    .c1_byte        (streamer_sector_byte),
    .c1_byte_index  (streamer_sector_byte_index),
    .c1_done        (streamer_sector_done),
    .c1_error       (streamer_sector_error),

    .c2_req         (wav_sector_req),
    .c2_lba         (wav_sector_lba),
    .c2_busy        (wav_sector_busy),
    .c2_valid       (wav_sector_valid),
    .c2_byte        (wav_sector_byte),
    .c2_byte_index  (wav_sector_byte_index),
    .c2_done        (wav_sector_done),
    .c2_error       (wav_sector_error),

    .m_req          (arb_sector_req),
    .m_lba          (arb_sector_lba),
    .m_busy         (arb_sector_busy),
    .m_valid        (arb_sector_valid),
    .m_byte         (arb_sector_byte),
    .m_byte_index   (arb_sector_byte_index),
    .m_done         (arb_sector_done),
    .m_error        (arb_sector_error),

    .grant_dbg      (arb_grant_dbg)
);

// ---------- 卷扫描器 ----------
fat32_volume_scanner #(
    .SECTOR_TIMEOUT_CYCLES (FAT_WD_CYCLES)
) u_scanner (
    .clk                 (clk),
    .rst_n               (~rst_g2),
    .abort               (sd_flush),
    .scan_start          (scan_start_pulse),
    .scan_busy           (scanner_scan_busy),
    .scan_done           (scanner_scan_done),
    .fs_error            (scanner_fs_error),
    .partition_lba       (),
    .sectors_per_cluster (scanner_spc),
    .sectors_per_cluster_shift (scanner_spc_shift),
    .fat_start_lba       (scanner_fat_start),
    .data_start_lba      (scanner_data_start),
    .max_cluster         (scanner_max_cluster),
    .resource_valid      (scan_resource_valid),
    .resource0_cluster   (img_cluster0),
    .resource1_cluster   (img_cluster1),
    .resource2_cluster   (img_cluster2),
    .resource3_cluster   (img_cluster3),
    .resource0_size      (img_size0),
    .resource1_size      (img_size1),
    .resource2_size      (img_size2),
    .resource3_size      (img_size3),
    // 第 11 讲：顺带识别的音频文件
    .audio_valid         (scanner_audio_valid),
    .audio_cluster       (scanner_audio_cluster),
    .audio_size          (scanner_audio_size),
    .sector_req          (scanner_sector_req),
    .sector_lba          (scanner_sector_lba),
    .sector_busy         (scanner_sector_busy),
    .sector_valid        (scanner_sector_valid),
    .sector_byte         (scanner_sector_byte),
    .sector_byte_index   (scanner_sector_byte_index),
    .sector_done         (scanner_sector_done),
    .sector_error        (scanner_sector_error)
);

// ---------- 文件流读取器 ----------
fat32_file_streamer #(
    .SECTOR_TIMEOUT_CYCLES (FAT_WD_CYCLES)
) u_streamer (
    .clk                 (clk),
    .rst_n               (~rst_g2),
    .abort               (sd_flush),
    .file_start          (load_start_pulse),
    // 图片加载不需要流控：一次读几 MB，灌满就走
    .file_pause          (1'b0),
    .first_cluster       (load_cluster),
    .file_size           (load_file_size),
    .sectors_per_cluster (scanner_spc),
    .sectors_per_cluster_shift (scanner_spc_shift),
    .fat_start_lba       (scanner_fat_start),
    .data_start_lba      (scanner_data_start),
    .max_cluster         (scanner_max_cluster),
    .file_busy           (streamer_file_busy),
    .file_done           (streamer_file_done),
    .file_valid          (streamer_file_valid),
    .file_byte           (streamer_file_byte),
    .file_byte_offset    (streamer_file_offset),
    .fs_error            (streamer_fs_error),
    .sector_req          (streamer_sector_req),
    .sector_lba          (streamer_sector_lba),
    .sector_busy         (streamer_sector_busy),
    .sector_valid        (streamer_sector_valid),
    .sector_byte         (streamer_sector_byte),
    .sector_byte_index   (streamer_sector_byte_index),
    .sector_done         (streamer_sector_done),
    .sector_error        (streamer_sector_error)
);

// ---------- 第 11 讲阶段 B：WAV 音频供给器（复用 streamer + 扇区级流控）----------
// wav_play_en：扫到 MUSIC.PCM 且扫描完成、卡就绪 → 持续为 1，直到拔卡/重扫。
//   它同时是播放侧的"该播了"电平，播放侧据此拉起 wav_active（再经 mux 选中 WAV）。
reg wav_play_en_r;
always @(posedge clk or posedge rst) begin
    if (rst)
        wav_play_en_r <= 1'b0;
    else
        wav_play_en_r <= WAV_EN[0] && sd_init_done && scan_done_latched &&
                         scanner_audio_valid;
end
assign wav_play_en     = wav_play_en_r;
assign wav_audio_found = scanner_audio_valid;

wav_sector_feeder #(
    .LOOP_EN          (WAV_LOOP),
    .LOOP_GAP_CYCLES  (WAV_LOOP_GAP),
    .RETRY_GAP_CYCLES (WAV_RETRY_GAP),
    .FIFO_ROOM_MIN    (256)             // 一个扇区 512 字节 = 256 个 16bit 字
) u_wav_feeder (
    .clk                       (clk),
    .rst_n                     (~rst_g2),
    .abort                     (sd_flush),

    .play_en                   (wav_play_en_r),
    .first_cluster             (scanner_audio_cluster),
    .file_size                 (scanner_audio_size),
    .sectors_per_cluster       (scanner_spc),
    .sectors_per_cluster_shift (scanner_spc_shift),
    .fat_start_lba             (scanner_fat_start),
    .data_start_lba            (scanner_data_start),
    .max_cluster               (scanner_max_cluster),

    .fifo_room                 (wav_fifo_room),
    .fifo_wr_en                (wav_fifo_wr_en),
    .fifo_wr_data              (wav_fifo_wr_data),

    .busy                      (wav_busy),
    .done_pulse                (),
    .fs_error                  (wav_fs_error),
    .file_offset_out           (wav_file_offset),

    .sector_req                (wav_sector_req),
    .sector_lba                (wav_sector_lba),
    .sector_busy               (wav_sector_busy),
    .sector_valid              (wav_sector_valid),
    .sector_byte               (wav_sector_byte),
    .sector_byte_index         (wav_sector_byte_index),
    .sector_done               (wav_sector_done),
    .sector_error              (wav_sector_error)
);

// ---------- 第11讲阶段C：播放进度换算（字节 → 秒）----------
// 一次性移位减法的除法器（32 拍算完），时分复用算两个数：
//   pos = file_offset / 96000、tot = file_size / 96000
// 全部在 sd_card_clk 域内完成，跨域交给 OSD 里的两级同步（1Hz 级慢变化）。
assign wav_pos_sec = wav_pos_sec_w;
assign wav_tot_sec = wav_tot_sec_w;

wav_progress #(
    .BYTES_PER_SEC (96000),     // 48000 样本/s × 2 字节
    .MAX_SEC       (999)
) u_wav_progress (
    .clk         (clk),
    .rst_n       (~rst_g2),
    .size_stable (scan_done_latched && scanner_audio_valid),
    .file_size   (scanner_audio_size),
    .file_offset (wav_file_offset),
    .pos_sec     (wav_pos_sec_w),
    .tot_sec     (wav_tot_sec_w)
);

// ---------- SD 卡接口适配层（sector_* → 官方 sd_sec_*）----------
sd_sector_adapter #(
    .TIMEOUT_CYCLES         (SD_TIMEOUT_CYCLES)
) u_adapter (
    .clk                    (clk),
    .rst_n                  (~rst_g3),
    .abort                  (sd_flush),
    .sector_req             (arb_sector_req),
    .sector_lba             (arb_sector_lba),
    .sector_busy            (arb_sector_busy),
    .sector_valid           (arb_sector_valid),
    .sector_byte            (arb_sector_byte),
    .sector_byte_index      (arb_sector_byte_index),
    .sector_done            (arb_sector_done),
    .sector_error           (arb_sector_error),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_end        (sd_sec_read_end)
);

// ---------- BMP 解析适配：文件流 → bmp24_decoder（保留 11 项校验）----------
// 像素区起点 = pixel_offset（decoder 解析的头字段），终点由 file_valid 隐含（file_size 内）
// ★ 第13讲：行末 padding 期间（row_pad_left != 0）禁止组装像素
wire in_pixel_area  = streamer_file_valid && (streamer_file_offset >= pixel_offset_w);
wire px_assemble_en = in_pixel_area && (row_pad_left == 2'd0);

// 调试状态码：1=扫描中 2=加载中 0=空闲（原 bmp_read 的 state_code 语义简化）
assign state_code = scanner_scan_busy ? 4'd1 : (streamer_file_busy ? 4'd2 : 4'd0);

bmp24_decoder u_bmp24_decoder (
    .clk               (clk),
    .rst               (rst_g3),
    .max_width         (max_width),
    .max_height        (max_height),
    // 头解析：文件流前 512 字节（BMP 头 54 字节在内）
    .hdr_parse_en      (streamer_file_valid && (streamer_file_offset < 32'd512)),
    .hdr_byte          (streamer_file_byte),
    .hdr_byte_valid    (streamer_file_valid && (streamer_file_offset < 32'd512)),
    .hdr_byte_idx      (streamer_file_offset[9:0]),
    // 校验输出
    .header_match      (header_match_w),
    .bmp_error         (decoder_bmp_error),
    .header_0_out      (),
    .header_1_out      (),
    .file_len_out      (file_len_w),
    .pixel_offset_out  (pixel_offset_w),
    // ★ 第13讲：真实分辨率输出（写侧缩放器抽样用、OSD 显示用）
    .img_width_out     (img_width_w),
    .img_height_out    (img_height_w),
    // 像素组装：从像素偏移开始，BGR 三字节拼 RGB888
    .px_assemble_en    (px_assemble_en),
    .px_byte           (streamer_file_byte),
    .px_byte_idx       (px_byte_cnt),
    // 像素输出（接写 FIFO 链路）
    .pixel_valid       (bmp_data_wr_en),
    .pixel_rgb         (bmp_data)
);

sd_card_top #(
    .SPI_LOW_SPEED_DIV      (SPI_LOW_DIV            ),
    .SPI_HIGH_SPEED_DIV     (SPI_HIGH_DIV           ),
    .CMD_TIMEOUT_BYTES      (CMD_TIMEOUT_BYTES      ),
    .PROBE_EN               (PROBE_EN               ),
    .PROBE_GAP_CYCLES       (PROBE_GAP_CYCLES       ),
    .PWR_WAIT_CYCLES        (PWR_WAIT_CYCLES        )
) sd_card_top_m0(
    .clk                    (clk),
    .rst                    (rst_g1),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule

module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,    // 默认：松开=1，按下=0
    output reg  press_pulse
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule
