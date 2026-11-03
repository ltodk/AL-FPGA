`timescale 1ns/1ps
// =============================================================================
// media_sdram_write_cdc —— 帧缓存写通道的跨时钟域搬运（第8讲）
//
// 为什么需要它：
//   讲义里完赛工程把 BMP 解码器也放在 mem_clk 域，解码器和 loader 同时钟，直接相连。
//   我校工程不同：BMP 解码跑在 sd_card_clk(100MHz)，SDRAM 控制器跑在
//   ext_mem_clk(125MHz)，两者频率不同、相位无关。loader 输出的
//   app_wr_en/addr/data 是"每拍都可能变化"的总线，绝不能直接跨域。
//
// 做法：
//   src 侧（100MHz）把每一次写请求打成 {addr, mask, data} 共 57 位压进异步命令
//   FIFO；dst 侧（125MHz）按 128 字一块组织，一块数据攒满才发起连续 burst，
//   保证 App_wr_en 是成块连续的，不会在中途断流。
//
// 指针跨域用格雷码（每次只变 1 位），两侧各自两级同步，避免亚稳态。
//
// ★ 与 SDRAM 读通道的互斥（2026-10-21 修）：
//   SDRAM 控制器的 App 端口不能读写同时进行。原来写侧由 frame_fifo_write
//   驱动，读侧靠它的 O_wr_busy 避让；改成 CDC 直接驱动 App_wr_* 后这条链
//   断了，读写 burst 互相打断 → 画面出现随机横向亮线（花屏）。
//   现在恢复成"写优先 + 读让路 + 写主动让位"：
//     · dst_wr_busy 送到 frame_fifo_read 的 App_wr_busy，读侧据此让路；
//       它包含"即将启动"（组合），读侧同一拍就能看到，与写侧零竞争。
//     · dst_rd_busy 是读侧 burst 进行中，写侧在启动新 burst 前必须等它落。
//     · 每发完一个 burst 强制让位 YIELD_CYCLES 拍，否则一帧 2400 个 burst
//       连发会把读侧饿死（显示断流同样花屏）。
// =============================================================================
module media_sdram_write_cdc #(
    parameter integer ADDR_WIDTH       = 21,
    parameter integer FIFO_ADDR_WIDTH  = 9,      // 512 深
    parameter integer BURST_SIZE       = 64,
    // 写后静默期拍数（0 = 关闭，用于复现"修复前"的横条现象；24 = 正常修复值）
    parameter integer GUARD_CYCLES     = 24
) (
    // ---- 源侧：loader 所在时钟域 ----
    input  wire                  src_clk,
    input  wire                  src_rst_n,
    input  wire                  src_wr_en,
    input  wire [ADDR_WIDTH-1:0] src_wr_addr,
    input  wire [3:0]            src_wr_mask,
    input  wire [31:0]           src_wr_data,
    output wire                  src_full,
    output reg                   src_overflow,
    // ★ 2026-10-27 新增：新帧启动脉冲（src_clk 域，与 loader 的 load_start 同源）。
    //   见下方 src 侧 always 里的"残留块收尾"，这是修 ERR:0x38 永久卡死的关键。
    input  wire                  src_frame_start,

    // ---- 目的侧：SDRAM 控制器所在时钟域 ----
    input  wire                  dst_clk,
    input  wire                  dst_rst_n,
    output wire                  dst_app_wr_en,
    output wire [ADDR_WIDTH-1:0] dst_app_wr_addr,
    output wire [3:0]            dst_app_wr_mask,
    output wire [31:0]           dst_app_wr_data,
    output wire                  dst_idle,          // 全部排空（顶层用它判"真正写完"）

    // ---- 与 SDRAM 读通道的互斥握手 ----
    input  wire                  dst_rd_busy,       // 读侧 burst 进行中，写侧必须等
    output wire                  dst_wr_busy,       // 写侧 busy（含即将启动），读侧据此刻避让

    // ★ 2026-10-28 新增：SDRAM 刷新窗口标志（dst_clk 域，来自 sdr_as_ram）。
    //   官方 APUG011 参考设计 app_wrrd.v 的硬性规则：
    //     if(judge_cnt==3'b111 & (!Sdr_init_ref_vld)) tx_vld <= 1'b1;   // 空闲才启动
    //     else if(tx_cnt[13] | Sdr_init_ref_vld)     tx_vld <= 1'b0;   // 刷新一来立刻停
    //   我们原来【只有读侧】避让刷新，写侧从来没检查过；写一张图约 300ms，
    //   期间刷新约 9600 次，每次约 20 拍 ⇒ 约 0.5% 的写窗口踩进刷新 ⇒
    //   每帧约 1500 个像素被写坏（≈1% 密度），且位置固定。
    input  wire                  dst_ref_vld,

    // ---- 答辩演示开关（KEY8）----
    //   1 = 关闭写后静默期（复现"修复前"的横条现象）
    //   0 = 静默期生效（正常，横条消失）
    //   来自 50MHz 按键扫描域，模块内部用 dst_clk 做两级同步。
    input  wire                  guard_dis,

    // ★ 2026-10-29 第15讲新增：外部占住 SDRAM 写口（1 = 不要发起新的写 burst）。
    //   缩略图页生成引擎在消隐期自己发 128bit 写组，这段时间 App_wr_* 被顶层
    //   mux 切给引擎。CDC 必须整体停手：它一个 burst 是连续 64 拍，
    //   中途被抢会丢掉那几十个字（那一帧就永久缺一块，且没有任何错误码）。
    //   只挡住"启动"即可 —— 已经在跑的 burst 会自然跑完（≤64 拍）。
    input  wire                  dst_wr_hold,

    // ★ 2026-10-31 新增：垂直消隐期标志（dst_clk 域，来自 media_frame_reader 的
    //   blanking = 显示侧连续 SCRUB_IDLE_MIN 拍没有像素消费）。
    //   目的：把"加载写"挪到消隐期做，活动视频期间只让显示读占用总线。
    //   背景（实测）：第 1 张显示出来、后台还在加载第 2~4 张的那约 1 秒里，
    //   画面像被左右拉扯/拉伸（只放一张图则不出现），诊断码 ERR:E3 证明
    //   读侧 FIFO 确实欠载过。写侧只占 0.8% 带宽，但它的 wr_intent 长时间为高，
    //   读侧的 frame_fifo_read 就不敢预取 —— 显示被饿。改成分时即可。
    input  wire                  dst_blanking,

    // ★ 同一批改动：真正的写命令窗口（= fifo_rd_valid），给读侧做"窄避让"用。
    output wire                  dst_wr_active
);
    // 等消隐期的计时：一帧 16.7 ms，兜底阈值必须比一帧更长（2^22 拍 @125MHz ≈ 33.5 ms）。
    // 只有"连续两帧都没等到消隐期"才退化成"随时可写"，保证不会死锁。
    reg  [22:0] blank_wait;
    wire        blank_ok = dst_blanking || blank_wait[22];

    // ★ 2026-10-21 四次修正（针对"加载期间随机横线"）：
    //   现象：只有按下切换、写通道在搬数据时才出现随机横条；加载完立刻恢复。
    //   说明读的是已写好的旧帧（BANK0），数据本身没问题，是读写争用导致读出错误。
    //   写通道的实际需求极低（SPI 25MHz → 约 1M 字/s，仅占 125M 总线的 0.8%），
    //   完全没必要抢总线。因此把策略改成"极低占空比 + 长隔离"：
    //     · 单次 burst 缩到 64 字（0.5μs），一次占用时间减半，出错窗口更小；
    //     · 每个 burst 后强制让位 256 拍（2μs），把总线让回给读侧；
    //     · 启动前要求读侧连续空闲 9 拍，给读写换向留足余量；
    //     · burst 进行中若读侧突然开 burst，写侧就地暂停（见下方 burst 分支），
    //       宁可写慢也不与读撞车。
    //   写一帧 307200 字 ≈ 4800 个 burst × 320 拍 ≈ 12ms，而 SD 读一帧要 1s 量级，
    //   这点开销对加载时间毫无影响。
    localparam integer YIELD_CYCLES       = 512;  // 每个 burst 后强制让位拍数
    // ★ 写后静默期：最后一个写命令发出后，继续对读侧保持 busy 这么多拍。
    //   原因：SDRAM 写完最后一个字后需要 tWR(写恢复) + tRP(预充电) + tRCD(行激活)
    //   才能接受读命令，合计约 6~10 拍 @125MHz。原设计里 burst_active 一落
    //   dst_wr_busy 立刻变 0，读侧下一拍就发读命令，距最后一个写命令仅 1~2 拍，
    //   必然踩在写恢复窗口里 → 读到错误数据（画面横条）。
    //   这正是"改让位长短完全无效"的原因：让位是从 burst 开始就计数的事后休息，
    //   并没有覆盖"写刚结束"那个瞬间。
    //   取 24 拍 ≈ 192ns，对 tWR+tRP+tRCD 有 2 倍以上余量。
    //   ★ 拍数由模块参数 GUARD_CYCLES 控制（顶层可配），
    //     设成 0 即可复现"修复前"的横条现象，用于录制对比素材。
    localparam integer STARVE_CYCLES      = 96;   // 写侧饥饿多久后开始"预告抢占总线"
    localparam [7:0]   STARVE_LIMIT       = STARVE_CYCLES[7:0];
    localparam integer COMMAND_WIDTH      = ADDR_WIDTH + 36;   // addr + mask + data
    localparam integer BURST_COUNT_WIDTH  = $clog2(BURST_SIZE);
    localparam integer BLOCK_PTR_WIDTH    = FIFO_ADDR_WIDTH - BURST_COUNT_WIDTH + 1;
    localparam integer BR_W               = $clog2(BURST_SIZE + 1);  // burst_remaining 位宽

    wire [FIFO_ADDR_WIDTH:0] fifo_rd_level;

    wire [COMMAND_WIDTH-1:0] fifo_rd_data;
    wire fifo_rd_valid;
    wire fifo_empty;
    reg  fifo_rd_en;
    reg  burst_active;
    reg  [$clog2(BURST_SIZE+1)-1:0] burst_remaining;

    // ★ 2026-10-27：burst 实际发多少字，按 FIFO 里**真实可用**的字数取
    //
    //   正常情况（一个整块 64 字）：rd_level >= BURST_SIZE → 发 64 字，和原来一样。
    //   异常情况（帧启动收尾补出来的残留块，只有 k<64 字）：如果还硬发 64 字，
    //   burst_active 会在第 k+1 个字那里停下来等数据，而 dst_wr_busy 含
    //   burst_active → **读通道被按死**，直到下一帧的头几个像素到来才解开
    //   （而下一帧要先重扫 SD 卡，实机是几百毫秒到一秒）→ 表现为"卡住不动"。
    //   改为按实际字数发，残留块当拍就能收尾，读通道不受影响。
    //
    //   为什么不用 rd_level 直接判：block_pending 和 rd_level 用的是同一组
    //   两级同步（wr_gray_sync2），block_pending 一成立 rd_level 必然 >= 64，
    //   所以正常块绝不会被误截短；这里再兜一个"读成 0 也给 1 字"，防止
    //   underflow 把 burst_remaining 打成全 1。
    wire [BR_W-1:0] burst_len =
        (fifo_rd_level >= BURST_SIZE) ? BURST_SIZE[BR_W-1:0] :
        (fifo_rd_level == 0)          ? {{(BR_W-1){1'b0}}, 1'b1} :
                                        fifo_rd_level[BR_W-1:0];
    reg  [9:0] yield_cnt;
    reg  [7:0] wait_cnt;
    reg        wr_intent;          // 已向读侧预告"要占用总线"（寄存）
    reg  [3:0] rd_quiet;           // 读侧连续空闲拍数（启动前静默窗口）
    reg  [4:0] wr_guard;           // 写后静默期倒计数（>0 时挡住读侧）

    reg [BURST_COUNT_WIDTH-1:0] src_burst_word_count;
    reg                         frame_close_req;   // 帧启动时"把残留不满块补成整块"的请求
    reg [BLOCK_PTR_WIDTH-1:0]   src_block_binary;
    reg [BLOCK_PTR_WIDTH-1:0]   src_block_gray;
    reg [BLOCK_PTR_WIDTH-1:0]   src_block_gray_sync1;
    reg [BLOCK_PTR_WIDTH-1:0]   src_block_gray_sync2;
    reg [BLOCK_PTR_WIDTH-1:0]   dst_block_binary;
    reg [BLOCK_PTR_WIDTH-1:0]   dst_block_gray;

    wire src_wr_ack;                         // FIFO 真正接收了这一笔
    wire src_wr_reject;                      // FIFO 真正拒收了这一笔（= 丢字）

    // ★ 注意必须用 wr_ack（真正被接收），不能用 !src_full：
    //   src_full 现在只是"提前 2 个字"的反压预告，它为高时写照样会被收下。
    //   早先把块计数挂在 !src_full 上，结果块计数严重少记 →
    //   block_pending 提前归零 → 目的侧以为没活了 → 剩下的字全卡在 FIFO 里。
    wire src_accept   = src_wr_ack;
    wire block_pending = src_block_gray_sync2 != dst_block_gray;

    // 有整块待搬 + 让位窗口已过 = 想占用总线
    wire want_burst = block_pending && (yield_cnt == 10'd0);

    // 【2026-10-21 三次修正】原来 can_start 直接组合进 dst_wr_busy：
    //     can_start = ... && !dst_rd_busy
    //     dst_wr_busy = burst_active || can_start || ...
    // 这条链是
    //     读侧 state → O_rd_busy → dst_rd_busy → can_start → dst_wr_busy
    //     → App_wr_busy → 读侧 state 的次态逻辑
    // 两级模块纯组合穿过去，落在 125MHz(8ns) 域，是最长的组合路径之一。
    // 一旦在硬件上没收敛，读侧就可能"看到 busy 还是 0"而开一轮读 burst，
    // 与写 burst 撞在同一个周期 —— App 端口不允许读写并发，撞上的那几个字
    // 就会读错/写错，表现为画面里零散的错像素（轻微花屏）。
    //
    // 现在改为"寄存预告 + 静默窗口"：
    //   ① want_burst 一旦成立就置 wr_intent（寄存输出），读侧提前 ≥1 拍看到 busy；
    //   ② 真正启动要求读侧连续 3 拍不在读 burst（rd_quiet），给总线换向留余量；
    //   ③ dst_wr_busy 全部由寄存器组成，不再穿模块。
    // ★ 2026-10-28：加 !dst_ref_vld —— 刷新窗口内不许发起新的写 burst
    //   （照官方 app_wrrd.v 的规则；burst 一旦开始不中途打断，由 IP 内部
    //    的 Sdr_ref_ack / Sdr_rw_vld 握手负责等总线空闲再刷）。
    wire can_start = wr_intent && !burst_active && !dst_rd_busy &&
                     (rd_quiet >= 4'd8) && !dst_ref_vld && !dst_wr_hold &&
                     blank_ok;   // ★ 只在垂直消隐期（或兜底超时）启动写 burst

    // 饥饿保护：读侧如果一直抢总线，写侧会永远等不到 !dst_rd_busy（死锁）。
    // 等够 STARVE_CYCLES 后把 dst_wr_busy 提前拉高（只预告、不抢占），
    // 读侧在本轮 burst 结束时看到 busy 高就不会再发下一轮，写完自然轮到读。
    // 注意必须排除 yield_cnt != 0 的窗口：让位期的作用就是"把总线还给读侧"，
    // 这段时间 busy 必须为 0，否则让位形同虚设（旧版就在这里自相矛盾）。
    wire starved = block_pending && !burst_active && (yield_cnt == 10'd0) &&
                   (wait_cnt >= STARVE_LIMIT);

    // 最后一个写字的 FIFO 读（与下方 burst 分支的清零条件完全一致）
    wire wr_burst_last = burst_active && (burst_remaining == 1) &&
                         !dst_rd_busy && !fifo_empty;

    // 送给读侧的 busy：burst 进行中、已登记占用意图、饥饿预告、
    // ★ 写后静默期（wr_guard）—— 覆盖 SDRAM 的 tWR+tRP+tRCD，
    // 外加 fifo_rd_valid —— burst_active 在最后一个字之前就清零了，
    // 若只看 burst_active，最后一个写脉冲会和读侧的新 burst 撞上（实测重叠）。
    // 必须把"最后一个字正在发"也算进 busy。以上全部来自寄存器，无组合穿模块。
    assign dst_wr_busy = burst_active || wr_intent || starved || fifo_rd_valid ||
                         (wr_guard != 5'd0);

    async_command_fifo #(
        .DATA_WIDTH (COMMAND_WIDTH),
        .ADDR_WIDTH (FIFO_ADDR_WIDTH)
    ) u_fifo (
        .wr_clk   (src_clk),
        .wr_rst_n (src_rst_n),
        .wr_en    (src_wr_en),
        .wr_data  ({src_wr_addr, src_wr_mask, src_wr_data}),
        .wr_full  (src_full),
        .wr_ack   (src_wr_ack),
        .wr_reject(src_wr_reject),
        .rd_clk   (dst_clk),
        .rd_rst_n (dst_rst_n),
        .rd_en    (fifo_rd_en),
        .rd_data  (fifo_rd_data),
        .rd_valid (fifo_rd_valid),
        .rd_empty (fifo_empty),
        .rd_level (fifo_rd_level)
    );

    // ------------------------------------------------------------------
    // 源侧：每写满 BURST_SIZE 个字，推进一个 block 计数（格雷码送给目的侧）
    //
    // ★★ 2026-10-27 修：ERR:0x38「切不了图」永久卡死的真根因
    //
    //   目的侧只按**整块**发 burst，而 dst_idle（= 整帧真正写进 SDRAM）
    //   的判据里含 `!block_pending`。也就是说：只要源侧最后有一个
    //   1~(BURST_SIZE-1) 字的"不满块"，它就永远发不出去 →
    //   block_pending 永远为 1 → dst_idle 永远不拉高 →
    //   media_write_commit 的 frame_write_toggle 永远不翻 →
    //   sd_card_bmp 等不到写完成 → 500ms 后报 ERR_UPSTREAM(0x38)。
    //
    //   最坑的是：**loader 自己完全不报错**（它已经把 307200 个字都交给
    //   CDC 了，pixels_received/pixels_written 都对，还会正常 load_done），
    //   所以数码管上看不到任何 0x4x，只有笼统的 0x38；而 recover_state
    //   反复 RESCAN/RETRY 会不停重写同一张图（V_FLIP 自下而上写 → 看起来
    //   就是"从下往上刷一屏"），最后停在原图 = 现象完全吻合。
    //
    //   触发条件（实机成立）：上一帧中途少给了字。典型两种 ——
    //     ① CDC 掉字（`src_accept` 被 `src_full` 否掉，见下面的 FULL_RESERVE）
    //     ② 帧被 SD 层提前中断（streamer 读超时 → 解码器提前收尾）
    //   掉 k 个字后帧长不再是 BURST_SIZE 的整数倍的"已接受字数"，
    //   于是每帧都留下 k 个字的残留块，一帧都提交不了 → 永久卡死。
    //
    //   修法：下一帧启动时补一个块边界，把残留放出去。只影响"帧启动那一拍"，
    //   正常运行（帧长恰好 4800 块）走不到这条路径，零副作用。
    // ------------------------------------------------------------------
    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            src_overflow        <= 1'b0;
            src_burst_word_count<= 0;
            src_block_binary    <= 0;
            src_block_gray      <= 0;
            frame_close_req     <= 1'b0;
        end else begin
            // 注意：判据用"被真正拒收"而不是"src_full 为高"。
            //   src_full 现在只是提前 FULL_RESERVE 个字的反压预告，
            //   它拉高时写仍然会被接收（物理有空位），不能算丢字；
            //   只有 src_wr_reject 才代表这一笔真的没了。这样 0x3A 才有诊断意义。
            if (src_wr_reject)
                src_overflow <= 1'b1;

            // 请求先寄存一拍：这样它一定不会和"同拍的 src_accept"抢状态
            if (src_frame_start)
                frame_close_req <= 1'b1;

            // 收尾块：等这一拍没有新的字进来再做，保证不丢 accept
            if (frame_close_req && !src_accept) begin
                frame_close_req      <= 1'b0;
                src_burst_word_count <= 0;
                // 只有真的攒了半块才补边界；恰好整块就不动计数（否则会凭空多一块）
                if (src_burst_word_count != 0) begin
                    src_block_binary <= src_block_binary + 1'b1;
                    src_block_gray   <= ((src_block_binary + 1'b1) >> 1) ^
                                        (src_block_binary + 1'b1);
                end
            end else if (src_accept) begin
                if (src_burst_word_count == BURST_SIZE - 1) begin
                    src_burst_word_count <= 0;
                    src_block_binary     <= src_block_binary + 1'b1;
                    src_block_gray       <= ((src_block_binary + 1'b1) >> 1) ^
                                            (src_block_binary + 1'b1);
                end else begin
                    src_burst_word_count <= src_burst_word_count + 1'b1;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // 目的侧：有整块待搬时发起 128 字连续 burst
    // ------------------------------------------------------------------
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            fifo_rd_en            <= 1'b0;
            burst_active          <= 1'b0;
            burst_remaining       <= 0;
            yield_cnt             <= 10'd0;
            wait_cnt              <= 8'd0;
            wr_intent             <= 1'b0;
            rd_quiet              <= 2'd0;
            src_block_gray_sync1  <= 0;
            src_block_gray_sync2  <= 0;
            dst_block_binary      <= 0;
            dst_block_gray        <= 0;
        end else begin
            src_block_gray_sync1 <= src_block_gray;
            src_block_gray_sync2 <= src_block_gray_sync1;
            fifo_rd_en           <= 1'b0;

            // 饥饿计时：只有"想写却因为读侧占着总线写不出去"才累加。
            // 让位窗口和空闲期一律清零 —— 让位是主动让路，不算饥饿。
            if (burst_active || !block_pending || yield_cnt != 10'd0)
                wait_cnt <= 8'd0;
            else if (wait_cnt != 8'hFF)
                wait_cnt <= wait_cnt + 8'd1;

            // 读侧静默计数：连续 9 拍不在读 burst 才允许写启动（总线换向余量）
            if (dst_rd_busy)
                rd_quiet <= 4'd0;
            else if (rd_quiet != 4'd15)
                rd_quiet <= rd_quiet + 4'd1;

            // 占用意图：有整块待搬就先登记（寄存输出 → 读侧提前看到 busy）
            if (!block_pending)
                wr_intent <= 1'b0;
            else if (want_burst)
                wr_intent <= 1'b1;

            if (can_start) begin
                // 启动一个连续 burst，并在结束后预留让位窗口
                wr_intent       <= 1'b0;
                burst_active    <= 1'b1;
                burst_remaining <= burst_len;   // 正常 64 字；残留块按实际字数
                yield_cnt       <= YIELD_CYCLES[9:0];
            end else if (burst_active) begin
                // 防御 1：burst 途中若读侧竟然开了 burst（说明仲裁被时序打破），
                //         就地暂停发写命令。App_wr_en 由同一个 FIFO 读使能驱动，
                //         停一拍只是"这个字晚一拍写"，数据不丢、地址不乱序。
                // 防御 2：FIFO 真的空了就等（源侧慢于目的侧），绝不能发出
                //         "重复上一个字"的写命令，那会直接写成错误像素。
                if (!dst_rd_busy && !fifo_empty) begin
                    fifo_rd_en <= 1'b1;
                    if (burst_remaining == 1) begin
                        burst_active      <= 1'b0;
                        burst_remaining   <= 0;
                        dst_block_binary  <= dst_block_binary + 1'b1;
                        dst_block_gray    <= ((dst_block_binary + 1'b1) >> 1) ^
                                             (dst_block_binary + 1'b1);
                    end else begin
                        burst_remaining <= burst_remaining - 1'b1;
                    end
                end
            end else if (yield_cnt != 10'd0) begin
                // 让位窗口：dst_wr_busy 为 0，读侧有需求就能进 burst
                yield_cnt <= yield_cnt - 1'b1;
            end
        end
    end

    // ------------------------------------------------------------------
    // 写后静默期：最后一个写命令发出后保持 busy，让 SDRAM 完成写恢复
    // （独立 always，避免与上面的状态机抢同一寄存器）
    // guard_dis 来自 50MHz 按键扫描域，先做两级同步。
    // ------------------------------------------------------------------
    reg guard_dis_d1, guard_dis_d2;
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            guard_dis_d1 <= 1'b0;
            guard_dis_d2 <= 1'b0;
        end else begin
            guard_dis_d1 <= guard_dis;
            guard_dis_d2 <= guard_dis_d1;
        end
    end

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n)
            wr_guard <= 5'd0;
        else if (wr_burst_last && !guard_dis_d2)
            wr_guard <= GUARD_CYCLES[4:0];
        else if (wr_guard != 5'd0)
            wr_guard <= wr_guard - 5'd1;
    end

    assign dst_app_wr_en   = fifo_rd_valid;
    assign dst_app_wr_addr = fifo_rd_data[COMMAND_WIDTH-1:36];
    assign dst_app_wr_mask = fifo_rd_data[35:32];
    assign dst_app_wr_data = fifo_rd_data[31:0];

    // ★ 真正的写命令窗口：fifo_rd_valid 高表示这一拍就在往 SDRAM 写。
    //   给读侧做"窄避让"（只在这段时间让路），不再用含 wr_intent 的宽 busy。
    assign dst_wr_active = fifo_rd_valid;

    // 等消隐期的计时（只在"有整块待搬、但还没开始写"时累加；开始写就清零）
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n)
            blank_wait <= 23'd0;
        else if (burst_active || !block_pending || dst_blanking)
            blank_wait <= 23'd0;
        else if (blank_wait != 23'h7FFFFF)
            blank_wait <= blank_wait + 23'd1;
    end

    // 排空判据：FIFO 空 + 无待搬块 + 无进行中的 burst + 无在读的数据
    assign dst_idle = fifo_empty && !block_pending &&
                      !burst_active && !fifo_rd_valid;
endmodule


// =============================================================================
// async_command_fifo —— 通用异步 FIFO（格雷码指针 + 两侧两级同步）
//   数据宽度参数化（本讲用于 57 位命令），深度 2^ADDR_WIDTH。
// =============================================================================
module async_command_fifo #(
    parameter integer DATA_WIDTH = 57,
    parameter integer ADDR_WIDTH = 9
) (
    input  wire                  wr_clk,
    input  wire                  wr_rst_n,
    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_data,
    output reg                   wr_full,
    output wire                  wr_ack,      // 本次写被真正接收（块计数必须用它）
    output wire                  wr_reject,   // 本次写被真正拒绝（= 丢字），诊断用

    input  wire                  rd_clk,
    input  wire                  rd_rst_n,
    input  wire                  rd_en,
    output reg  [DATA_WIDTH-1:0] rd_data,
    output reg                   rd_valid,
    output reg                   rd_empty,
    output wire [ADDR_WIDTH:0]   rd_level
);
    localparam integer PTR_WIDTH = ADDR_WIDTH + 1;
    localparam integer DEPTH     = 1 << ADDR_WIDTH;

    // ★★ 2026-10-27 修：满判据留出 FULL_RESERVE 个空位（见下方说明）
    localparam integer FULL_RESERVE = 2;

    reg [DATA_WIDTH-1:0] memory [0:DEPTH-1];
    reg [PTR_WIDTH-1:0]  wr_binary;
    reg [PTR_WIDTH-1:0]  wr_gray;
    reg [PTR_WIDTH-1:0]  rd_binary;
    reg [PTR_WIDTH-1:0]  rd_gray;
    reg [PTR_WIDTH-1:0]  rd_gray_sync1;
    reg [PTR_WIDTH-1:0]  rd_gray_sync2;
    reg [PTR_WIDTH-1:0]  rd_binary_wr;    // 读指针在写域的二进制形式（寄存一级，见下）
    reg [PTR_WIDTH-1:0]  wr_gray_sync1;
    reg [PTR_WIDTH-1:0]  wr_gray_sync2;

    // 格雷码 → 二进制。必须在使用之前声明，且满/空判据、rd_level 都要用。
    function [PTR_WIDTH-1:0] gray_to_binary;
        input [PTR_WIDTH-1:0] value;
        integer bit_index;
        begin
            gray_to_binary[PTR_WIDTH-1] = value[PTR_WIDTH-1];
            for (bit_index = PTR_WIDTH-2; bit_index >= 0; bit_index = bit_index - 1)
                gray_to_binary[bit_index] =
                    gray_to_binary[bit_index+1] ^ value[bit_index];
        end
    endfunction

    wire rd_accept = rd_en && !rd_empty;

    // ★★ 关键：接收判据（wr_accept）和反压判据（wr_full）必须分开
    //
    //   之前两者都用 wr_full，于是"满"一旦拉高，**即使物理上还有空位也拒绝写入**，
    //   而拒绝 = 丢字 = 静默损坏。上游是寄存器输出，永远存在这一拍：
    //       t   拍 src_full=0 → 上游决定 t+1 拍写
    //       t+1 拍 wr_full 因 t 拍那一笔刚拉高 → 这一笔被拒
    //   所以只"多留几个空位"没用：任何阈值上都存在同一拍竞争。
    //
    //   现在：
    //     · wr_accept 只看**物理还有没有位置**（wr_count_now < DEPTH）；
    //     · wr_full（= 上游看到的 src_full）在 DEPTH-FULL_RESERVE 就提前拉高，
    //       纯粹是给上游的反压预告。
    //   配合 FULL_RESERVE=2：上游最晚在 n=DEPTH-2 停手，唯一在途的那一笔
    //   落到 n=DEPTH-2 时物理必然还有空位 → 永远不会被拒 → 永远不丢字。
    //
    //   wr_count_now 用的是同步过来的读指针（还多打了一拍），只会把占用算得
    //   偏大（更保守），所以不会出现"以为有空位、其实满了"的反向风险。
    wire [PTR_WIDTH-1:0] wr_count_now = wr_binary - rd_binary_wr;
    wire wr_accept = wr_en && (wr_count_now < DEPTH);
    assign wr_ack    = wr_accept;
    assign wr_reject = wr_en && !wr_accept;

    wire [PTR_WIDTH-1:0] wr_binary_next = wr_binary + wr_accept;
    wire [PTR_WIDTH-1:0] rd_binary_next = rd_binary + rd_accept;
    wire [PTR_WIDTH-1:0] wr_gray_next   = (wr_binary_next >> 1) ^ wr_binary_next;
    wire [PTR_WIDTH-1:0] rd_gray_next   = (rd_binary_next >> 1) ^ rd_binary_next;

    // 反压预告（提前 FULL_RESERVE 个字告诉上游"别再送了"）
    wire [PTR_WIDTH-1:0] wr_used      = wr_count_now + wr_accept;
    wire                 wr_full_next = (wr_used >= (DEPTH - FULL_RESERVE));
    wire                 rd_empty_next = rd_gray_next == wr_gray_sync2;

    wire [PTR_WIDTH-1:0] wr_binary_rd = gray_to_binary(wr_gray_sync2);
    assign rd_level = wr_binary_rd - rd_binary;

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_binary     <= 0;
            wr_gray       <= 0;
            rd_gray_sync1 <= 0;
            rd_gray_sync2 <= 0;
            rd_binary_wr  <= 0;
            wr_full       <= 1'b0;
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
            rd_binary_wr  <= gray_to_binary(rd_gray_sync2);
            if (wr_accept)
                memory[wr_binary[ADDR_WIDTH-1:0]] <= wr_data;
            wr_binary <= wr_binary_next;
            wr_gray   <= wr_gray_next;
            wr_full   <= wr_full_next;
        end
    end

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_binary     <= 0;
            rd_gray       <= 0;
            wr_gray_sync1 <= 0;
            wr_gray_sync2 <= 0;
            rd_data       <= 0;
            rd_valid      <= 1'b0;
            rd_empty      <= 1'b1;
        end else begin
            wr_gray_sync1 <= wr_gray;
            wr_gray_sync2 <= wr_gray_sync1;
            rd_valid      <= rd_accept;
            if (rd_accept)
                rd_data <= memory[rd_binary[ADDR_WIDTH-1:0]];
            rd_binary <= rd_binary_next;
            rd_gray   <= rd_gray_next;
            rd_empty  <= rd_empty_next;
        end
    end
endmodule
