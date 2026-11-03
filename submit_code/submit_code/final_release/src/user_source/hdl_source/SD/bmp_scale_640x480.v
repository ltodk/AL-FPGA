`timescale 1ns/1ps
// =============================================================================
// bmp_scale_640x480 —— 写侧最近邻缩放器（第 13 讲 · 图片缩放适配）
//
// 为什么缩放放在"写侧"而不是"读侧"
// -----------------------------------------------------------------------------
// 读侧（media_frame_reader）是"一次读满 307200 字进 512 深 FIFO，显示侧按 de
// 顺序弹出"的流式结构。若源图比 640×480 大，每显示 1 个像素就要跳过若干源像素，
// 但像素时钟 25MHz 下 FIFO 每拍只能弹出 1 个字 —— 缩小倍数 > 1 时根本追不上，
// 必须把异步 FIFO 的读侧整体搬到 125MHz 再跨一次域。那是在已经稳定了 6 讲的
// 链路上动刀，风险远大于收益。
//
// 写侧（sd_card_clk 100MHz，SD 卡 SPI 每个像素间隔约 96 拍）时间余量极大：
// 用"源驱动累加器"抽样成恰好 640×480，SDRAM 里存的永远是标准帧。于是
// media_frame_loader / media_frame_reader / write_commit / 转场 / 亮度 / 波形
// 全部一行都不用改，回归风险接近零。
//
// ★★ 1:1 直通旁路（本模块最重要的一个设计决定）
// -----------------------------------------------------------------------------
// 640×480 源图不经过行缓冲，out_valid/out_rgb 直接等于 in_valid/in_rgb，
// 与第 12 讲（缩放器还不存在时）的像素流**逐拍完全一致**，连 out_ready 都不看
// （第 12 讲本来就没有反压，实测能跑，说明 SPI 每像素约 96 拍的速率下 loader
// 的 512 深 FIFO 永远不会被冲爆）。
//
// 为什么必须这么干：行缓冲一旦写满，读侧会把 640 个像素以"1 拍 1 个"连发灌给
// media_frame_loader；而第 12 讲以前，loader 收到的是解码器吐出的一拍一个、
// 间隔约 96 拍的平滑流。两种流的**平均速率一样**，但瞬时形状完全不同 —— 前者是
// 每行一次 640 字的写突发，后者是每 12288 拍一次 128 字的写突发。曾尝试在输出
// 侧加"节拍器"（每 8 拍才放行 1 个像素）来削峰，代价是整体慢 8 倍，而且会把
// 读侧拖到追不上写侧（tb 的 C5 用例直接暴露：行缓冲两个半区全满、整行前缀被丢，
// 每行横向错位 58 个源像素）。**削峰不能靠降速，只能靠根本不制造尖峰。**
// 于是：1:1 就旁路（绝大多数素材都是 640×480，零回归风险），
//       非 1:1 才走行缓冲（这条路本来就是要缩放，慢一点无所谓）。
//
// ★ 判定由上层端口 mode_1to1 给出，**必须是一根寄存器输出**。不要在模块内拿
//   src_width/src_height 现场比较：byp_eff 会退化成
//   「解码器宽度寄存器 → 16 位比较 → 2:1 mux」三到四级组合逻辑，而它又是
//   out_valid 的选择端，等于直接串进 media_frame_loader 的 FIFO 使能链。
//   那条链是全工程最紧的（11 级、net 占 66%），实测一加就让 setup 从
//   +0.107ns 掉到 -0.543ns（78 个失败端点）。改成寄存器输入后只剩 1 级 LUT5。
//
// ★★★ 输出握手的特殊契约（第 13 讲 B 轮实机 ERR:0x42 之后才彻底搞清）
// -----------------------------------------------------------------------------
// 下游 media_frame_loader 的 pixel_valid/pixel_ready **不是标准 AXI-Stream**：
//   assign push_allowed = pixel_ready && pixel_valid;
//   else if (pixel_valid && !push_allowed) fail_load(ERR_FIFO_OVERFLOW);  // 0x42
// 它把"上游端出了数据但我收不下"直接判为 FIFO 溢出故障，**一个周期就 abort 整个
// load**。所以上游必须做到：**ready 为低的每一拍，valid 都必须是 0**。
// 标准 AXI-Stream 那种"valid 拉高后握到 ready 为止"在这里是致命的。
//
// 本模块因此把 out_ready 并进 out_valid：
//   out_valid = rd_active && rd_primed && out_ready;   // 非旁路
// 组合不成环 —— loader 的 pixel_ready 只由 FIFO 计数 / burst / out_full 决定，
// 不看 pixel_valid。
//
// 这个坑之所以能躲过 12 讲：解码器时代 pixel_valid 的节拍是 SPI 每像素约 96 拍，
// 512 深 FIFO 永远填不满，ready 恒为 1，两种语义完全等价。一旦换成写侧缩放器，
// 放大工况（如 320×240 → 640×480）输出是源速率的 4 倍，FIFO 会被填到 512，
// ready 出现低谷 → 每帧都可能撞上一拍 → 实机症状是"上电后一分多钟切不了图、
// 闪 ERR:42、偶尔 0x38、音频 POS 卡在 4/6、坏图残留上一帧（SDRAM 复位不清零）"。
//
// 算法（源驱动累加器，同时天然支持放大与缩小）
// -----------------------------------------------------------------------------
// 水平：每来 1 个源像素 acc_x += DST_W；只要 acc_x >= src_w 就往行缓冲写一个
//       像素并 acc_x -= src_w。★ 判定必须每拍都做（不只看 in_valid 那拍），
//       否则放大倍数 > 1 时一个源像素的份额写不完（320→640 需要连写 2 拍）。
//         src_w = 640  → 640 个源像素恰好写 640 个（1:1，实际走旁路）
//         src_w = 1280 → 每 2 个源像素写 1 个            → 640 个
//         src_w = 320  → 每 1 个源像素连写 2 拍          → 640 个
//       无论宽多少，行缓冲写满 DST_W 就是一行结束，同时 acc_x 归零（逐行独立，
//       与理论公式 sx = floor(((dx+1)*src_w-1)/DST_W) 严格等价）。
//       数学上 sum(每行写 DST_W 个) 与"源图每行 src_w 个"恰好对齐：
//       src_w × DST_W = DST_W × src_w，所以第 DST_W 次写必然落在该源行的最后一个
//       像素上，行相位不会逐行漂移（已用 643 / 800 / 1024 / 1920 逐一验算）。
// 垂直：每写满一行 acc_y += DST_H（记下"这一行贡献了多少目标行"）。
//       输出侧拿到一个满行后看 acc_y：
//         acc_y <  src_h  → 这一行整行丢弃（缩小），立刻释放缓冲
//         acc_y >= src_h  → 输出一遍，acc_y -= src_h；若仍 >= src_h 就再输出一遍
//                           （放大 = 重复整行），直到不足一行才释放
//       1080 行源 → 480 行目标：每 2.25 行输出 1 行
//       240 行源  → 480 行目标：每行输出 2 遍
//
// 为什么要行缓冲，而且必须是 ping-pong 双缓冲
// -----------------------------------------------------------------------------
// 垂直放大要把同一行输出多次，而源像素早已流过，必须缓存。同时 SD 卡文件流
// 不能暂停 —— 收下一行的同时必须能输出上一行，所以是双缓冲：
//   line_mem 共 2*DST_W 个 24bit 字。★ 半区基址是 sel*DST_W（算术加法），
//   并不是 {sel, cnt} 拼位 —— 640 不是 2 的幂，拼位会隐含 stride=1024，
//   第二半区直接越界写到 1280 以上的不存在地址（本模块第一版就踩了这个坑，
//   现象是"第 0 行全对、之后全 X"）。
//   单写口 + 单读口 → 标准双口 BRAM 模板（约 4 个 bram9k）。
//
// 出错时的行为
// -----------------------------------------------------------------------------
// 若源图太小（如 128×96 放大 5 倍）导致输出长期跟不上，两个半区都被占满，
// 这时置 overflow 并丢弃该份像素（累加器照常扣减，后面的抽样不会错位）。
// ★ 但"丢前缀"会让该行整体横向偏移，肉眼就是模糊 —— 所以 overflow 一旦置位，
// 数据就不可信了。下游 media_frame_loader 会因像素数不足报 SHORT_FRAME(0x45)，
// 不会静默出花屏。安全条件：源图 W×H 不低于约 640×480/18 ≈ 17000 像素。
// =============================================================================
module bmp_scale_640x480 #(
    parameter integer DST_W = 640,
    parameter integer DST_H = 480
)(
    input  wire        clk,
    input  wire        rst,             // 高有效（sd_card_clk 域）
    input  wire        start,           // 加载发起脉冲：清空所有状态
    input  wire [15:0] src_width,       // 源图宽（BMP 头解析结果）
    input  wire [15:0] src_height,      // 源图高
    // 1 = 源图恰好就是 640×480，整帧直通、完全不碰行缓冲（见模块头 ★★ 段）
    // ★ 必须由上层用**寄存器**给出（比 src_width/src_height 晚一拍、整帧恒定），
    //   不在这里用 src_width/src_height 现场比：那样 byp_eff 会变成
    //   "解码器宽度寄存器 → 16 位比较 → 2:1 mux" 三到四级组合逻辑，直接串进
    //   out_valid → media_frame_loader 的 FIFO 使能链。那条链本来就是全工程最紧的
    //   （11 级、net 占 66%），一加就是 setup -0.5ns。上层给寄存器则只剩 1 级 LUT5。
    input  wire        mode_1to1,

    // ---- 源像素流（恰好 src_width × src_height 个，行 padding 已由上层去掉）----
    input  wire        in_valid,
    input  wire [23:0] in_rgb,

    // ---- 下游能否接收（接 media_frame_loader 的 pixel_ready）----
    input  wire        out_ready,

    // ---- 缩放结果（恰好 DST_W × DST_H 个）----
    // ★ out_valid/out_rgb 是组合输出：见下面读侧的握手说明。
    //   若把它写成寄存器（本拍看 ready、下一拍才送 valid），反压时下游那一拍
    //   可能没接，像素就永久丢了 —— tb_bmp_scaler 的 C5 用例专门打这个点。
    output wire        out_valid,
    output wire [23:0] out_rgb,

    // ---- 下游实际收下的那个像素的指示（1:1 旁路时下游没有反压权，恒等于 valid）----
    // ★ 必须由本模块给出，不能让上层自己拼 out_valid && out_ready：
    //   旁路模式下 out_ready 可能是 0 而像素照样要送出去（第 12 讲没有反压），
    //   上层若按 && out_ready 计数就会少算，frame_commit 永远不来 → 卡死。
    output wire        out_fire,

    // ---- 诊断 ----
    output reg         overflow         // 1 = 出现过丢像素（行缓冲不够）
);
    // ------------------------------------------------------------------------
    // 行缓冲：2 × DST_W × 24bit，单写口 + 单读口
    // 地址 = 半区基址(0 或 DST_W) + 区内偏移(0..DST_W-1)
    // ------------------------------------------------------------------------
    localparam integer ADDR_W    = 11;                  // 2*DST_W=1280 → 11 位
    localparam [10:0]  HALF_BASE = DST_W;               // 第二半区基址 = 640
    localparam [15:0]  DST_W16   = DST_W;               // 常数位宽收敛，便于 16 位加
    localparam [15:0]  DST_H16   = DST_H;

    // ★ 多留 1 个槽位：读侧地址跑在输出前面一拍（见读侧说明），输出最后一个
    //   像素时地址会被预取到 base+DST_W，那个位置的数据不用，但地址必须合法。
    reg [23:0] line_mem [0:2*DST_W];

    reg               wr_en;
    reg [ADDR_W-1:0]  wr_addr;
    reg [23:0]        wr_data;

    always @(posedge clk) begin
        if (wr_en)
            line_mem[wr_addr] <= wr_data;
    end

    reg [ADDR_W-1:0]  rd_addr;
    reg [23:0]        rd_data;

    // ------------------------------------------------------------------------
    // 状态寄存器
    // ------------------------------------------------------------------------
    reg        cfg_valid;       // 已锁存源尺寸
    reg [15:0] sw;              // 锁存的源宽
    reg [15:0] sh;              // 锁存的源高

    reg [15:0] acc_x;           // 水平累加器（逐行归零）
    reg [15:0] acc_y;           // 垂直累加器（只由"行写满"更新）
    reg [23:0] hold_rgb;        // 上一个源像素（放大时要重复输出它）

    reg [10:0] wr_cnt;          // 行缓冲写指针 0..DST_W-1
    reg        wr_sel;          // 正在写的半区
    reg [1:0]  buf_full;        // 各半区是否已写满待读

    reg [10:0] rd_cnt;          // 行缓冲读指针
    reg        rd_sel;          // 正在读的半区
    reg        rd_active;       // 正在输出某个半区
    reg        rd_primed;       // 同步 RAM 首拍预取标志（只在本遍第一个像素处等 1 拍）

    // ------------------------------------------------------------------------
    // 1:1 直通旁路（见模块头 ★★ 段）
    //   判定由上层寄存器给出，这里不做任何比较，byp_eff 就是一根寄存器输出。
    // ------------------------------------------------------------------------
    wire        cfg_eff = cfg_valid || in_valid;
    wire [15:0] sw_eff  = cfg_valid ? sw : src_width;
    wire [15:0] sh_eff  = cfg_valid ? sh : src_height;
    wire        byp_eff = mode_1to1;

    // ------------------------------------------------------------------------
    // 水平累加器
    // ★ 关键 1：第一个源像素到达那一拍 cfg_valid 还是 0（锁存要下一拍生效），
    //   若只用 cfg_valid 判断，第一个像素的抽样份额会被整个丢掉，
    //   导致后续所有像素整体偏移一个。所以用组合旁路 cfg_eff / sw_eff / sh_eff。
    // ★ 关键 2：x_hit 必须每拍都判定（acc 只在有源像素那拍加 DST_W），
    //   否则放大倍数 > 1 时一个源像素该写多份却只写 1 份，行永远填不满。
    // ------------------------------------------------------------------------
    wire [15:0] acc_x_inc = in_valid ? (acc_x + DST_W16) : acc_x;
    wire        x_hit     = cfg_eff && (acc_x_inc >= sw_eff);

    wire [ADDR_W-1:0] wr_base = wr_sel ? HALF_BASE : {ADDR_W{1'b0}};
    wire [ADDR_W-1:0] rd_base = rd_sel ? HALF_BASE : {ADDR_W{1'b0}};
    // 释放当前半区后，读指针落到另一半区的首地址（下一行就绪时直接可用）
    wire [ADDR_W-1:0] rd_base_nxt = rd_sel ? {ADDR_W{1'b0}} : HALF_BASE;
    wire              wr_free = !buf_full[wr_sel];

    // 当前读半区是否有一整行待输出
    wire        row_have   = !rd_active && buf_full[rd_sel];

    // ------------------------------------------------------------------------
    // 输出握手
    //   rd_primed 已经保证 rd_data = line_mem[rd_addr]（当前要输出的像素），
    //   因此把 rd_data 直接送出去即可，不需要再打一拍。
    //
    // ★★★ 第 13 讲 B 轮实机 ERR:0x42 的根因就在这一行（10-26 定位）
    //
    //   下游 media_frame_loader 的 pixel_ready/pixel_valid **不是标准 AXI-Stream**：
    //     assign push_allowed = pixel_ready && pixel_valid;
    //     else if (pixel_valid && !push_allowed) fail_load(ERR_FIFO_OVERFLOW);
    //   它把"valid 为高但自己收不下"直接当成 FIFO 溢出故障，**一个周期就 abort
    //   整个 load**。也就是说它要求上游永远不要在 ready 为低时端出数据。
    //
    //   原来的写法 `out_valid = rd_active && rd_primed` 是标准 AXI-Stream 语义
    //   （valid 拉高后一直握到 ready），本身没错，但与上面那条规则正好冲突：
    //   只要出现**一拍** out_ready=0，loader 立刻报 0x42 退出。
    //
    //   为什么老设计从没炸过：第 12 讲之前 pixel_valid 来自解码器，速率是
    //   SPI 节拍（每像素约 96 拍），512 深 FIFO 永远填不满，ready 一直为 1，
    //   两种语义恰好等价。换成写侧缩放器后放大工况输出 4 倍像素，FIFO 会被
    //   填到 512，ready 出现低谷 —— 于是每帧都可能撞上一拍，实机表现为
    //   "上电后一分多钟切不了图、闪 ERR:42、偶尔 0x38、POS 卡在 4/6 秒、
    //    坏图残留上一帧内容（SDRAM 复位不清零）"。
    //
    //   所以这里**必须**把 ready 并进 valid：保证 out_valid=1 时下游一定收得下。
    //   组合不成环：loader 的 pixel_ready 只由 FIFO 计数/burst/out_full 决定，
    //   不依赖 pixel_valid。
    //   旁路模式保持第 12 讲原语义（in_valid 直接透传、不看 ready）—— 那条路
    //   的节拍天然合规，A 轮实机已验证，不动它。
    // ------------------------------------------------------------------------
    assign out_valid = byp_eff ? in_valid : (rd_active && rd_primed && out_ready);
    assign out_rgb   = byp_eff ? in_rgb   : rd_data;
    // 成交判据：非旁路时 out_valid 已并入 out_ready（见上），旁路时下游没有否决权
    assign out_fire  = out_valid;

    // ★ 读数据必须能在下游暂停时冻结：
    //   读地址刻意跑在输出前面一拍（见读侧说明），如果这里无条件每拍装载，
    //   一旦 out_ready 拉低等待，rd_data 就会被"下一个像素"覆盖，成交时送出的
    //   是后一个像素 —— 现象是每个输出行的首像素取到源像素 2 而不是 1，且
    //   只在有反压时出现（无背压时暂停恰好为零拍，测不出来）。
    //   判据 out_valid && !out_ready 就是"已经握着有效像素但下游不收"。
    wire out_stall = rd_active && rd_primed && !out_ready;

    always @(posedge clk) begin
        if (!out_stall)
            rd_data <= line_mem[rd_addr];
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cfg_valid <= 1'b0;
            sw        <= 16'd0;
            sh        <= 16'd0;
            acc_x     <= 16'd0;
            acc_y     <= 16'd0;
            hold_rgb  <= 24'd0;
            wr_cnt    <= 11'd0;
            wr_sel    <= 1'b0;
            buf_full  <= 2'b00;
            rd_cnt    <= 11'd0;
            rd_sel    <= 1'b0;
            rd_active <= 1'b0;
            rd_primed <= 1'b0;
            rd_addr   <= {ADDR_W{1'b0}};
            overflow  <= 1'b0;
            wr_en     <= 1'b0;
            wr_addr   <= {ADDR_W{1'b0}};
            wr_data   <= 24'd0;
        end else if (start) begin
            cfg_valid <= 1'b0;
            acc_x     <= 16'd0;
            acc_y     <= 16'd0;
            wr_cnt    <= 11'd0;
            wr_sel    <= 1'b0;
            buf_full  <= 2'b00;
            rd_cnt    <= 11'd0;
            rd_sel    <= 1'b0;
            rd_active <= 1'b0;
            rd_primed <= 1'b0;
            rd_addr   <= {ADDR_W{1'b0}};
            overflow  <= 1'b0;
            wr_en     <= 1'b0;
        end else begin
            wr_en <= 1'b0;

            // ---- 尺寸锁存（旁路模式下用不到，但锁了不影响任何东西）----
            if (in_valid && !cfg_valid) begin
                // 第一个源像素：BMP 头早已解析完（像素区从 offset>=54 起）
                sw        <= src_width;
                sh        <= src_height;
                cfg_valid <= 1'b1;
            end

            // =============================================================
            // 以下两段只在"需要真正缩放"时工作
            // =============================================================
            if (!byp_eff) begin
                // =========================================================
                // 写侧：水平缩放 → 行缓冲
                // =========================================================
                if (in_valid)
                    hold_rgb <= in_rgb;

                if (x_hit) begin
                    // 该往行缓冲写一个像素了。本拍有源像素就用它，否则重复上一个
                    // （放大时同一个源像素要连写多拍）。
                    // ★ 不能用 `in_valid && cfg_valid`：第一个像素那拍 cfg_valid 还是 0，
                    //   会退化成写 hold_rgb（未初始化），把整幅图推偏一个像素。
                    if (wr_free) begin
                        wr_en   <= 1'b1;
                        wr_addr <= wr_base + wr_cnt;
                        wr_data <= in_valid ? in_rgb : hold_rgb;

                        acc_x  <= acc_x_inc - sw_eff;
                        wr_cnt <= wr_cnt + 11'd1;

                        if (wr_cnt == DST_W-1) begin
                            // 一行写满：登记"这一行贡献的垂直份额"，切到另一半区
                            buf_full[wr_sel] <= 1'b1;
                            acc_y            <= acc_y + DST_H16;
                            wr_sel           <= ~wr_sel;
                            wr_cnt           <= 11'd0;
                            acc_x            <= 16'd0;   // 每行水平累加器归零
                        end
                    end else begin
                        // =====================================================
                        // 两个半区都占满：这一份写不进去。
                        //
                        // ★★★ 第 13 讲 B 轮实机 ERR:0x38 的根因就在这里（10-26 定位）
                        //   x_hit 的次数是**固定**的：每个源行恰好 DST_W 次
                        //   （acc_x 每源像素加 DST_W、每次 x_hit 减 sw_eff，
                        //    累计 DST_W*sw / sw = DST_W，一次不多一次不少）。
                        //   如果 DROP 时 wr_cnt 不动，这一行就永远凑不满 DST_W 个
                        //   → buf_full 永不置位 → 这一行永不发布 → 整帧输出远少于
                        //   DST_W*DST_H → media_write_commit 数不到第 307200 个像素
                        //   → frame_commit 不触发 → loader 的 load_done 不来
                        //   → sd_card_bmp 的 500ms 写等待超时 → **ERR:0x38**。
                        //
                        //   实测（tb_bmp_scaler C8，320×240 + 暴力反压）：
                        //     drop=129280 → pub 只有 38/240 行、out 只有 48640/307200。
                        //   这正是实机 B 轮"切图时不时报 0x38、画面保持上一张"的情形；
                        //   而 A 轮走旁路不碰行缓冲，所以从不复现。
                        //
                        //   ★ 修法：DROP 也照常推进 wr_cnt / 换半区 / 登记 buf_full，
                        //     保证"每源行恰好产出 DST_W 个、总共恰好 DST_H 行"这个
                        //     不变量恒定成立。丢掉的只是那**一格**的内容（wr_en 不拉，
                        //     该地址保留半区里的旧值 —— 1 个像素的杂色），
                        //     换来的是整帧一定完整、一定不会报 0x38。
                        //
                        //   ★ 为什么可以安全地置 buf_full[wr_sel]：wr_free=0 说明这个
                        //     半区已经满且正被读侧消费，置 1 只是重申"满"；随后切到
                        //     另一半区，若那边也满则继续走本分支（wr_en 恒 0），
                        //     全程不会写坏任何正在被读的数据。
                        // =====================================================
                        overflow <= 1'b1;
                        acc_x    <= acc_x_inc - sw_eff;
                        wr_cnt   <= wr_cnt + 11'd1;

                        if (wr_cnt == DST_W-1) begin
                            buf_full[wr_sel] <= 1'b1;
                            acc_y            <= acc_y + DST_H16;
                            wr_sel           <= ~wr_sel;
                            wr_cnt           <= 11'd0;
                            acc_x            <= 16'd0;   // 每行水平累加器归零
                        end
                    end
                end else begin
                    acc_x <= acc_x_inc;
                end

                // =========================================================
                // 读侧：垂直重复输出
                //
                // ★ 同步 RAM 的坑（本模块第一版踩了两个）：
                //   ① `always @(posedge clk) rd_data <= line_mem[rd_addr];` 决定了
                //      rd_data 恒**滞后 rd_addr 一拍**。若在同一拍里既"输出 rd_data"
                //      又"把 rd_addr 推到当前像素"，输出的其实是上一拍地址的数据，
                //      整行错位一个像素（现象：每行第 0 个像素正确、之后全部滞后一格）。
                //   ② 反向也不行：若每个像素都重新预取（2 拍/像素），吞吐减半并
                //      没有额外好处。
                //   正解：地址**恒定领先输出一拍**，只在换行/换遍时插 1 拍预取：
                //       预取拍：rd_addr = base，本拍末 rd_data = line_mem[base]（第 0 个）
                //       输出拍：out_rgb = rd_data（rd_cnt 指向的像素），同时 rd_addr++
                //       → 输出第 i 个像素时 rd_addr = base+i+1，rd_data = line_mem[base+i]
                //   于是稳态 1 拍/像素。真实场景余量很大：SPI 每像素约 96 拍，
                //   最苛刻的放大 2 倍（320 宽源图）一行源像素要 320×96 = 30720 拍，
                //   而输出两遍只要 2×640 = 1280 拍。4 倍以上放大才会跟不上，那时
                //   行缓冲写满、overflow 置位，代价是丢像素而不是错位。
                // =========================================================
                if (row_have) begin
                    if (acc_y >= sh_eff) begin
                        // 这一行要输出 → 取走半区，地址指向首像素，等 1 拍预取
                        acc_y     <= acc_y - sh_eff;
                        rd_active <= 1'b1;
                        rd_primed <= 1'b0;
                        rd_cnt    <= 11'd0;
                        rd_addr   <= rd_base;
                    end else begin
                        // 这一行整行丢弃（缩小），立刻释放半区
                        buf_full[rd_sel] <= 1'b0;
                        rd_sel           <= ~rd_sel;
                        rd_addr          <= rd_base_nxt;
                    end
                end else if (rd_active) begin
                    if (!rd_primed) begin
                        // 预取拍：本拍末 rd_data 装载 line_mem[rd_base]（= 第 0 个像素），
                        // 同时把地址推到 base+1，于是下一拍起地址始终领先输出一拍。
                        rd_primed <= 1'b1;
                        rd_addr   <= rd_base + 11'd1;
                    end else if (out_ready) begin
                        // 输出拍：out_valid 本拍为 1（组合），下游 ready 即为成交；
                        // rd_data 就是当前 rd_cnt 指向的那个像素。
                        if (rd_cnt == DST_W-1) begin
                            // 这一遍输出完了
                            rd_primed <= 1'b0;
                            if (acc_y >= sh_eff) begin
                                // 垂直放大：同一行再输出一遍 → 回到首像素重新预取
                                acc_y  <= acc_y - sh_eff;
                                rd_cnt <= 11'd0;
                                rd_addr<= rd_base;
                            end else begin
                                // 本行彻底结束，释放半区
                                buf_full[rd_sel] <= 1'b0;
                                rd_sel           <= ~rd_sel;
                                rd_active        <= 1'b0;
                                rd_cnt           <= 11'd0;
                                rd_addr          <= rd_base_nxt;
                            end
                        end else begin
                            rd_cnt  <= rd_cnt + 11'd1;
                            rd_addr <= rd_addr + 11'd1;    // 地址领先输出一拍
                        end
                    end
                end
            end
        end
    end

endmodule
