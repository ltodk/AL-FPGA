`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// media_frame_reader —— 读侧帧读取器（第8讲 阶段B）
//
// 目标：替换 frame_read_write 的读侧 + video_delay，输出对齐到 1 拍的像素流。
//
// 架构（三层）：
//   ① SDRAM 读 burst 状态机 —— 直接复用已验证的 frame_fifo_read（不重写），
//      它负责 App_rd_en/App_rd_addr 连续 128 拍 burst、10 拍 rd_delay 补偿、
//      FIFO 空间判断（wrusedw < 512-128 才启动）、读写互斥避让
//      （App_wr_busy 输入 / O_rd_busy 输出，这是 A 阶段修好横条的成果，必须保留）。
//   ② 异步 FIFO rfifo_32_32_512 —— 125M(mem_clk) 写 → 25M(pix_clk) 读。
//   ③ fifo_prefetch_2 —— 两级预取，把数据提前 1 拍顶出来，
//      替代原 video_delay 的"提前 1 拍"相位。
//
// 输出 pixel_rgb/pixel_valid 已对齐到 1 拍，顶层直接用 de 做 video_x 对齐，
// 不再需要 video_delay 的 21 拍移位。
//
// 说明：frame_fifo_read 的 read_addr_0~3 用于选择 bank 基址。阶段 B 由顶层把
//       read_addr_index 对应 bank 的真实基址（index*307200）接到 read_addr_0~3。
//       这里透传这些基址给 frame_fifo_read。
// -----------------------------------------------------------------------------
module media_frame_reader #(
    parameter MEM_DATA_BITS  = 32,
    parameter ADDR_BITS      = 21,
    parameter BURST_BITS     = 9,
    parameter BURST_SIZE     = 128,
    parameter FIFO_DEPTH     = 512,
    parameter FIFO_MARGIN    = 8,
    // ---- 2026-10-28 新增：SDRAM 擦洗(scrub)参数 ----
    parameter integer SCRUB_EN      = 1,        // 1 = 开启擦洗（0 = 关闭，用于 A/B 对比）
    parameter integer SCRUB_BURST   = 128,      // 单次擦洗 burst 字数（与 BURST_SIZE 对齐）
    parameter integer SCRUB_FULL_HI = 500,      // (保留) FIFO 满度参考，仅观测用
    parameter integer SCRUB_IDLE_MIN = 2000,    // 显示侧连续不消费多少拍才算"消隐期"
    // 每次消隐期最多做多少个擦洗 burst。限制它是为了让消隐期【后段】干净地
    // 留给 frame_fifo_read 预填并稳定下来，绝不在活动视频开始前还占着总线。
    // 400 burst/帧 = 51200 字/帧 ≈ 3.1M 字/s，全扫 122.88 万字约 400ms 一遍，
    // 而数据实测能撑数分钟，余量仍然极大。
    parameter integer SCRUB_MAX_PER_FRAME = 1200,
    parameter integer SCRUB_WORDS   = 1228800,  // 擦洗覆盖范围：4 帧 = 4*307200 字

    // ---- 2026-10-29【第15讲】2×2 缩略图页生成引擎 ----
    parameter integer         TB_EN        = 1,        // 1 = 使能缩略图页生成
    parameter integer         TB_TEST_EN   = 0,        // 1 = 纯色测试图（不读源图）
    parameter integer         TB_PASSES    = 2,        // 整页生成跑几遍（覆盖瞬态错位）
    parameter [ADDR_BITS-1:0] TB_ADDR      = 21'd1228800, // 缩略图页基址
    parameter integer         TB_SRC_W     = 640,      // 源图宽
    parameter integer         TB_W         = 320,      // 页内象限宽
    parameter integer         TB_H         = 240       // 页内象限高
)(
    // ---- SDRAM 读端口（mem_clk 域）----
    input                            mem_clk,
    input                            rst,                 // mem_clk 域高有效复位
    input                            Sdr_init_done,
    input                            Sdr_init_ref_vld,
    input                            Sdr_busy,
    input                            Sdr_rd_en,           // 读数据 valid
    input  [MEM_DATA_BITS-1:0]       Sdr_rd_dout,
    output                           App_rd_en,
    output [ADDR_BITS-1:0]           App_rd_addr,

    // ---- 读写互斥握手（与 CDC 写通道对接）----
    input                            ext_wr_busy,         // 写侧忙（宽：含"想写"），擦洗让路用
    // ★ 2026-10-31 新增：写侧"真正在写"的窄窗口（= CDC 的 fifo_rd_valid）。
    //   预取避让改用它 —— 原来用宽的 ext_wr_busy，而它的 wr_intent 会长时间为高，
    //   导致 frame_fifo_read 不敢预取、显示 FIFO 被饿（实测 ERR:E3 里的 underflow）。
    input                            ext_wr_active,
    output                           app_rd_busy,         // 读侧忙，写侧避让
    // ★ 2026-10-31 新增：当前是否处于垂直消隐期（显示侧长时间不消费像素）。
    //   给 CDC 用：把加载写限制在消隐期，活动视频期间读侧独占总线。
    output                           display_blank,

    // ---- 显示侧（pix_clk 域）----
    input                            pix_clk,             // video_clk 25M
    input                            pix_rst,             // pix_clk 域高有效复位
    input                            read_req,            // 帧读请求（来自 video_timing_data）
    output                           read_req_ack,
    input  [ADDR_BITS-1:0]           read_addr_0,         // bank0 基址
    input  [ADDR_BITS-1:0]           read_addr_1,         // bank1 基址
    input  [ADDR_BITS-1:0]           read_addr_2,         // bank2 基址
    input  [ADDR_BITS-1:0]           read_addr_3,         // bank3 基址
    input  [1:0]                     read_addr_index,     // 当前显示 bank
    input  [ADDR_BITS-1:0]           read_len,            // 一帧字数 = 307200
    input                            pixel_consume,       // 显示侧消费（de 有效）
    output [23:0]                    pixel_rgb,           // 对齐后的 24 位 RGB
    output                           pixel_valid,

    // ---- 2026-10-29【第15讲】缩略图页生成引擎 ----
    input  [3:0]                     tb_image_complete,   // 四张都加载完
    input  [ADDR_BITS-1:0]           src_addr_0,          // 源图 bank0 基址（恒为 BUF0）
    input  [ADDR_BITS-1:0]           src_addr_1,
    input  [ADDR_BITS-1:0]           src_addr_2,
    input  [ADDR_BITS-1:0]           src_addr_3,
    output                           tb_wr_en,            // 页写使能（128bit 组内连发 4 拍）
    output [ADDR_BITS-1:0]           tb_wr_addr,
    output [MEM_DATA_BITS-1:0]       tb_wr_data,
    output                           tb_busy,             // 1 = 引擎占用写口/读口
    output                           tb_done,             // 粘滞：整页生成完成

    // ---- 观测 ----
    output reg                       underflow            // 读侧下溢标志
);

    // ---- frame_fifo_read 的避让输出 + FIFO 控制 ----
    wire        O_rd_busy;
    wire        fifo_aclr;
    wire [8:0]  fifo_wrusedw;   // 9 位，与 rfifo_32_32_512 的 wrusedw 一致

    // =========================================================================
    // ★ 2026-10-28【SDRAM 擦洗 / scrub】
    //
    // 背景（三天实测定性）：
    //   片内 EM638325 的前 512K 字（0x00000~0x7FFFF）读写完全正常；
    //   再往上的地址区，**没被读写过的数据会衰减**，读回来是错的（单比特错，
    //   颜色精确对应 DQ15/DQ14/DQ22），而且一旦读到就会把错值感放写回、锁死。
    //   证据：四帧身份探针的分层边界与"地址 0x80000 / 0x100000"完全吻合；
    //         平移 +0x10000 后分层边界按预测从 70.6%→49%、41.3%→20% 移动；
    //         把读侧钉在 bank0（全在 0x7FFFF 以下）后连续 15 分钟零噪点。
    //
    // 对策（本段逻辑）：
    //   SDRAM 的读是"破坏性读 + 感放回写"，**每读一次等于刷新一次**。
    //   正在显示的那个 bank 每 16.7ms 被整帧读一遍，所以永远干净；
    //   其余三个 bank 开机写完后没人碰 → 衰减。
    //   ⇒ 在**垂直消隐期**用空闲总线把 4 帧覆盖的地址区（0 ~ 4*IMAGE_WORDS）
    //      轮流读一遍，数据丢弃即可让它们保持活性。
    //
    // 为什么只在消隐期做：
    //   消隐期显示侧不消费（pixel_consume=0），FIFO 会被灌满到 511~512；
    //   用"FIFO 接近满"作为消隐期的判定，天然避开活动视频期，不会抢显示带宽；
    //   即使擦洗 burst 被帧边界（read_req → S_ACK → fifo_aclr）打断也无害，
    //   因为那时显示侧本来就不消费。
    //
    // 带宽核算：消隐 45 行 ≈ 1.1ms/帧 → 约 13 万拍 → 约 1000 个 128 字 burst
    //           → 每帧约 12.8 万字 → 全扫 122.88 万字约 157ms 一遍。
    //           数据实测能撑数分钟，157ms 的刷新间隔余量极大。
    // =========================================================================
    // （擦洗参数已移到模块头部参数表：SCRUB_EN / SCRUB_BURST / SCRUB_FULL_HI / SCRUB_WORDS）

    reg  [1:0]           scr_state;
    localparam S_SCR_IDLE = 2'd0,
               S_SCR_RUN  = 2'd1;

    reg  [ADDR_BITS-1:0] scr_addr;
    reg  [7:0]           scr_left;
    wire                 scr_bus  = (scr_state == S_SCR_RUN);

    // 擦洗覆盖范围：4 帧的全部地址（0 ~ 4*IMAGE_WORDS-1）
    localparam [ADDR_BITS-1:0] SCRUB_LIMIT = SCRUB_WORDS[ADDR_BITS-1:0];

    // 「消隐期」判据：把 pixel_consume(de) 同步到 mem_clk 域，连续多少拍不消费
    // 才算消隐。用 de 而不是用 FIFO 满度，是因为 9 位 wrusedw 在 FIFO 真满时
    // 可能回绕到 0，用它做判据不可靠（仿真已实测踩到）。
    reg  [1:0]  pc_sync;
    reg  [15:0] idle_cnt;
    always @(posedge mem_clk or posedge rst) begin
        if (rst) begin
            pc_sync  <= 2'b00;
            idle_cnt <= 16'd0;
        end else begin
            pc_sync <= {pc_sync[0], pixel_consume};
            if (pc_sync[1])
                idle_cnt <= 16'd0;
            else if (idle_cnt != 16'hFFFF)
                idle_cnt <= idle_cnt + 16'd1;
        end
    end
    wire blanking = (idle_cnt >= SCRUB_IDLE_MIN[15:0]);

    // 给 CDC 用：消隐期标志（同上；scrub 与"写只在消隐期"共用这一个判据）
    assign display_blank = blanking;

    // ---------------------------------------------------------------------
    // ★ 2026-10-28 抖动修复：擦洗必须"干净地插进"显示读的空隙
    //
    // 上一版抖动的原因（实机现象：图片高频抖动、噪点已消失）：
    //   擦洗 burst 与显示 burst 在**同一个周期**交界时，App_rd_en 被 mux 成擦洗
    //   的请求，但 frame_fifo_read 内部仍把那一拍记成自己的请求 → 显示/擦洗的
    //   归属计数各差 1 → 之后某个显示数据被误当擦洗数据丢掉 → 整帧错位 1 像素；
    //   每个帧边界错位量随机 → 看着就是高频抖动。
    //
    // 修法（两道保险 + 两个看门狗 + 一根保险丝）：
    //   ① 擦洗只在【显示侧完全没有在途请求】时启动（pd_cnt == 0），
    //      且 frame_fifo_read 已连续 4 拍不请求（ffr_quiet ≥ 4）；
    //   ② ★ 绝不用 ps_cnt 去长期挡 ffr —— 那会造成死锁（实机整屏灰）。
    //      只在 scr_bus 这一小段窗口内挡。
    //   ★ 看门狗：pd_cnt/ps_cnt 非零持续 256 拍就强制清零，防计数器卡死。
    //   ★ 保险丝：擦洗若让显示侧长时间读不到数据，永久关掉擦洗（直到复位）。
    // ---------------------------------------------------------------------
    reg [9:0]  pd_cnt;      // (保留观测，不再参与判断)
    reg [9:0]  ps_cnt;      // (保留观测，不再参与判断)
    reg [5:0]  ffr_quiet;   // ffr 连续不请求的拍数（饱和到 63）
    reg [9:0]  pd_age, ps_age;
    reg [12:0] starve_cnt;
    reg        scrub_kill;
    reg [9:0]  blk_bursts;   // 本次消隐期已做的擦洗 burst 数
    // ★ 2026-10-28 第四次修正：不再用"计数数据归属"（实机不可靠），
    //   改成纯时间窗口：
    //     · ffr 连续静默 ≥24 拍（在途数据早已排空）才启动擦洗；
    //     · 擦洗结束后再把抑制窗口延长 TAIL 拍，把最后一波返回数据也盖住；
    //     · 整个窗口内 fifo_we 被关掉、ffr 也被挡住 ⇒ 不可能混流、不可能累积误差。
    reg [7:0]  scr_tail;
    wire       scrub_window = scr_bus || (scr_tail != 8'd0);
    wire       stale_pd = (pd_age >= 10'd255);
    wire       stale_ps = (ps_age >= 10'd255);
    wire       ffr_rd_en;
    wire [ADDR_BITS-1:0] ffr_rd_addr;

    // ---- 2026-10-29【第15讲】生成引擎的仲裁信号（此处只声明，逻辑在文件末尾）----
    wire       tb_pending;    // 引擎有待做工作（四张已加载 + 消隐期）
    wire       tb_rd_win;     // 引擎占用读口（S_TB_RD，连发 8 拍）
    wire       tb_collect;    // 引擎等待读数据返回（S_TB_COLLECT）
    wire       tb_win;        // tb_rd_win | tb_collect
    wire       tb_wr_busy;    // 引擎写组在途或写恢复期
    // 读地址 mux 在下面（第 320 行附近）就要用到这两个，必须提前声明：
    // Verilog 里"先用后声明"会被当成隐式 wire，后面再显式声明就是重复定义
    // （vlog-2388 / TD 综合同样会报）。
    reg  [3:0]          tb_rn;        // 读组内第几拍 0..7
    wire [ADDR_BITS-1:0] tb_rd_addr;  // 源图读首地址（8 字对齐）

    always @(posedge mem_clk or posedge rst) begin
        if (rst)          ffr_quiet <= 6'd0;
        else if (ffr_rd_en) ffr_quiet <= 6'd0;
        else if (ffr_quiet != 6'h3F) ffr_quiet <= ffr_quiet + 6'd1;
    end

    // 擦洗结束后的抑制尾巴：把最后一批返回数据也盖住
    always @(posedge mem_clk or posedge rst) begin
        if (rst) scr_tail <= 8'd0;
        else if (scr_bus && (scr_left == 8'd1)) scr_tail <= 8'd48;
        else if (scr_tail != 8'd0)             scr_tail <= scr_tail - 8'd1;
    end

    always @(posedge mem_clk or posedge rst) begin
        if (rst) begin
            scr_state <= S_SCR_IDLE;
            scr_addr  <= {ADDR_BITS{1'b0}};
            scr_left  <= 8'd0;
        end else begin
            case (scr_state)
                S_SCR_IDLE: begin
                    // ★ 2026-10-28 第三次修正：启动条件只保留"必需的"几条。
                    //   实机教训：把 pd_cnt==0 / ps_cnt==0 也当启动条件，会导致
                    //   擦洗永远不启动（IP 的返回拍数不像仿真那样严格 1:1，
                    //   pd_cnt 可能长期非零）→ 噪点原样回来。
                    //   真正防抖动竞态的只有 ffr_quiet ≥ 4（ffr 已彻底停手），
                    //   其余条件都是"可能锁死自己"的多余约束，全部去掉。
                    if (SCRUB_EN && Sdr_init_done && !Sdr_init_ref_vld &&
                        !O_rd_busy && !ext_wr_busy && blanking &&
                        (ffr_quiet >= 6'd24) &&
                        // ★ 第15讲：缩略图页生成期间不给擦洗发新 burst（引擎优先）
                        !tb_pending && !tb_wr_busy && !tb_win &&
                        (blk_bursts < SCRUB_MAX_PER_FRAME[9:0])) begin
                        scr_state <= S_SCR_RUN;
                        scr_left  <= SCRUB_BURST[7:0];
                    end
                end
                S_SCR_RUN: begin
                    scr_addr <= scr_addr + 1'b1;
                    if (scr_addr + 1'b1 >= SCRUB_LIMIT)
                        scr_addr <= {ADDR_BITS{1'b0}};     // 扫完一轮从头再来
                    if (scr_left == 8'd1)
                        scr_state <= S_SCR_IDLE;
                    else
                        scr_left <= scr_left - 8'd1;
                end
            endcase
        end
    end

    // 擦洗窗口内挡住 ffr（wrusedw 喂"满"值），保证窗口内不会有显示请求发出
    wire [9:0] ffr_wrusedw = (scrub_window || tb_win) ? {1'b0, 9'h1FF} : {1'b0, fifo_wrusedw};

    always @(posedge mem_clk or posedge rst) begin
        if (rst) begin
            pd_cnt <= 10'd0;
            ps_cnt <= 10'd0;
        end else begin
            // ★ 看门狗：归属计数器一旦异常（实机上出现过"发出请求却收不到数据"），
            //   必须在几百拍内自愈，否则会永久误判、把显示数据当擦洗数据丢掉。
            if (stale_pd)
                pd_cnt <= 10'd0;
            else
                pd_cnt <= pd_cnt + (ffr_rd_en ? 10'd1 : 10'd0)
                                 - ((Sdr_rd_en && (pd_cnt != 10'd0)) ? 10'd1 : 10'd0);

            if (stale_ps)
                ps_cnt <= 10'd0;
            else
                ps_cnt <= ps_cnt + (scr_bus    ? 10'd1 : 10'd0)
                                 - ((Sdr_rd_en && (pd_cnt == 10'd0) && (ps_cnt != 10'd0)) ? 10'd1 : 10'd0);
        end
    end

    // 看门狗计时：非零持续超过 256 拍就强制清零
    always @(posedge mem_clk or posedge rst) begin
        if (rst) begin pd_age <= 10'd0; ps_age <= 10'd0; end
        else begin
            pd_age <= (pd_cnt == 10'd0) ? 10'd0 : (pd_age + 10'd1);
            ps_age <= (ps_cnt == 10'd0) ? 10'd0 : (ps_age + 10'd1);
        end
    end

    // 每次消隐期重置 burst 计数
    always @(posedge mem_clk or posedge rst) begin
        if (rst)            blk_bursts <= 10'd0;
        else if (!blanking) blk_bursts <= 10'd0;
        else if (scr_bus && (scr_left == 8'd1) && (blk_bursts != 10'h3FF))
                            blk_bursts <= blk_bursts + 10'd1;
    end

    // ★ 保险丝：擦洗一旦让显示侧长时间读不到数据，就永久关掉擦洗（直到复位）。
    //   判据同时要求"SDRAM 读 FIFO 已经空"——只靠 pd_cnt 会误判，
    //   因为实机上 pd_cnt 的语义不像仿真那样严格（上面已踩过这个坑）。
    always @(posedge mem_clk or posedge rst) begin
        if (rst) begin
            starve_cnt <= 13'd0;
            scrub_kill <= 1'b0;
        end else begin
            if (!blanking && (pd_cnt == 10'd0) && (fifo_wrusedw <= 9'd4)) begin
                if (starve_cnt != 13'h1FFF) starve_cnt <= starve_cnt + 13'd1;
                else                        scrub_kill <= 1'b1;
            end else begin
                starve_cnt <= 13'd0;
            end
        end
    end

    // 只有"确定是擦洗数据"时才丢；万一计数器异常（两边都 0）仍按显示数据放行，
    // 宁可多进一个字，也不要把显示数据丢掉。
    wire scr_data = Sdr_rd_en && (pd_cnt == 10'd0) && (ps_cnt != 10'd0);

    // ---- 读端口 mux：擦洗 burst 优先（它只在显示侧空闲时才启动），
    //      ★ 第15讲新增第二级：缩略图生成引擎的 8 拍连读 ----
    assign App_rd_en   = scr_bus   ? 1'b1        :
                         tb_rd_win ? 1'b1        :
                                     ffr_rd_en;
    assign App_rd_addr = scr_bus   ? scr_addr    :
                         tb_rd_win ? (tb_rd_addr + tb_rn) :
                                     ffr_rd_addr;

    // 写侧要靠 app_rd_busy 避让，擦洗/生成期间也必须算"读侧忙"，否则会撞车
    assign app_rd_busy = O_rd_busy | scr_bus | tb_win | tb_wr_busy;

    // =========================================================================
    // ① SDRAM 读 burst 状态机 —— 复用 frame_fifo_read
    //    App_wr_busy = 内部写忙（本模块无写，恒 0）| 外部 CDC 写忙
    // =========================================================================
    // App_wr_busy = 内部写忙（本模块无写，恒 0）| 外部 CDC 写忙 | 缩略图页生成写忙
    // ★ 2026-11-01 00:3x：改回**宽**避让（ext_wr_busy，含"想写"意图）。
    //   窄避让（ext_wr_active，只算真正的写命令窗口）会让 reader 在 CDC 等待静默期时就去
    //   抢总线预取 ⇒ 读写交错取样，实测出现噪点 + 图片变形 + 花屏（用户 2026-11-01 00:30 反馈）。
    //   宽避让 = 用户 2026-10-31 22:03 验证过的那版行为。
    wire App_wr_busy = ext_wr_busy | tb_wr_busy;

    frame_fifo_read #(
        .MEM_DATA_BITS  (MEM_DATA_BITS),
        .ADDR_BITS      (ADDR_BITS),
        .BURST_BITS     (BURST_BITS),
        .FIFO_DEPTH     (FIFO_DEPTH),
        .BURST_SIZE     (BURST_SIZE)
    ) u_frame_fifo_read (
        .rst             (rst),
        .mem_clk         (mem_clk),
        .Sdr_init_done   (Sdr_init_done),
        .Sdr_init_ref_vld(Sdr_init_ref_vld),
        .Sdr_busy        (Sdr_busy),
        .Sdr_rd_en       (Sdr_rd_en),
        .App_wr_busy     (App_wr_busy),
        .O_rd_busy       (O_rd_busy),
        .App_rd_en       (ffr_rd_en),
        .App_rd_addr     (ffr_rd_addr),
        .read_req        (read_req),
        .read_req_ack    (read_req_ack),
        .read_finish     (),
        .read_addr_0     (read_addr_0),
        .read_addr_1     (read_addr_1),
        .read_addr_2     (read_addr_2),
        .read_addr_3     (read_addr_3),
        .read_addr_index (read_addr_index),
        .read_len        (read_len),
        .fifo_aclr       (fifo_aclr),
        .wrusedw         (ffr_wrusedw)
    );

    // =========================================================================
    // ② 异步 FIFO：mem_clk 写口（SDRAM 读数据）/ pix_clk 读口
    //    valid 语义（SHOW_AHEAD_EN=0, OUTREG_EN=NOREG）：valid = re 延迟 1 拍且非空，
    //    即 re 发出后下一拍 dout 才是有效数据。
    // =========================================================================
    wire [31:0] fifo_dout;
    wire        fifo_valid;
    wire        fifo_empty;
    wire        fifo_rd_en;

    rfifo_32_32_512 #(
        .DATA_WIDTH_W  (32),
        .DATA_WIDTH_R  (32),
        .ADDR_WIDTH_W  (9),
        .ADDR_WIDTH_R  (9),
        .AL_FULL_NUM   (FIFO_DEPTH - FIFO_MARGIN),
        .AL_EMPTY_NUM  (2),
        .SHOW_AHEAD_EN (1'b0),
        .OUTREG_EN     ("NOREG")
    ) u_read_fifo (
        .rst        (fifo_aclr),
        .clkw       (mem_clk),
        .clkr       (pix_clk),
        // ★ 2026-10-28：擦洗窗口内回来的数据一律不进 FIFO（纯时间窗口，不靠计数）
        // ★ 2026-10-29：缩略图生成引擎占用读口期间，回来的数据也不进 FIFO
        .we         (Sdr_rd_en & ~scrub_window & ~tb_win),
        .di         (Sdr_rd_dout),
        .re         (fifo_rd_en),
        .dout       (fifo_dout),
        .valid      (fifo_valid),
        .full_flag  (),
        .empty_flag (fifo_empty),
        .afull      (),
        .aempty     (),
        .wrusedw    (fifo_wrusedw),
        .rdusedw    ()
    );

    // =========================================================================
    // ③ 两级预取：把数据提前 1 拍顶出来，替代 video_delay 的提前相位
    // =========================================================================
    wire        prefetch_valid;
    wire [31:0] prefetch_data;
    wire        pixel_pop = pixel_consume && prefetch_valid;

    fifo_prefetch_2 #(.DATA_WIDTH(32)) u_prefetch (
        .clk         (pix_clk),
        .rst         (pix_rst),
        .raw_empty   (fifo_empty),
        .raw_valid   (fifo_valid),
        .raw_data    (fifo_dout),
        .raw_read_en (fifo_rd_en),
        .consume     (pixel_pop),
        .out_valid   (prefetch_valid),
        .out_empty   (),
        .out_data    (prefetch_data)
    );

    assign pixel_rgb   = prefetch_data[23:0];
    assign pixel_valid = prefetch_valid;

    // ---- 下溢检测 ----
    // 参考工程语义：只有"帧已武装（frame_armed）"之后，消费时无数据才算 underflow。
    // 启动期（read_req 发出、SDRAM 数据尚未回流）不算 underflow ——
    // 因为真实视频里 read_req 在 vsync 发出，有整帧时间让 FIFO 预填，de 才有效。
    reg frame_armed;
    reg read_req_seen;
    always @(posedge pix_clk or posedge pix_rst) begin
        if (pix_rst) begin
            frame_armed  <= 1'b0;
            read_req_seen<= 1'b0;
            underflow    <= 1'b0;
        end else begin
            // read_req 上升沿 → 武装
            if (read_req && !read_req_seen) begin
                read_req_seen <= 1'b1;
                frame_armed   <= 1'b0;   // 新帧请求，重新武装
            end
            if (!read_req)
                read_req_seen <= 1'b0;

            // 帧读完成后解除武装（read_req_ack 拉高表示开始读，之后数据陆续来）
            if (read_req_ack)
                frame_armed <= 1'b1;

            // underflow：armed 之后消费时无数据
            if (pixel_consume && !prefetch_valid && frame_armed)
                underflow <= 1'b1;
        end
    end

    // =========================================================================
    // ★ 2026-10-29【第15讲】2×2 缩略图页生成引擎
    //
    // 目的：把四个 bank 里的 640×480 主图各做一次 2:1 抽取（水平取偶数列、
    //       垂直取偶数行），拼成一张 640×480 的 2×2 选择页写到 TB_ADDR。
    //       页布局：象限 idx = qy*2+qx → 页内偏移 (qy*TB_H + d)*TB_SRC_W + qx*TB_W + c
    //
    // 两条硬规则（都是实机踩出来的）：
    //   ① APUG011 2.4：ERAM 的写以 4 个 32bit 字（128bit）为一组，组内必须
    //      【连发 4 拍】、首地址 4 字对齐。孤立单拍写控制器一组都不收
    //      ⇒ 旧版 307200 次写全被丢弃（页里永远是上电垃圾）。
    //   ② 源图基址必须走专用端口 src_addr_0..3（恒为 BUF0..3）。绝不能复用
    //      read_addr_0..3 —— 后者在缩略图/诊断模式下会被顶层 rd_base mux 改写
    //      成 THUMB_ADDR，引擎就会读自己正在写的那个页（实机：四块全花）。
    //
    // 读写配对：每个写组（目的列 c..c+3）需要源像素 (2d, 2c/2c+2/2c+4/2c+6)，
    // 正好落在两个相邻 128bit 读组里 ⇒ 从基址连读 8 拍，只取序号 0/2/4/6。
    // =========================================================================
    localparam [1:0] S_TB_IDLE    = 2'd0,
                     S_TB_RD      = 2'd1,
                     S_TB_COLLECT = 2'd2,
                     S_TB_WR      = 2'd3;

    reg  [1:0]  tb_state;
    reg  [1:0]  tb_img;        // 当前源图 0..3
    reg  [1:0]  tb_pass;       // 第几遍
    reg  [8:0]  tb_c;          // 页内列（步进 4）
    reg  [7:0]  tb_d;          // 页内行
    reg  [3:0]  tb_di;         // 已收返回字数
    reg  [7:0]  tb_to;         // 超时保护
    reg  [31:0] tb_q0, tb_q1, tb_q2, tb_q3;
    reg  [3:0]  tb_qv;
    reg  [1:0]  tb_g;          // 写组内第几个字 0..3
    reg         tb_go;         // 1 = 正在连发 4 字写组
    reg  [4:0]  tb_wr_guard;   // 写恢复期倒数（SDRAM tWR/tRP/tRCD ≈ 24 拍）
    reg  [ADDR_BITS-1:0] tb_wr_addr_r;
    reg  [31:0] tb_wr_data_r;
    reg         tb_done_r;
    reg         tb_step;       // 单拍：推进生成指针

    // 四张是否都加载完（同步到 mem_clk 域）
    reg  [3:0]  tb_imc_s1, tb_imc_s2;
    always @(posedge mem_clk or posedge rst) begin
        if (rst) begin
            tb_imc_s1 <= 4'd0;
            tb_imc_s2 <= 4'd0;
        end else begin
            tb_imc_s1 <= tb_image_complete;
            tb_imc_s2 <= tb_imc_s1;
        end
    end

    // 纯色测试图：tb_img 0/1/2/3 → 红/绿/蓝/黄
    wire [23:0] tb_test_pix = (tb_img == 2'd0) ? 24'hFF0000 :
                              (tb_img == 2'd1) ? 24'h00FF00 :
                              (tb_img == 2'd2) ? 24'h0000FF : 24'hFFFF00;

    // 源图基址（专用端口，绝不与 read_addr_* 共用）
    wire [ADDR_BITS-1:0] tb_src_base = (tb_img == 2'd0) ? src_addr_0 :
                                       (tb_img == 2'd1) ? src_addr_1 :
                                       (tb_img == 2'd2) ? src_addr_2 : src_addr_3;

    // 源读地址：行 2d（每行 TB_SRC_W 字，行步进 2*TB_SRC_W），列 2c → 8 字对齐
    assign tb_rd_addr = tb_src_base + (tb_d * (2 * TB_SRC_W)) + {tb_c, 1'b0};

    // 页写地址：象限行偏移 + 行内偏移 + 象限列偏移 + 列
    wire [ADDR_BITS-1:0] tb_page_off = (tb_img[1] ? (TB_H * TB_SRC_W) : 0)
                                     + (tb_d * TB_SRC_W)
                                     + (tb_img[0] ? TB_W : 0)
                                     + tb_c;
    wire [ADDR_BITS-1:0] tb_wr_addr_c = TB_ADDR + tb_page_off;

    assign tb_rd_win  = (tb_state == S_TB_RD);
    assign tb_collect = (tb_state == S_TB_COLLECT);
    assign tb_win     = tb_rd_win || tb_collect;
    assign tb_wr_busy = tb_go || (tb_wr_guard != 5'd0);

    // 有活要干：四张都加载完 + 已初始化 + 消隐期（不给显示抢带宽）
    assign tb_pending = (TB_EN != 0) && !tb_done_r && (tb_imc_s2 == 4'b1111) &&
                        Sdr_init_done && blanking;

    // 可以动：总线全静 + 擦洗窗口已收尾 + ffr 已停手 ≥24 拍 + 写恢复期已过
    wire tb_want = tb_pending && !Sdr_init_ref_vld && !O_rd_busy && !ext_wr_busy &&
                   !scrub_window && (ffr_quiet >= 6'd24) && !Sdr_busy &&
                   (tb_wr_guard == 5'd0);

    assign tb_wr_en   = tb_go;
    assign tb_wr_addr = tb_wr_addr_r;
    assign tb_wr_data = tb_wr_data_r;
    assign tb_busy    = (tb_state != S_TB_IDLE) || tb_wr_busy;
    assign tb_done    = tb_done_r;

    // ---- 写恢复期倒数（整组发完才开始计）----
    // 注意：tb_wr_guard 只在本块里赋值一次。组末装载用 tb_go_end 表达，
    // 绝不能在状态机里再写一句 tb_wr_guard <= 24（TD 会报 HDL-8007 多驱动）。
    wire tb_go_end = (tb_state == S_TB_WR) && tb_go && (tb_g == 2'd3);

    always @(posedge mem_clk or posedge rst) begin
        if (rst)                          tb_wr_guard <= 5'd0;
        else if (tb_go_end)               tb_wr_guard <= 5'd24;
        else if (tb_wr_guard != 5'd0)     tb_wr_guard <= tb_wr_guard - 5'd1;
    end

    // ---- 主状态机 ----
    always @(posedge mem_clk or posedge rst) begin
        if (rst) begin
            tb_state     <= S_TB_IDLE;
            tb_rn        <= 4'd0;
            tb_to        <= 8'd0;
            tb_g         <= 2'd0;
            tb_go        <= 1'b0;
            tb_wr_addr_r <= {ADDR_BITS{1'b0}};
            tb_wr_data_r <= 32'd0;
            tb_step      <= 1'b0;
        end else begin
            tb_step <= 1'b0;
            case (tb_state)
                S_TB_IDLE: begin
                    tb_rn <= 4'd0;
                    tb_to <= 8'd0;
                    if (tb_want) begin
                        if (TB_TEST_EN != 0) begin
                            // 纯色测试：不读源图，直接锁存一个写组
                            tb_wr_addr_r <= tb_wr_addr_c;
                            tb_wr_data_r <= {8'h00, tb_test_pix};
                            tb_g         <= 2'd0;
                            tb_go        <= 1'b1;
                            tb_state     <= S_TB_WR;
                        end else begin
                            tb_state     <= S_TB_RD;
                        end
                    end
                end
                S_TB_RD: begin
                    // 连发 8 拍读（App_rd_en 由 tb_rd_win 驱动）
                    if (tb_to != 8'd255) tb_to <= tb_to + 8'd1;
                    if (tb_rn == 4'd7) begin
                        tb_rn    <= 4'd0;
                        tb_state <= S_TB_COLLECT;
                    end else begin
                        tb_rn    <= tb_rn + 4'd1;
                    end
                    if (tb_to == 8'd255) tb_state <= S_TB_IDLE;
                end
                S_TB_COLLECT: begin
                    if (tb_to != 8'd255) tb_to <= tb_to + 8'd1;
                    if (tb_qv == 4'b1111) begin
                        tb_wr_addr_r <= tb_wr_addr_c;
                        tb_wr_data_r <= tb_q0;
                        tb_g         <= 2'd0;
                        tb_go        <= 1'b1;
                        tb_state     <= S_TB_WR;
                    end else if (tb_to == 8'd255) begin
                        // 超时保护：这一组放弃，指针照常推进，绝不死锁
                        tb_step  <= 1'b1;
                        tb_state <= S_TB_IDLE;
                    end
                end
                S_TB_WR: begin
                    if (tb_go) begin
                        if (tb_g == 2'd3) begin
                            // 第 4 个字这一拍已经发出去了，收尾
                            // （写恢复期的装载由 tb_go_end 在 guard 块内完成）
                            tb_go        <= 1'b0;
                            tb_step      <= 1'b1;
                            tb_state     <= S_TB_IDLE;
                        end else begin
                            tb_g         <= tb_g + 2'd1;
                            tb_wr_addr_r <= tb_wr_addr_r + 1'b1;
                            tb_wr_data_r <= (TB_TEST_EN != 0) ? {8'h00, tb_test_pix} :
                                            (tb_g == 2'd0) ? tb_q1 :
                                            (tb_g == 2'd1) ? tb_q2 : tb_q3;
                        end
                    end else begin
                        tb_state <= S_TB_IDLE;
                    end
                end
                default: tb_state <= S_TB_IDLE;
            endcase
        end
    end

    // ---- 读数据收集：只在 tb 窗口内收，按返回序号 0/2/4/6 取偶数列 ----
    always @(posedge mem_clk or posedge rst) begin
        if (rst) begin
            tb_di <= 4'd0;
            tb_qv <= 4'd0;
            tb_q0 <= 32'd0;
            tb_q1 <= 32'd0;
            tb_q2 <= 32'd0;
            tb_q3 <= 32'd0;
        end else if (tb_state == S_TB_IDLE) begin
            tb_di <= 4'd0;
            tb_qv <= 4'd0;
        end else if (tb_win && Sdr_rd_en && (tb_di < 4'd8)) begin
            tb_di <= tb_di + 4'd1;
            case (tb_di)
                4'd0: begin tb_q0 <= Sdr_rd_dout; tb_qv[0] <= 1'b1; end
                4'd2: begin tb_q1 <= Sdr_rd_dout; tb_qv[1] <= 1'b1; end
                4'd4: begin tb_q2 <= Sdr_rd_dout; tb_qv[2] <= 1'b1; end
                4'd6: begin tb_q3 <= Sdr_rd_dout; tb_qv[3] <= 1'b1; end
                default: ;
            endcase
        end
    end

    // ---- 生成指针推进（每组写完 / 超时跳过后推进一步）----
    always @(posedge mem_clk or posedge rst) begin
        if (rst) begin
            tb_c      <= 9'd0;
            tb_d      <= 8'd0;
            tb_img    <= 2'd0;
            tb_pass   <= 2'd0;
            tb_done_r <= 1'b0;
        end else if (tb_step) begin
            if (tb_c == (TB_W - 4)) begin
                tb_c <= 9'd0;
                if (tb_d == (TB_H - 1)) begin
                    tb_d <= 8'd0;
                    if (tb_img == 2'd3) begin
                        // 四张都做完：跑够 TB_PASSES 遍才算完成
                        if ((tb_pass + 2'd1) >= TB_PASSES) begin
                            tb_done_r <= 1'b1;
                        end else begin
                            tb_pass <= tb_pass + 2'd1;
                            tb_img  <= 2'd0;
                        end
                    end else begin
                        tb_img <= tb_img + 2'd1;
                    end
                end else begin
                    tb_d <= tb_d + 8'd1;
                end
            end else begin
                tb_c <= tb_c + 9'd4;
            end
        end
    end

endmodule
