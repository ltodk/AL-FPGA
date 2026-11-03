// 第5讲系统稳定性增强版：
//  1) BMP 严格校验（11项，含DIB头/平面数/pixel_offset范围/file_len合理性）
//  2) 错误码上报（bmp_error + load_error 脉冲，8位细分）
//  3) 坏图快速跳过（"BM"签名但校验失败时按文件大小整文件跳过）
//  4) SD 读取超时（SD_READ_TIMEOUT_MS，防死锁）
//  5) 像素完整性校验（截断图报 ERR_TRUNCATED，防撕裂）
//  Step3：BMP 解析/校验/像素组装已抽离到 bmp24_decoder
`include "bmp_err_def.vh"

// =====================================================================
// 【已退役 · 请勿例化】
// 第七讲起，本模块的职责已被 FAT32 链路完全替代：
//   物理扇区扫描 + "BM" 魔数匹配  →  fat32_volume_scanner（按 8.3 文件名匹配）
//   顺序扇区读取                 →  fat32_file_streamer（按 FAT 链跨簇读）
//   sd_card_top 直连             →  sd_sector_adapter（扇区接口适配）
//   BMP 头解析/校验/像素组装     →  bmp24_decoder（Step3 已抽离，仍在用）
// 本文件已从 TD 工程编译列表移除，仅作第5讲历史参考保留，不参与综合。
// =====================================================================

module bmp_read #(
    parameter integer CLK_FREQ_HZ        = 100_000_000,
    parameter integer SD_READ_TIMEOUT_MS = 500,         // 扇区读取超时（毫秒）
    // 启动优化开关：FAT32 快速定位数据区。
    // 真卡诊断确认解析值正确(DS:00004000=数据区起始16384扇区)，启用后扫描
    // 跳过 MBR+FAT 表区(~16384无效扇区)，开机从20s降到2~3s。
    parameter        FAST_BOOT_EN         = 1'b1
)(
    input                       clk,
    input                       rst,
    output                      ready,

    // 上电扫描：从 scan_start_sector 开始，顺序寻找前 scan_target_count 张 BMP
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,

    // 按指定扇区加载一张图到 SDRAM
    input                       load_start,
    input                       load_abort,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    output reg [3:0]            state_code,
    // 第13讲：尺寸校验从"必须等于"放开成"不超过上限"，端口同步改名
    input  [15:0]               max_width,
    input  [15:0]               max_height,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output wire                 bmp_data_wr_en,
    output wire [23:0]          bmp_data,

    // 错误上报（第5讲，8位细分错误码）
    output reg [7:0]            bmp_error,     // 错误码（加载/扫描失败当拍有效）
    output reg                  load_error,    // 错误单拍脉冲

    // FAT 诊断输出（每次扫描先解析 MBR/DBR，把解析结果上报供 UART 打印；
    // 是否采用解析结果由 FAST_BOOT_EN 决定——实机验证前保持0）
    output reg                  fat_dbg_pulse, // DBR 解析完成单拍脉冲
    output reg                  fat_dbg_ok,    // 1=解析合法 0=解析失败(将回退)
    output reg [31:0]           fat_dbg_val,   // 解析出的数据区起始扇区

    // 扫描结束诊断（scan_done 当拍起保持到下次扫描）：
    //   zero=0 -> val=首张好图所在扇区（定位文件在卡上的真实位置）
    //   zero=1 -> val=扫描终止扇区（0图收场，通常=SCAN_MAX_SECTOR）
    output reg                  scan_diag_pulse,
    output reg                  scan_diag_zero,
    output reg [31:0]           scan_diag_val
);

// 超时阈值（时钟周期数）：500ms @ 100MHz = 50,000,000
localparam TIMEOUT_CYCLES = (CLK_FREQ_HZ / 1000) * SD_READ_TIMEOUT_MS;

localparam ST_IDLE      = 3'd0;
localparam ST_SCAN      = 3'd1;
localparam ST_LOAD_HDR  = 3'd2;
localparam ST_LOAD_WAIT = 3'd3;
localparam ST_LOAD_DATA = 3'd4;
// FAT32 快速启动（第5讲 Step3 启动优化）：
// 扫描前先解析 MBR/DBR 定位数据区起始扇区，跳过 FAT 表区（数千扇区无效读），
// 上电出图 10s -> 2~3s。解析失败自动回退传统全盘扫描（兼容仿真/非常规卡）。
localparam ST_FAT_MBR   = 3'd5;   // 读扇区0(MBR)，解析分区起始LBA
localparam ST_FAT_DBR   = 3'd6;   // 读DBR，解析保留扇区/FAT表数/FAT表大小

reg [2:0]  state;
reg [9:0]  rd_cnt;

// ---- FAT32 解析寄存器 ----
reg [31:0] part_lba;        // MBR 分区表项1的起始 LBA（偏移0x1C6）
reg [15:0] fat_bps;         // DBR 每扇区字节数（偏移0x0B，应为512）
reg [15:0] fat_reserved;    // 保留扇区数（偏移0x0E）
reg [7:0]  fat_cnt;         // FAT 表份数（偏移0x10，通常2）
reg [15:0] fat_size16;      // FAT 表大小（扇区，偏移0x16，FAT16用）
reg [31:0] fat_size32;      // FAT 表大小（扇区，偏移0x24，FAT32用）
reg        mbr_sig_ok;      // MBR 尾签名 0x55AA 有效（无则扇区0当DBR，如免分区格式）
reg [7:0]  sig_lo;          // MBR 签名字节 0x1FE
reg [7:0]  sig_hi;          // MBR 签名字节 0x1FF

// 数据区起始 = 分区起始 + 保留扇区 + FAT份数 × FAT表大小
// （FAT16 用 fat_size16，FAT32 时 fat_size16==0 改用 fat_size32）
wire [31:0] fat_size_eff = (fat_size16 != 16'd0) ? {16'd0, fat_size16} : fat_size32;
wire [31:0] fat_data_start = part_lba + {16'd0, fat_reserved} + fat_cnt * fat_size_eff;
// 解析结果有效性：BPS=512、FAT份数1~8、FAT表大小非0、数据区落在扫描范围内
wire fat_parse_ok = (fat_bps == 16'd512) && (fat_cnt >= 8'd1) && (fat_cnt <= 8'd8) &&
                    (fat_size_eff != 32'd0) &&
                    (fat_data_start < scan_max_sector) && (fat_data_start >= part_lba);

reg [31:0] scan_sector;
reg [15:0] miss_streak;        // 连续"非BMP"扇区计数：FAT32顺序拷入的文件数据连续，
                               // 扫过最后一张图后全是空白，连续SCAN_MISS_LIMIT个
                               // 非BMP扇区即可判定"后面没有了"提前结束（否则要
                               // 一路扫到SCAN_MAX_SECTOR，14.8GB卡耗时20s+）
localparam SCAN_MISS_LIMIT = 16'd4096;  // 连续4096扇区(2MB)无BMP即提前结束
                                        // （实测首图离数据区起点差4160扇区，512太小
                                        //   会在到达首图前误判"后面没了"导致漏图黑屏）
reg [31:0] first_found_sec;    // 本轮扫描首个好图所在扇区（诊断用）
reg        scan_diag_running;  // 已发起扫描待上报标志（防重复脉冲）
reg [31:0] load_sector_latched;
reg [31:0] bmp_len_cnt;
reg [1:0]  bmp_byte_idx;
reg [31:0] sd_timeout_cnt;        // SD 读取超时计数器
reg [31:0] pixel_count;           // 已输出像素计数（完整性校验）
reg        finish_pending;        // 整帧读完待校验像素数（延迟1拍，确保计数稳定）

wire header_match;
wire bmp_data_valid;
wire [31:0] file_sector_count;
wire [31:0] next_scan_sector_if_match;
wire [31:0] next_scan_sector_if_miss;
wire [31:0] next_scan_sector_if_skip;
wire [31:0] expected_pixels;
wire        sd_read_timeout;

// ===== bmp24_decoder 接口（头解析/校验/像素组装已抽离） =====
wire [7:0]  header_0_w;          // 签名 B（坏图跳过判断用）
wire [7:0]  header_1_w;          // 签名 M
wire [31:0] file_len_w;          // 文件长度（扫描/加载流程需要）
wire [31:0] pixel_offset_w;      // 像素偏移（bmp_data_valid 判断用）
wire        pixel_valid_w;       // 像素组装完成脉冲
wire [23:0] pixel_rgb_w;         // RGB888 像素
wire [7:0]  dec_err_w;           // decoder 细分错误码（头校验类）

assign ready = (state == ST_IDLE);
// 严格校验（11 项）已移入 bmp24_decoder，header_match 由其输出
assign bmp_data_valid = (sd_sec_read_data_valid == 1'b1) &&
                        (bmp_len_cnt >= pixel_offset_w) &&
                        (bmp_len_cnt <  file_len_w);
assign file_sector_count = (file_len_w == 32'd0) ? 32'd1 : ((file_len_w + 32'd511) >> 9);
assign next_scan_sector_if_match = scan_sector + file_sector_count;
assign next_scan_sector_if_miss  = scan_sector + 32'd1;
// "BM"签名但校验失败：按文件大小整文件跳过（坏图快速跳过）
assign next_scan_sector_if_skip  = scan_sector + file_sector_count;
// ★ 第13讲：像素总数必须按"头里真实解析出来的 W×H"算，不能用上限乘积，
//   否则 800×600 的图会被要求读 307200 个像素而判成 ERR_TRUNCATED。
wire [15:0] img_width_w;
wire [15:0] img_height_w;
assign expected_pixels = {16'd0, img_width_w} * {16'd0, img_height_w};
// 超时判定：扫描/读头/等应答/读数据 任一状态卡住超过 SD_READ_TIMEOUT_MS
assign sd_read_timeout = (sd_timeout_cnt >= TIMEOUT_CYCLES) &&
                         ((state == ST_SCAN) || (state == ST_LOAD_HDR) ||
                          (state == ST_LOAD_WAIT) || (state == ST_LOAD_DATA) ||
                          (state == ST_FAT_MBR) || (state == ST_FAT_DBR));

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_cnt <= 10'd0;
    end else if ((state == ST_SCAN) || (state == ST_LOAD_HDR) ||
                 (state == ST_FAT_MBR) || (state == ST_FAT_DBR)) begin
        if (sd_sec_read_data_valid)
            rd_cnt <= rd_cnt + 10'd1;
        else if (sd_sec_read_end)
            rd_cnt <= 10'd0;
    end else begin
        rd_cnt <= 10'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_len_cnt <= 32'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (sd_sec_read_data_valid)
            bmp_len_cnt <= bmp_len_cnt + 32'd1;
    end else begin
        bmp_len_cnt <= 32'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_byte_idx <= 2'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid)
            bmp_byte_idx <= (bmp_byte_idx == 2'd2) ? 2'd0 : (bmp_byte_idx + 2'd1);
    end else begin
        bmp_byte_idx <= 2'd0;
    end
end

// 像素完整性计数：每输出一个像素（第3字节到达）+1；进入读头状态清零
always @(posedge clk or posedge rst) begin
    if (rst) begin
        pixel_count <= 32'd0;
    end else if (state == ST_LOAD_HDR) begin
        pixel_count <= 32'd0;
    end else if ((state == ST_LOAD_DATA) && bmp_data_valid && (bmp_byte_idx == 2'd2)) begin
        pixel_count <= pixel_count + 32'd1;
    end
end

// SD 读取超时计数：按"状态停留时间"计——任一状态停留超过 SD_READ_TIMEOUT_MS
// 且期间无扇区完成，则判超时。每进入新状态或读完一个扇区都清零，
// 保证大卡整体扫描（最坏 ~0.7s）不会误报，只有真死锁（SD 卡无响应）才触发。
reg [2:0] state_d;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        state_d        <= ST_IDLE;
        sd_timeout_cnt <= 32'd0;
    end else begin
        state_d <= state;
        if ((state != state_d) || sd_sec_read_end)
            sd_timeout_cnt <= 32'd0;
        else if ((state == ST_SCAN) || (state == ST_LOAD_HDR) ||
                 (state == ST_LOAD_WAIT) || (state == ST_LOAD_DATA) ||
                 (state == ST_FAT_MBR) || (state == ST_FAT_DBR)) begin
            if (sd_timeout_cnt != 32'hffff_ffff)
                sd_timeout_cnt <= sd_timeout_cnt + 32'd1;
        end else begin
            sd_timeout_cnt <= 32'd0;
        end
    end
end

// 像素组装已移入 bmp24_decoder，此处 bmp_data_wr_en / bmp_data 由其输出驱动
assign bmp_data_wr_en = pixel_valid_w;
assign bmp_data       = pixel_rgb_w;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        miss_streak       <= 16'd0;
        first_found_sec   <= 32'd0;
        scan_diag_pulse   <= 1'b0;
        scan_diag_running <= 1'b0;
        fat_dbg_pulse     <= 1'b0;
        fat_dbg_ok        <= 1'b0;
        fat_dbg_val       <= 32'd0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
        bmp_error         <= ERR_NONE;
        load_error        <= 1'b0;
        finish_pending    <= 1'b0;
    end else if (!sd_init_done) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
        bmp_error         <= ERR_NONE;
        load_error        <= 1'b0;
        finish_pending    <= 1'b0;
    end else if (load_abort) begin
        // 加载超时中止：只回到 IDLE 并停止读，保留 scan_done 与已扫描的图信息，
        // 避免 scan_done 被清零导致所有按键永久失效
        state             <= ST_IDLE;
        state_code        <= 4'd1;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_found_valid  <= 1'b0;
        load_sector_latched <= 32'd0;
        load_error        <= 1'b0;
        finish_pending    <= 1'b0;
    end else begin
        scan_found_valid <= 1'b0;
        load_error       <= 1'b0;
        finish_pending   <= 1'b0;

        if (sd_read_timeout) begin
            // SD 读取超时（防死锁最后一道保险）：报 ERR_UPSTREAM 并回 IDLE
            bmp_error         <= ERR_UPSTREAM;
            load_error        <= 1'b1;
            state             <= ST_IDLE;
            state_code        <= 4'd5;
            sd_sec_read       <= 1'b0;
            sd_sec_read_addr  <= 32'd0;
            write_req         <= 1'b0;
            scan_found_valid  <= 1'b0;
        end else begin
        case (state)
            ST_IDLE: begin
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;

                // 延迟1拍的像素完整性校验（此时 pixel_count 已稳定）
                if (finish_pending) begin
                    if (pixel_count < expected_pixels) begin
                        bmp_error  <= ERR_TRUNCATED;
                        load_error <= 1'b1;
                    end
                end

                if (scan_start) begin
                    scan_done         <= 1'b0;
                    scan_found_total  <= 3'd0;
                    miss_streak       <= 16'd0;
                    first_found_sec   <= 32'd0;
                    scan_diag_running <= 1'b1;
                    scan_diag_pulse   <= 1'b0;   // 清脉冲，保证本轮结束能产生干净 0->1 上升沿
                    scan_sector       <= scan_start_sector;
                    // 每次扫描先读 MBR/DBR 做诊断（结果经 fat_dbg_* 上报 UART）；
                    // 扫描起点是否采用解析值由 FAST_BOOT_EN 决定（实机验证前=0，走稳定路径）
                    sd_sec_read_addr <= 32'd0;
                    state            <= ST_FAT_MBR;
                end else if (load_start) begin
                    load_sector_latched <= load_sector;
                    sd_sec_read_addr    <= load_sector;
                    state               <= ST_LOAD_HDR;
                end
            end

            ST_FAT_MBR: begin
                state_code  <= 4'd6;
                sd_sec_read <= 1'b1;
                fat_dbg_pulse <= 1'b0;   // 单拍脉冲：离开 DBR 状态即清

                // 采 MBR 字段：分区表项1起始LBA(0x1C6)、尾签名(0x1FE~0x1FF)
                if (sd_sec_read_data_valid) begin
                    case (rd_cnt)
                        10'd454: part_lba[7:0]   <= sd_sec_read_data;
                        10'd455: part_lba[15:8]  <= sd_sec_read_data;
                        10'd456: part_lba[23:16] <= sd_sec_read_data;
                        10'd457: part_lba[31:24] <= sd_sec_read_data;
                        10'd510: sig_lo <= sd_sec_read_data;
                        10'd511: sig_hi <= sd_sec_read_data;
                        default: ;
                    endcase
                end

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    // 有 MBR 签名(55AA) → 读分区首扇区(DBR)；无签名(免分区格式) → 扇区0本身即DBR
                    mbr_sig_ok <= (sig_lo == 8'h55) && (sig_hi == 8'hAA);
                    if (!((sig_lo == 8'h55) && (sig_hi == 8'hAA)))
                        part_lba <= 32'd0;
                    sd_sec_read_addr <= ((sig_lo == 8'h55) && (sig_hi == 8'hAA)) ? part_lba : 32'd0;
                    state <= ST_FAT_DBR;
                end
            end

            ST_FAT_DBR: begin
                state_code  <= 4'd7;
                sd_sec_read <= 1'b1;

                // 采 DBR 字段：BPS(0x0B)、保留扇区(0x0E)、FAT份数(0x10)、FAT大小(0x16/0x24)
                if (sd_sec_read_data_valid) begin
                    case (rd_cnt)
                        10'd11 : fat_bps[7:0]    <= sd_sec_read_data;
                        10'd12 : fat_bps[15:8]   <= sd_sec_read_data;
                        10'd14 : fat_reserved[7:0]  <= sd_sec_read_data;
                        10'd15 : fat_reserved[15:8] <= sd_sec_read_data;
                        10'd16 : fat_cnt            <= sd_sec_read_data;
                        10'd22 : fat_size16[7:0]    <= sd_sec_read_data;
                        10'd23 : fat_size16[15:8]   <= sd_sec_read_data;
                        10'd36 : fat_size32[7:0]    <= sd_sec_read_data;   // 0x24 FAT32表大小
                        10'd37 : fat_size32[15:8]   <= sd_sec_read_data;
                        10'd38 : fat_size32[23:16]  <= sd_sec_read_data;
                        10'd39 : fat_size32[31:24]  <= sd_sec_read_data;
                        default: ;
                    endcase
                end

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    // 诊断上报：无论是否采用，都把解析结果打给 UART
                    fat_dbg_pulse <= 1'b1;
                    fat_dbg_ok    <= fat_parse_ok;
                    fat_dbg_val   <= fat_data_start;
                    if (FAST_BOOT_EN && fat_parse_ok) begin
                        // 启用快速启动且解析合法：扫描从数据区起始扇区开始
                        sd_sec_read_addr <= fat_data_start;
                        scan_sector      <= fat_data_start;
                    end else begin
                        // 默认/解析失败：回退原始扫描起点（实机验证稳定路径）
                        sd_sec_read_addr <= scan_sector;
                        scan_sector      <= scan_sector;
                    end
                    state <= ST_SCAN;
                end
            end

            ST_SCAN: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;
                fat_dbg_pulse <= 1'b0;   // 进入扫描即清脉冲，保证 DS 消息只发一次

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;

                    if (header_match) begin
                        scan_found_valid  <= 1'b1;
                        scan_found_sector <= scan_sector;
                        scan_found_total  <= scan_found_total + 3'd1;
                        miss_streak       <= 16'd0;   // 命中好图，连续空扇区计数清零
                        if (scan_found_total == 3'd0)
                            first_found_sec <= scan_sector;   // 记录首图位置（诊断）

                        if ((scan_found_total + 3'd1 >= scan_target_count) || (next_scan_sector_if_match > scan_max_sector)) begin
                            scan_done        <= 1'b1;
                            scan_diag_pulse  <= scan_diag_running;
                            scan_diag_zero   <= 1'b0;
                            scan_diag_val    <= first_found_sec;
                            scan_diag_running <= 1'b0;
                            state            <= ST_IDLE;
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end else begin
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end
                    end else if ((header_0_w == "B") && (header_1_w == "M")) begin
                        // 坏图快速跳过：是 BMP 签名但校验失败，按文件大小整文件跳过
                        miss_streak <= 16'd0;   // 命中签名，连续空扇区计数清零
                        if (next_scan_sector_if_skip > scan_max_sector) begin
                            scan_done       <= 1'b1;
                            scan_diag_pulse <= scan_diag_running;
                            scan_diag_zero  <= (scan_found_total == 3'd0);
                            scan_diag_val   <= (scan_found_total == 3'd0) ? scan_sector : first_found_sec;
                            scan_diag_running <= 1'b0;
                            state           <= ST_IDLE;
                        end else begin
                            sd_sec_read_addr <= next_scan_sector_if_skip;
                            scan_sector      <= next_scan_sector_if_skip;
                        end
                    end else begin
                        // 非BMP扇区：找到首图之后，连续 SCAN_MISS_LIMIT 个空扇区
                        // 即提前结束（FAT32顺序写盘尾部全是空白，扫到卡尾要20s+）。
                        // 找到首图之前不提前终止——文件真实位置未知，必须扫全程，
                        // 否则数据区起点与文件间有间隙时会漏掉所有图（实机踩坑）。
                        if (scan_sector >= scan_max_sector) begin
                            miss_streak     <= 16'd0;
                            scan_done       <= 1'b1;
                            scan_diag_pulse <= scan_diag_running;
                            scan_diag_zero  <= (scan_found_total == 3'd0);
                            scan_diag_val   <= (scan_found_total == 3'd0) ? scan_sector : first_found_sec;
                            scan_diag_running <= 1'b0;
                            state           <= ST_IDLE;
                        end else if ((miss_streak >= SCAN_MISS_LIMIT) && (scan_found_total != 3'd0)) begin
                            miss_streak     <= 16'd0;
                            scan_done       <= 1'b1;
                            scan_diag_pulse <= scan_diag_running;
                            scan_diag_zero  <= 1'b0;
                            scan_diag_val   <= first_found_sec;
                            scan_diag_running <= 1'b0;
                            state           <= ST_IDLE;
                        end else begin
                            miss_streak      <= miss_streak + 16'd1;
                            sd_sec_read_addr <= next_scan_sector_if_miss;
                            scan_sector      <= next_scan_sector_if_miss;
                        end
                    end
                end
            end

            ST_LOAD_HDR: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (header_match) begin
                        write_req        <= 1'b1;
                        sd_sec_read_addr <= load_sector_latched;
                        state            <= ST_LOAD_WAIT;
                    end else begin
                        // 严格校验失败：报 decoder 细分错误码（不再是静默回 IDLE）
                        bmp_error <= dec_err_w;
                        load_error <= 1'b1;
                        state     <= ST_IDLE;
                    end
                end
            end

            ST_LOAD_WAIT: begin
                state_code <= 4'd3;
                if (write_req_ack) begin
                    write_req <= 1'b0;
                    state     <= ST_LOAD_DATA;
                end
            end

            ST_LOAD_DATA: begin
                state_code  <= 4'd4;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (bmp_len_cnt >= file_len_w) begin
                        // 延迟1拍再校验像素数，确保最后一个像素计数已写入
                        finish_pending <= 1'b1;
                        state <= ST_IDLE;
                    end else begin
                        sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                    end
                end
            end

            default: begin
                state       <= ST_IDLE;
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
            end
        endcase
        end
    end
end

// ============================================================================
// bmp24_decoder 实例化：头解析 + 11项校验 + 像素组装（已从本模块抽离）
// ============================================================================
bmp24_decoder u_bmp24_decoder (
    .clk              (clk),
    .rst              (rst),
    // 尺寸上限（运行时输入）
    .max_width        (max_width),
    .max_height       (max_height),
    // 头里真实解析出来的分辨率
    .img_width_out    (img_width_w),
    .img_height_out   (img_height_w),
    // 头解析：SCAN/HDR 状态期间流式喂入
    .hdr_parse_en     ((state == ST_SCAN) || (state == ST_LOAD_HDR)),
    .hdr_byte         (sd_sec_read_data),
    .hdr_byte_valid   (sd_sec_read_data_valid),
    .hdr_byte_idx     (rd_cnt),
    // 校验输出
    .header_match     (header_match),
    .bmp_error        (dec_err_w),
    .header_0_out     (header_0_w),
    .header_1_out     (header_1_w),
    .file_len_out     (file_len_w),
    .pixel_offset_out (pixel_offset_w),
    // 像素组装：LOAD_DATA 且有效字节期间
    .px_assemble_en   (bmp_data_valid),
    .px_byte          (sd_sec_read_data),
    .px_byte_idx      (bmp_byte_idx),
    // 像素输出
    .pixel_valid      (pixel_valid_w),
    .pixel_rgb        (pixel_rgb_w)
);

endmodule
