// ============================================================================
// bmp24_decoder.v —— BMP24 解码器（独立化，第5讲 Step 3 架构化整理）
//
// 职责（从 bmp_read.v 中抽离的纯解析/校验/组装逻辑）：
//   1) BMP 文件头解析：从扇区字节流中流式提取 11 个字段
//        header_0/1(签名)、file_len、pixel_offset、dib_size、
//        width、height、planes、bit_count、compression
//   2) 严格校验（11 项）：header_match 输出 + 8 位细分错误码 bmp_error
//   3) 像素字节流组装：BGR 三字节 → RGB888 像素输出（pixel_valid + pixel_rgb）
//
// ★ 第 13 讲变更（图片缩放适配）：从"只认 640×480"放开到"任意合法分辨率"
//   - 尺寸校验：`width == 640 && height == 480`  →  `1 <= width <= max_width`
//     且 `1 <= height <= max_height`（上限由上层传，见 sd_card_bmp 的 MAX_IMG_W/H）
//   - 文件长度校验：`pixel_offset + 640*480*3`  →  `pixel_offset + width*height*3`
//     必须用真实像素数，否则小图（如 320×240）的 file_len 装不下 640×480 的
//     理论字节数，会被误判成 ERR_TRUNCATED(0x37) 当坏图跳过。
//   - 新增 `img_width_out / img_height_out` 输出真实分辨率，
//     供写侧缩放器（bmp_scale_640x480）做抽样，也供 OSD 显示源尺寸。
//   - 仍然拒绝：负数宽高（32 位有符号的负值 = 自上而下存储，与 loader 的
//     VFLIP 冲突）、位深≠24、有压缩、planes≠1、DIB<40。
//   本模块**不做任何缩放和行 padding**：padding 由 sd_card_bmp 按行 stride
//   跳过（保证本模块收到的像素流是纯粹的 W×H 个），缩放由写侧独立模块做。
//
// 设计说明：
//   - 输入仍对接 SD 扇区读接口（sd_sec_read_data + data_valid + 解析使能），
//     与当前工程架构保持一致，不引入课程完赛工程的"文件字节流"抽象，
//     保证抽取后对外接口与功能完全等价。
//   - 头解析使能由上层（bmp_read.v）在 ST_SCAN/ST_LOAD_HDR 状态给出。
//   - 像素组装使能由上层在 ST_LOAD_DATA 且 bmp_data_valid 时给出。
// ============================================================================
`include "bmp_err_def.vh"

module bmp24_decoder (
    input                       clk,
    input                       rst,

    // ---- 尺寸上限（第 13 讲：由"必须等于 640×480"改为"不超过上限"）----
    //   语义变更提醒：这两个端口以前叫 expected_width/expected_height，
    //   校验是 `width == expected`；第 13 讲改成 `1 <= width <= max`，
    //   才能真正支持 800×600 / 1024×768 / 1920×1080 等任意分辨率。
    //   上限的含义是"能支持多大"，不是"必须多大"。
    input       [15:0]          max_width,           // 允许的最大宽（如 2048）
    input       [15:0]          max_height,          // 允许的最大高（如 1536）

    // ---- 头解析接口（在 ST_SCAN / ST_LOAD_HDR 状态期间有效）----
    input                       hdr_parse_en,        // 头解析使能
    input       [7:0]           hdr_byte,            // 当前扇区字节
    input                       hdr_byte_valid,      // 字节有效（等价 sd_sec_read_data_valid）
    input       [9:0]           hdr_byte_idx,        // 扇区内字节偏移（等价 rd_cnt）

    // ---- 校验输出 ----
    output reg                  header_match,        // 11 项严格校验结果（流水寄存后输出）
    output reg  [7:0]           bmp_error,           // 8 位细分错误码（流水寄存后输出）

    // ---- 解析字段输出（供上层扫描/加载流程使用）----
    output wire [7:0]           header_0_out,        // 签名 B
    output wire [7:0]           header_1_out,        // 签名 M
    output wire [31:0]          file_len_out,        // 文件大小
    output wire [31:0]          pixel_offset_out,    // 像素偏移
    // ---- 第 13 讲：真实分辨率输出（供写侧缩放器用）----
    output wire [15:0]          img_width_out,       // 图片真实宽（来自 BMP 头）
    output wire [15:0]          img_height_out,      // 图片真实高

    // ---- 像素组装接口（在 ST_LOAD_DATA 期间有效）----
    input                       px_assemble_en,      // 像素组装使能（等价 bmp_data_valid）
    input       [7:0]           px_byte,             // 当前像素字节
    input       [1:0]           px_byte_idx,         // 0=B 1=G 2=R

    // ---- 像素输出 ----
    output reg                  pixel_valid,         // 一个像素组装完成（R 字节到达当拍）
    output reg  [23:0]          pixel_rgb            // RGB888 像素
);

// ============================================================================
// 字段寄存器
// ============================================================================
reg [7:0]  header_0;
reg [7:0]  header_1;
reg [31:0] file_len;
reg [31:0] pixel_offset;
reg [31:0] width;
reg [31:0] height;
reg [15:0] bit_count;
reg [31:0] compression;
reg [31:0] dib_size;
reg [15:0] planes;

// 字段输出（供上层使用）
assign header_0_out     = header_0;
assign header_1_out     = header_1;
assign file_len_out     = file_len;
assign pixel_offset_out = pixel_offset;
assign img_width_out    = width[15:0];
assign img_height_out   = height[15:0];

// 严格校验：11 项拆分（第5讲 2.1 节），逐项判断以生成细分错误码
wire sig_ok      = (header_0 == "B") && (header_1 == "M");
// ★ 第 13 讲：尺寸校验从"相等"改为"合法范围"
//   ① 高 16 位必须为 0：BMP 的宽高是 32 位有符号，负数表示自上而下存储，
//      本工程不支持（会与 loader 的 VFLIP 冲突），直接判非法
//   ② 宽高必须 ≥1 且 ≤ 上限：上限由上层给（受 SPI 读卡时间和超时兜底约束）
wire dim_nonzero = (width[15:0]  != 16'd0) && (height[15:0] != 16'd0);
wire dim_inrange = (width[15:0]  <= max_width) && (height[15:0] <= max_height);
wire dim_hi_zero = (width[31:16] == 16'd0) && (height[31:16] == 16'd0);
wire dim_ok      = dim_hi_zero && dim_nonzero && dim_inrange;
wire planes_ok   = (planes == 16'd1);
wire bpp_ok      = (bit_count == 16'd24);
wire comp_ok     = (compression == 32'd0);
wire dib_ok      = (dib_size >= 32'd40);
wire offset_ok   = (pixel_offset >= 32'd54) && (pixel_offset < file_len);

// ============================================================================
// 校验流水化（2026-10-21）
//
// 原写法：
//   wire flen_ok = (file_len >= pixel_offset + expected_pixels * 32'd3);
//   wire header_match = 各项 && flen_ok;          // 组合直出
// 综合结果：32 位加法（4 级进位链）+ 32 位比较（4 级进位链）= 8 级逻辑，
//   Data Path Delay 10.902ns，100MHz 下 setup 违例 -1.254ns；
//   而且该组合结果一路穿过 sd_card_bmp 的错误汇总进 uart_logger 的 msg_val 使能，
//   形成 bmp24_decoder → sd_card_bmp → uart_logger 的跨模块长路径（最差 8 条同源）。
//
// 拆分：
//   级0（第13讲新增）：img_pixels = width * height              —— 1 个 16×16 乘法
//   级1（下方）：min_file_len = pixel_offset + img_pixels*3      —— 移位加法 + 32 位加法
//   级2（下方）：flen_ok = file_len >= min_file_len + 汇总       —— 4 级比较
//   级3（下方）：header_match / bmp_error 寄存输出               —— 切断跨模块路径
//
// 为什么安全：这些字段在头解析（前 54 字节）完成后即稳定，而 header_match /
//   bmp_error 唯一的消费点是 sd_card_bmp 的 load_error（判定条件含
//   streamer_file_done，即整个文件读完，至少几万拍之后）。4 拍流水延迟
//   远小于该时间差，功能完全不受影响。
// ============================================================================

// 级0（第 13 讲新增）：真实像素数 = width × height
//   ★ 原来这里是 `expected_width * expected_height`，两个都是常量 640/480，
//     综合后直接折成常数 307200，零成本。改成任意分辨率后宽高要到运行时
//     才知道，必须真算一次乘法 —— 用 1 个 DSP（16×16→32），比用 LUT 搭
//     进位链便宜得多，且每拍都刷新也无所谓（头解析完成后值就不再变）。
//   门控 hdr_byte_idx >= 34：1~33 字节还没收齐（compression 收在 30~33），
//   提前算会用半成品字段，算出一个假的小像素数 → 误判 ERR_TRUNCATED。
reg [31:0] img_pixels;
always @(posedge clk or posedge rst) begin
    if (rst)
        img_pixels <= 32'd0;
    else if (hdr_parse_en && hdr_byte_valid && (hdr_byte_idx >= 10'd34))
        img_pixels <= {16'd0, width[15:0]} * {16'd0, height[15:0]};
end

// 级1：最小合法文件长度 = 像素数据起始偏移 + 像素数据字节数
//   `*3` 写成 (x<<1)+x：三倍宽（24bpp 每像素 3 字节）用移位加法，不占 DSP
reg [31:0] min_file_len;
always @(posedge clk or posedge rst) begin
    if (rst)
        min_file_len <= 32'd0;
    else
        min_file_len <= pixel_offset + (img_pixels << 1) + img_pixels;
end

// 级2：比较与汇总（组合，最多 4 级）
wire flen_ok        = (file_len >= min_file_len);
wire header_match_w = sig_ok && dib_ok && dim_ok && planes_ok && bpp_ok &&
                      comp_ok && offset_ok && flen_ok;

// 细分错误码（优先级从高到低：签名→尺寸→平面→位深→压缩→偏移→文件大小）
reg [7:0] bmp_error_w;
always @* begin
    if (header_match_w) begin
        bmp_error_w = ERR_NONE;
    end else if (!sig_ok) begin
        bmp_error_w = ERR_SIGNATURE;     // 0x31 不是 BM 签名
    end else if (!dim_ok) begin
        bmp_error_w = ERR_DIMENSIONS;    // 0x32 尺寸不对
    end else if (!planes_ok) begin
        bmp_error_w = ERR_PLANES;        // 0x33 平面数不对
    end else if (!bpp_ok) begin
        bmp_error_w = ERR_BPP;           // 0x34 位深不对
    end else if (!comp_ok) begin
        bmp_error_w = ERR_COMPRESSION;   // 0x35 有压缩
    end else if (!offset_ok) begin
        bmp_error_w = ERR_OFFSET;        // 0x36 偏移异常
    end else if (!dib_ok) begin
        bmp_error_w = ERR_SIGNATURE;     // DIB<40 归为签名类（非标准 BMP）
    end else begin
        bmp_error_w = ERR_TRUNCATED;     // 0x37 文件装不下像素（截断）
    end
end

// 级3：寄存输出（切断 bmp24_decoder → sd_card_bmp → uart_logger 的长组合路径）
always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_match <= 1'b0;
        bmp_error    <= ERR_NONE;
    end else begin
        header_match <= header_match_w;
        bmp_error    <= bmp_error_w;
    end
end

// ============================================================================
// 头解析：流式提取 11 个字段（与原 bmp_read.v 的 case 完全等价）
// ============================================================================
always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        file_len     <= 32'd0;
        pixel_offset <= 32'd54;
        width        <= 32'd0;
        height       <= 32'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
        dib_size     <= 32'd0;
        planes       <= 16'd0;
    end else if (hdr_parse_en && hdr_byte_valid) begin
        case (hdr_byte_idx)
            10'd0 : header_0 <= hdr_byte;
            10'd1 : header_1 <= hdr_byte;

            10'd2 : file_len[7:0] <= hdr_byte;
            10'd3 : file_len[15:8] <= hdr_byte;
            10'd4 : file_len[23:16] <= hdr_byte;
            10'd5 : file_len[31:24] <= hdr_byte;

            10'd10: pixel_offset[7:0] <= hdr_byte;
            10'd11: pixel_offset[15:8] <= hdr_byte;
            10'd12: pixel_offset[23:16] <= hdr_byte;
            10'd13: pixel_offset[31:24] <= hdr_byte;

            10'd14: dib_size[7:0] <= hdr_byte;
            10'd15: dib_size[15:8] <= hdr_byte;
            10'd16: dib_size[23:16] <= hdr_byte;
            10'd17: dib_size[31:24] <= hdr_byte;

            10'd18: width[7:0] <= hdr_byte;
            10'd19: width[15:8] <= hdr_byte;
            10'd20: width[23:16] <= hdr_byte;
            10'd21: width[31:24] <= hdr_byte;

            10'd22: height[7:0] <= hdr_byte;
            10'd23: height[15:8] <= hdr_byte;
            10'd24: height[23:16] <= hdr_byte;
            10'd25: height[31:24] <= hdr_byte;

            10'd26: planes[7:0] <= hdr_byte;
            10'd27: planes[15:8] <= hdr_byte;

            10'd28: bit_count[7:0] <= hdr_byte;
            10'd29: bit_count[15:8] <= hdr_byte;

            10'd30: compression[7:0] <= hdr_byte;
            10'd31: compression[15:8] <= hdr_byte;
            10'd32: compression[23:16] <= hdr_byte;
            10'd33: compression[31:24] <= hdr_byte;
            default: ;
        endcase
    end
end

// ============================================================================
// 像素组装：BGR 三字节 → RGB888（与原 bmp_read.v 的拼装逻辑完全等价）
// ============================================================================
always @(posedge clk or posedge rst) begin
    if (rst) begin
        pixel_valid <= 1'b0;
        pixel_rgb   <= 24'd0;
    end else if (px_assemble_en) begin
        case (px_byte_idx)
            2'd0: begin
                pixel_valid       <= 1'b0;
                pixel_rgb[7:0]    <= px_byte;   // B
            end
            2'd1: begin
                pixel_valid       <= 1'b0;
                pixel_rgb[15:8]   <= px_byte;   // G
            end
            2'd2: begin
                pixel_valid       <= 1'b1;      // 一个像素完成
                pixel_rgb[23:16]  <= px_byte;   // R
            end
            default: begin
                pixel_valid <= 1'b0;
            end
        endcase
    end else begin
        pixel_valid <= 1'b0;
    end
end

endmodule
