// =====================================================================
// bmp_err_def.vh —— BMP 错误码统一定义（第5讲 Step3 8位细分错误码）
// 由 bmp24_decoder / bmp_read / sd_card_bmp / uart_logger / osd_overlay / top 共用，
// 避免各模块重复定义导致不一致。
//
// 8 位错误码（十六进制显示 0x31~0x38，数码管粗分类 1~8）：
//   0x31 ERR_SIGNATURE   不是 "BM" 签名
//   0x32 ERR_DIMENSIONS  尺寸不是 640x480
//   0x33 ERR_PLANES      平面数不是 1
//   0x34 ERR_BPP         位深不是 24
//   0x35 ERR_COMPRESSION 有压缩（BI_RLE 等）
//   0x36 ERR_OFFSET      像素偏移异常（<54 或 >=文件大小）
//   0x37 ERR_TRUNCATED   文件截断，像素不完整
//   0x38 ERR_UPSTREAM    上游（SD/文件系统）错误（超时/无响应）
// =====================================================================
`ifndef BMP_ERR_DEF_VH
`define BMP_ERR_DEF_VH

localparam [7:0] ERR_NONE        = 8'h00;   // 无错误
localparam [7:0] ERR_SIGNATURE   = 8'h31;   // 不是 "BM" 签名
localparam [7:0] ERR_DIMENSIONS  = 8'h32;   // 尺寸不是 640x480
localparam [7:0] ERR_PLANES      = 8'h33;   // 平面数不是 1
localparam [7:0] ERR_BPP         = 8'h34;   // 位深不是 24
localparam [7:0] ERR_COMPRESSION = 8'h35;   // 有压缩
localparam [7:0] ERR_OFFSET      = 8'h36;   // 像素偏移异常
localparam [7:0] ERR_TRUNCATED   = 8'h37;   // 文件截断
localparam [7:0] ERR_UPSTREAM    = 8'h38;   // 上游错误（超时/无响应）

`endif
