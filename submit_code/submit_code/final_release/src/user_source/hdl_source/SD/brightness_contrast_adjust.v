`timescale 1ns / 1ps
// =====================================================================
// brightness_contrast_adjust —— 亮度 / 对比度实时调节（第 14 讲）
//
//   亮度： y = x * (brightness + 1) / 256
//          ★ 用 (brightness+1) 而不是 brightness：
//            brightness=255 时 (255+1)/256 = 1.0，输出与输入**逐位相同**，
//            可作为严格的回归基准；若写成 x*255/256 最大会差 1 个 LSB，
//            「旁路」就不再是 bit-exact，上板比对时容易误判成 bug。
//
//   对比度： z = saturate( (y - 128) * contrast / 128 + 128 )
//          ★ 以中灰 128 为中心向两侧拉伸 / 压缩；
//            contrast=128 时 (y-128)*128/128+128 = y，天然旁路。
//          ★ /128 必须用**算术右移 >>>7**：(y-128) 可能为负，
//            逻辑右移 >> 会把负数移位成一个大正数，画面直接崩。
//
//   流水：2 拍（亮度拍 → 对比度拍）。de / vs / video_x 同步延迟 2 拍，
//         保证下游 OSD 拿到的坐标与像素严格同拍，叠加位置不会左右漂移。
//
//   参数跨域：brightness / contrast 来自控制域（50MHz），
//             内部 2FF 同步 + vs 上升沿（帧边界）锁存。
//             一帧内恒定 —— 否则会出现上半帧旧参数、下半帧新参数，
//             屏幕中间一条明显的分界线。
//
//   资源：3 个 8x8(9bit) 亮度乘法器 + 3 个对比度乘法器 = 6 个，
//         与讲义 3.2 的预估一致；无除法器（全部用移位实现）。
// =====================================================================
module brightness_contrast_adjust (
    input  wire        clk,          // 像素时钟 video_clk
    input  wire        rst_n,        // 低有效复位
    input  wire        de,
    input  wire        vs,           // 高有效，上升沿 = 帧边界
    input  wire [9:0]  video_x,      // active 对齐坐标 0~639
    input  wire [23:0] rgb_in,

    // 控制域参数（未同步，内部自行 2FF + 帧边界锁存）
    input  wire [7:0]  brightness,   // 0~255，255 = 原始亮度
    input  wire [7:0]  contrast,     // 0~255，128 = 原始对比度

    output reg         out_de,
    output reg         out_vs,
    output reg  [9:0]  out_x,
    output reg  [23:0] rgb_out,

    output reg  [7:0]  bright_eff,   // 实际生效值（锁存后，供 OSD 回显）
    output reg  [7:0]  contrast_eff
);

    // =================================================================
    // 一、参数跨域 + 帧边界锁存
    // =================================================================
    reg [7:0] bs0, bs1;      // 亮度 2FF
    reg [7:0] cs0, cs1;      // 对比度 2FF
    reg       vs_d;
    wire      vs_rise = vs & ~vs_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bs0 <= 8'd255;  bs1 <= 8'd255;
            cs0 <= 8'd128;  cs1 <= 8'd128;
            vs_d <= 1'b0;
            bright_eff   <= 8'd255;
            contrast_eff <= 8'd128;
        end else begin
            bs0 <= brightness;
            bs1 <= bs0;
            cs0 <= contrast;
            cs1 <= cs0;
            vs_d <= vs;
            // 帧边界才更新：一帧之内参数绝对不变
            if (vs_rise) begin
                bright_eff   <= bs1;
                contrast_eff <= cs1;
            end
        end
    end

    // =================================================================
    // 二、第 1 级流水：亮度   y = x * (brightness+1) / 256
    //     (brightness+1) 最大 256 → 9 bit；8bit x 9bit = 17bit
    //     ★ 取 [15:8] 即 >>8（÷256）：
    //       255*256 = 0xFF00，bit15..8 = 0xFF = 255 ✓
    //       写成 [16:9] 会多右移一位，输出只剩一半（实测 0x7F）——踩过。
    // =================================================================
    wire [8:0] bmul = {1'b0, bright_eff} + 9'd1;   // 1~256

    wire [16:0] mul_r = {9'd0, rgb_in[23:16]} * {8'd0, bmul};
    wire [16:0] mul_g = {9'd0, rgb_in[15:8]}  * {8'd0, bmul};
    wire [16:0] mul_b = {9'd0, rgb_in[7:0]}   * {8'd0, bmul};

    wire [7:0] br_r = mul_r[15:8];
    wire [7:0] br_g = mul_g[15:8];
    wire [7:0] br_b = mul_b[15:8];

    // 第 1 拍寄存器
    reg        de_p1, vs_p1;
    reg [9:0]  x_p1;
    reg [7:0]  p1_r, p1_g, p1_b;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_p1 <= 1'b0;  vs_p1 <= 1'b0;  x_p1 <= 10'd0;
            p1_r  <= 8'd0;  p1_g  <= 8'd0;  p1_b <= 8'd0;
        end else begin
            de_p1 <= de;
            vs_p1 <= vs;
            x_p1  <= video_x;
            p1_r  <= br_r;
            p1_g  <= br_g;
            p1_b  <= br_b;
        end
    end

    // =================================================================
    // 三、第 2 级流水：对比度 + 饱和截断
    //     z = saturate( (y-128) * contrast / 128 + 128 )
    //     位宽推导：
    //        cen  = y-128        → -128~127   9 bit 有符号
    //        ctr  = contrast     → 0~255      9 bit 有符号（正数）
    //        prod = cen*ctr      → ±32640     18 bit 有符号
    //        prod>>>7            → ±255       11 bit 有符号
    //        +128                → -127~383   11 bit 有符号
    // =================================================================
    wire signed [8:0] ctr_s = $signed({1'b0, contrast_eff});

    wire signed [8:0]  cen_r = $signed({1'b0, p1_r}) - 9'sd128;
    wire signed [8:0]  cen_g = $signed({1'b0, p1_g}) - 9'sd128;
    wire signed [8:0]  cen_b = $signed({1'b0, p1_b}) - 9'sd128;

    wire signed [17:0] prd_r = cen_r * ctr_s;
    wire signed [17:0] prd_g = cen_g * ctr_s;
    wire signed [17:0] prd_b = cen_b * ctr_s;

    wire signed [10:0] sum_r = (prd_r >>> 7) + 11'sd128;
    wire signed [10:0] sum_g = (prd_g >>> 7) + 11'sd128;
    wire signed [10:0] sum_b = (prd_b >>> 7) + 11'sd128;

    // 饱和：<0 取 0，>255 取 255
    wire [7:0] fin_r = (sum_r < 11'sd1)   ? 8'd0   :
                       (sum_r > 11'sd255) ? 8'd255 : sum_r[7:0];
    wire [7:0] fin_g = (sum_g < 11'sd1)   ? 8'd0   :
                       (sum_g > 11'sd255) ? 8'd255 : sum_g[7:0];
    wire [7:0] fin_b = (sum_b < 11'sd1)   ? 8'd0   :
                       (sum_b > 11'sd255) ? 8'd255 : sum_b[7:0];

    // 第 2 拍输出
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_de  <= 1'b0;  out_vs <= 1'b0;  out_x <= 10'd0;
            rgb_out <= 24'h000000;
        end else begin
            out_de  <= de_p1;
            out_vs  <= vs_p1;
            out_x   <= x_p1;
            // 消隐期强制清零，和上游 adapter 的消隐防护保持一致
            rgb_out <= de_p1 ? {fin_r, fin_g, fin_b} : 24'h000000;
        end
    end

endmodule
