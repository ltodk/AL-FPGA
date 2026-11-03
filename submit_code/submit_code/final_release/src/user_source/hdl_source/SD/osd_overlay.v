// =====================================================================
// osd_overlay: 屏幕状态叠加（第5讲 Step2 可观测性）
//   在画面左上角叠加三行 8x16 ASCII 文字：
//     行1: "SD:OK" / "SD:NO CARD"（无卡红色，正常白色）
//     行2: "IMG:1/4"（当前图片/总数）
//     行3: "ERR:0x31"~"ERR:0x38"（仅出错时显示红色，否则空白，8位细分码）
//     行4: "B:255 C:128"（第14讲，亮度/对比度，调节后显示 3 秒）
//     行5: "AU:NO FILE" / "AU:READY" / "AU:PLAY" / "AU:UNDER"（第11讲阶段C 音频状态）
//     行6: "POS  3/  6s"（第11讲阶段C 音频播放进度）
//   运行在 video_clk(pixel clock) 域；状态信号跨时钟域经两级同步。
//
//   输入 video_x 为 active 对齐坐标 0~639（顶层 video_x_ref）。
//   行计数 line_cnt：vs 下降沿清零，de 下降沿 +1（得到 0~479）。
// =====================================================================
`include "bmp_err_def.vh"
`include "osd_font.v"

module osd_overlay (
    input  wire        clk,          // pixel_clk (video_clk)
    input  wire        rst,          // 高有效复位
    input  wire        de,           // 视频有效
    input  wire        vs,           // 场同步
    input  wire [9:0]  video_x,      // active 对齐水平坐标 0~639
    input  wire [23:0] video_in,     // 输入 RGB888

    // 状态（跨时钟域，慢变化，内部两级同步）
    input  wire        sd_init_done,
    input  wire [1:0]  img_idx,      // 当前图片 0~3
    input  wire [2:0]  img_total,    // 图片总数 1~4
    input  wire [7:0]  bmp_err_code, // 错误码（8位细分 0x31~0x38）

    // 第14讲：亮度/对比度参数回显（调节后显示 3 秒）
    input  wire [7:0]  param_bright,
    input  wire [7:0]  param_contrast,
    input  wire        param_show,

    // 第11讲阶段C：音频状态四态显示
    //   audio_found    —— 卡上扫描到了 MUSIC.PCM（sd_card_clk 域，慢变化电平）
    //   audio_active   —— 正在出声（video_clk 域）
    //   audio_underrun —— 缓冲空、正在补数字静音（video_clk 域）
    input  wire        audio_found,
    input  wire        audio_active,
    input  wire        audio_underrun,
    // 第11讲阶段C：播放进度（来自 sd_card_clk 域，1Hz 级慢变化，内部两级同步）
    input  wire [9:0]  audio_pos_sec,
    input  wire [9:0]  audio_tot_sec,

    output wire [23:0] video_out
);

// ---------- 跨时钟域同步（2级）----------
reg        sd_s0, sd_s1;
reg [1:0]  idx_s0, idx_s1;
reg [2:0]  tot_s0, tot_s1;
reg [7:0]  err_s0, err_s1;
reg [2:0]  tot_show;   // 锁存的图片总数：total=0 时保持上一非零值

// 第14讲：亮度/对比度参数（来自 50MHz 控制域）+ 显示使能
reg [7:0]  pbr_s0, pbr_s1;
reg [7:0]  pct_s0, pct_s1;
reg        psh_s0, psh_s1;

// 第11讲阶段C：音频三态（audio_found 来自 100MHz 域需同步；
//   audio_active/audio_underrun 本来就在 video_clk 域，多打一拍无害且统一了时序）
reg        auf_s0, auf_s1;
reg        aud_s0, aud_s1;
reg        aun_s0, aun_s1;
reg [9:0]  aps_s0, aps_s1;      // 已播秒
reg [9:0]  ats_s0, ats_s1;      // 总秒

always @(posedge clk or posedge rst) begin
    if (rst) begin
        sd_s0 <= 1'b0;  sd_s1 <= 1'b0;
        idx_s0 <= 2'd0; idx_s1 <= 2'd0;
        tot_s0 <= 3'd0; tot_s1 <= 3'd0;
        err_s0 <= 8'd0; err_s1 <= 8'd0;
        tot_show <= 3'd0;
        pbr_s0 <= 8'd255; pbr_s1 <= 8'd255;
        pct_s0 <= 8'd128; pct_s1 <= 8'd128;
        psh_s0 <= 1'b0;   psh_s1 <= 1'b0;
        auf_s0 <= 1'b0;   auf_s1 <= 1'b0;
        aud_s0 <= 1'b0;   aud_s1 <= 1'b0;
        aun_s0 <= 1'b0;   aun_s1 <= 1'b0;
        aps_s0 <= 10'd0;  aps_s1 <= 10'd0;
        ats_s0 <= 10'd0;  ats_s1 <= 10'd0;
    end else begin
        sd_s0 <= sd_init_done;  sd_s1 <= sd_s0;
        idx_s0 <= img_idx;      idx_s1 <= idx_s0;
        tot_s0 <= img_total;    tot_s1 <= tot_s0;
        err_s0 <= bmp_err_code; err_s1 <= err_s0;
        pbr_s0 <= param_bright;   pbr_s1 <= pbr_s0;
        pct_s0 <= param_contrast; pct_s1 <= pct_s0;
        psh_s0 <= param_show;     psh_s1 <= psh_s0;
        auf_s0 <= audio_found;    auf_s1 <= auf_s0;
        aud_s0 <= audio_active;   aud_s1 <= aud_s0;
        aun_s0 <= audio_underrun; aun_s1 <= aun_s0;
        aps_s0 <= audio_pos_sec;  aps_s1 <= aps_s0;
        ats_s0 <= audio_tot_sec;  ats_s1 <= ats_s0;
        // 总数锁存：仅在同步后的 total 非零时更新，避免重扫瞬间显示 IMG:x/0
        if (tot_s0 != 3'd0)
            tot_show <= tot_s0;
    end
end

// ---------- 行计数（垂直坐标 0~479）----------
reg [9:0] line_cnt;
reg       de_d;
reg       vs_d;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        line_cnt <= 10'd0;
        de_d     <= 1'b0;
        vs_d     <= 1'b0;
    end else begin
        de_d <= de;
        vs_d <= vs;
        if (vs_d && !vs)
            line_cnt <= 10'd0;          // 场同步结束，回到第0行
        else if (de_d && !de)
            line_cnt <= line_cnt + 10'd1;  // 一行结束，行+1
    end
end

// ---------- OSD 布局 ----------
localparam [9:0] X0   = 10'd8;     // 起始水平
localparam [9:0] Y1   = 10'd8;     // 行1 起始垂直
localparam [9:0] Y2   = 10'd26;    // 行2
localparam [9:0] Y3   = 10'd44;    // 行3
localparam [9:0] Y4   = 10'd62;    // 行4（第14讲：亮度/对比度回显）
localparam [9:0] Y5   = 10'd80;    // 行5（第11讲阶段C：音频状态）
localparam [9:0] Y6   = 10'd98;    // 行6（第11讲阶段C：播放进度）
localparam [9:0] CW   = 10'd8;     // 字符宽
localparam [9:0] CH   = 10'd16;    // 字符高

wire in_l1 = (line_cnt >= Y1) && (line_cnt < Y1 + CH);
wire in_l2 = (line_cnt >= Y2) && (line_cnt < Y2 + CH);
wire in_l3 = (line_cnt >= Y3) && (line_cnt < Y3 + CH);
wire in_l4 = (line_cnt >= Y4) && (line_cnt < Y4 + CH);
wire in_l5 = (line_cnt >= Y5) && (line_cnt < Y5 + CH);
wire in_l6 = (line_cnt >= Y6) && (line_cnt < Y6 + CH);

// 相对每行起始的垂直偏移（0~15）
wire [3:0] row_l1 = line_cnt[3:0] - Y1[3:0];
wire [3:0] row_l2 = line_cnt[3:0] - Y2[3:0];
wire [3:0] row_l3 = line_cnt[3:0] - Y3[3:0];
wire [3:0] row_l4 = line_cnt[3:0] - Y4[3:0];
wire [3:0] row_l5 = line_cnt[3:0] - Y5[3:0];
wire [3:0] row_l6 = line_cnt[3:0] - Y6[3:0];

// 水平字符索引（0~127，7位！4位会每16字符/128像素回绕导致文字重复5次）
// 与字符内列（0~7）
wire [6:0] col_idx    = (video_x >= X0) ? ((video_x - X0) >> 3) : 7'd0;
wire [2:0] bit_in_col = (video_x >= X0) ? ((video_x - X0) & 3'd7) : 3'd0;

// ---------- 各行 ASCII 生成 ----------
// 行1：SD 状态
function [7:0] l1_ascii;
    input [6:0] c;
    input       ok;
    begin
        if (ok) begin
            case (c)
                4'd0: l1_ascii = "S";
                4'd1: l1_ascii = "D";
                4'd2: l1_ascii = ":";
                4'd3: l1_ascii = "O";
                4'd4: l1_ascii = "K";
                default: l1_ascii = " ";
            endcase
        end else begin
            case (c)
                4'd0: l1_ascii = "S";
                4'd1: l1_ascii = "D";
                4'd2: l1_ascii = ":";
                4'd3: l1_ascii = "N";
                4'd4: l1_ascii = "O";
                4'd5: l1_ascii = " ";
                4'd6: l1_ascii = "C";
                4'd7: l1_ascii = "A";
                4'd8: l1_ascii = "R";
                4'd9: l1_ascii = "D";
                default: l1_ascii = " ";
            endcase
        end
    end
endfunction

// 行2：图片编号
function [7:0] l2_ascii;
    input [6:0] c;
    input [1:0] idx;
    input [2:0] total;
    begin
        case (c)
            4'd0: l2_ascii = "I";
            4'd1: l2_ascii = "M";
            4'd2: l2_ascii = "G";
            4'd3: l2_ascii = ":";
            4'd4: l2_ascii = 8'h30 + {4'd0, idx} + 8'd1;   // 当前编号 1~4
            4'd5: l2_ascii = "/";
            4'd6: l2_ascii = 8'h30 + {5'd0, total};        // 总数 1~4
            default: l2_ascii = " ";
        endcase
    end
endfunction

// 行3：错误码（8位细分，显示 0x31~0x38）
// 十六进制数字转 ASCII
function [7:0] hex_ascii;
    input [3:0] h;
    begin
        hex_ascii = (h <= 4'd9) ? (8'h30 + {4'd0, h}) : (8'h37 + {4'd0, h});
    end
endfunction

function [7:0] l3_ascii;
    input [6:0] c;
    input [7:0] err;
    begin
        if (err == 8'd0) begin
            l3_ascii = " ";
        end else begin
            case (c)
                4'd0: l3_ascii = "E";
                4'd1: l3_ascii = "R";
                4'd2: l3_ascii = "R";
                4'd3: l3_ascii = ":";
                4'd4: l3_ascii = hex_ascii(err[7:4]);   // 高4位（如 3）
                4'd5: l3_ascii = hex_ascii(err[3:0]);   // 低4位（如 8）
                default: l3_ascii = " ";
            endcase
        end
    end
endfunction

// ---------- 行4（第14讲）：亮度/对比度十进制回显 ----------
// 显示格式： "B:255 C:128"（11 字符）
// ★ 十进制转换故意不用 / 和 %：8 位除 10 / 100 会引出一个除法器或乘法器，
//   改用"比较 + 常数减法"实现，纯 LUT，零乘法器开销（255 只需要 9 次比较）。
function [3:0] dec_digit;
    input [7:0] v;
    input [1:0] n;        // 0=百位 1=十位 2=个位
    reg   [7:0] r;        // 去掉百位后的余数（0~99）
    reg   [3:0] h, t;
    begin
        h = (v >= 8'd200) ? 4'd2 : (v >= 8'd100) ? 4'd1 : 4'd0;
        r = v - h * 8'd100;
        t = (r >= 8'd90) ? 4'd9 : (r >= 8'd80) ? 4'd8 :
            (r >= 8'd70) ? 4'd7 : (r >= 8'd60) ? 4'd6 :
            (r >= 8'd50) ? 4'd5 : (r >= 8'd40) ? 4'd4 :
            (r >= 8'd30) ? 4'd3 : (r >= 8'd20) ? 4'd2 :
            (r >= 8'd10) ? 4'd1 : 4'd0;
        case (n)
            2'd0:    dec_digit = h;
            2'd1:    dec_digit = t;
            default: dec_digit = r - t * 8'd10;
        endcase
    end
endfunction

function [7:0] l4_ascii;
    input [6:0] c;
    input [7:0] bv;
    input [7:0] cv;
    begin
        case (c)
            7'd0:  l4_ascii = "B";
            7'd1:  l4_ascii = ":";
            7'd2:  l4_ascii = 8'h30 + {4'd0, dec_digit(bv, 2'd0)};
            7'd3:  l4_ascii = 8'h30 + {4'd0, dec_digit(bv, 2'd1)};
            7'd4:  l4_ascii = 8'h30 + {4'd0, dec_digit(bv, 2'd2)};
            7'd5:  l4_ascii = " ";
            7'd6:  l4_ascii = "C";
            7'd7:  l4_ascii = ":";
            7'd8:  l4_ascii = 8'h30 + {4'd0, dec_digit(cv, 2'd0)};
            7'd9:  l4_ascii = 8'h30 + {4'd0, dec_digit(cv, 2'd1)};
            7'd10: l4_ascii = 8'h30 + {4'd0, dec_digit(cv, 2'd2)};
            default: l4_ascii = " ";
        endcase
    end
endfunction

// ---------- 行5（第11讲阶段C）：音频状态四态 ----------
//   !found            → "AU:NO FILE"  卡上没有 MUSIC.PCM
//    found & underrun → "AU:UNDER"    正在补数字静音（缓冲被抽空）
//    found & active   → "AU:PLAY"     正常出声
//    found & !active  → "AU:READY"    已找到、正在攒缓冲（还没开声）
//
// ★ 为什么把 UNDER 单独列出来：它和 PLAY 听起来都是"有声音"，
//   但 UNDER 时段实际吐的是 16'sd0（补静音），是"卡顿"的直接证据。
//   没有这一态，出问题时只能靠耳朵判断"是不是卡了一下"。
function [7:0] l5_ascii;
    input [6:0] c;
    input       f;      // found
    input       a;      // active
    input       u;      // underrun
    begin
        if (!f) begin
            case (c)
                4'd0: l5_ascii = "A";
                4'd1: l5_ascii = "U";
                4'd2: l5_ascii = ":";
                4'd3: l5_ascii = "N";
                4'd4: l5_ascii = "O";
                4'd5: l5_ascii = " ";
                4'd6: l5_ascii = "F";
                4'd7: l5_ascii = "I";
                4'd8: l5_ascii = "L";
                4'd9: l5_ascii = "E";
                default: l5_ascii = " ";
            endcase
        end else if (u) begin
            case (c)
                4'd0: l5_ascii = "A";
                4'd1: l5_ascii = "U";
                4'd2: l5_ascii = ":";
                4'd3: l5_ascii = "U";
                4'd4: l5_ascii = "N";
                4'd5: l5_ascii = "D";
                4'd6: l5_ascii = "E";
                4'd7: l5_ascii = "R";
                default: l5_ascii = " ";
            endcase
        end else if (a) begin
            case (c)
                4'd0: l5_ascii = "A";
                4'd1: l5_ascii = "U";
                4'd2: l5_ascii = ":";
                4'd3: l5_ascii = "P";
                4'd4: l5_ascii = "L";
                4'd5: l5_ascii = "A";
                4'd6: l5_ascii = "Y";
                default: l5_ascii = " ";
            endcase
        end else begin
            case (c)
                4'd0: l5_ascii = "A";
                4'd1: l5_ascii = "U";
                4'd2: l5_ascii = ":";
                4'd3: l5_ascii = "R";
                4'd4: l5_ascii = "E";
                4'd5: l5_ascii = "A";
                4'd6: l5_ascii = "D";
                4'd7: l5_ascii = "Y";
                default: l5_ascii = " ";
            endcase
        end
    end
endfunction

// ---------- 行6（第11讲阶段C）：播放进度 "POS  3/  6s" ----------
// ★ 十进制分解故意不用 / 和 %：wav_progress 里已经算过字节→秒，这里只要把
//   0..999 拆成三位。用"阈值比较 + 常数减法"实现，纯 LUT、零乘法器/除法器。
function [3:0] d3_hundreds;        // 0..999 → 百位
    input [9:0] v;
    begin
        d3_hundreds = (v >= 10'd900) ? 4'd9 : (v >= 10'd800) ? 4'd8 :
                      (v >= 10'd700) ? 4'd7 : (v >= 10'd600) ? 4'd6 :
                      (v >= 10'd500) ? 4'd5 : (v >= 10'd400) ? 4'd4 :
                      (v >= 10'd300) ? 4'd3 : (v >= 10'd200) ? 4'd2 :
                      (v >= 10'd100) ? 4'd1 : 4'd0;
    end
endfunction

function [9:0] d3_rem100;          // 0..999 → 去掉百位后的余数（0..99）
    input [9:0] v;
    begin
        d3_rem100 = (v >= 10'd900) ? (v - 10'd900) : (v >= 10'd800) ? (v - 10'd800) :
                    (v >= 10'd700) ? (v - 10'd700) : (v >= 10'd600) ? (v - 10'd600) :
                    (v >= 10'd500) ? (v - 10'd500) : (v >= 10'd400) ? (v - 10'd400) :
                    (v >= 10'd300) ? (v - 10'd300) : (v >= 10'd200) ? (v - 10'd200) :
                    (v >= 10'd100) ? (v - 10'd100) : v;
    end
endfunction

function [3:0] d3_tens;            // 0..99 → 十位
    input [9:0] v;
    begin
        d3_tens = (v >= 10'd90) ? 4'd9 : (v >= 10'd80) ? 4'd8 :
                  (v >= 10'd70) ? 4'd7 : (v >= 10'd60) ? 4'd6 :
                  (v >= 10'd50) ? 4'd5 : (v >= 10'd40) ? 4'd4 :
                  (v >= 10'd30) ? 4'd3 : (v >= 10'd20) ? 4'd2 :
                  (v >= 10'd10) ? 4'd1 : 4'd0;
    end
endfunction

function [9:0] d3_rem10;           // 0..99 → 个位
    input [9:0] v;
    begin
        d3_rem10 = (v >= 10'd90) ? (v - 10'd90) : (v >= 10'd80) ? (v - 10'd80) :
                   (v >= 10'd70) ? (v - 10'd70) : (v >= 10'd60) ? (v - 10'd60) :
                   (v >= 10'd50) ? (v - 10'd50) : (v >= 10'd40) ? (v - 10'd40) :
                   (v >= 10'd30) ? (v - 10'd30) : (v >= 10'd20) ? (v - 10'd20) :
                   (v >= 10'd10) ? (v - 10'd10) : v;
    end
endfunction

// 数字只在**外面算一次**，再由 l6_ascii 按列选取。
// 若把分解放进 l6_ascii 内部，综合器会为每个 case 分支各复制一份逻辑，面积翻好几倍。
wire [3:0] pos_h = d3_hundreds(aps_s1);
wire [9:0] pos_r = d3_rem100 (aps_s1);
wire [3:0] pos_t = d3_tens    (pos_r);
wire [3:0] pos_o = d3_rem10   (pos_r);
wire [3:0] tot_h = d3_hundreds(ats_s1);
wire [9:0] tot_r = d3_rem100 (ats_s1);
wire [3:0] tot_t = d3_tens    (tot_r);
wire [3:0] tot_o = d3_rem10   (tot_r);

// 布局：c=0..2 "POS"  c=3 空格  c=4..6 已播秒（高位空白抑制）
//       c=7 '/'   c=8..10 总秒  c=11 's'
// 例：6 秒素材播到第 3 秒 → "POS   3/  6s"
function [7:0] l6_ascii;
    input [6:0] c;
    input [9:0] p;
    input [9:0] t;
    input [3:0] ph, pt, po;
    input [3:0] th, tt, to;
    begin
        case (c)
            7'd0:  l6_ascii = "P";
            7'd1:  l6_ascii = "O";
            7'd2:  l6_ascii = "S";
            7'd3:  l6_ascii = " ";
            7'd4:  l6_ascii = (p >= 10'd100) ? (8'h30 + {4'd0, ph}) : " ";
            7'd5:  l6_ascii = (p >= 10'd10)  ? (8'h30 + {4'd0, pt}) : " ";
            7'd6:  l6_ascii = 8'h30 + {4'd0, po};
            7'd7:  l6_ascii = "/";
            7'd8:  l6_ascii = (t >= 10'd100) ? (8'h30 + {4'd0, th}) : " ";
            7'd9:  l6_ascii = (t >= 10'd10)  ? (8'h30 + {4'd0, tt}) : " ";
            7'd10: l6_ascii = 8'h30 + {4'd0, to};
            7'd11: l6_ascii = "s";
            default: l6_ascii = " ";
        endcase
    end
endfunction

// ---------- 组合逻辑选字模 ----------
reg [7:0] ascii_sel;
reg [3:0] font_row_sel;
reg [2:0] font_bit;

always @(*) begin
    ascii_sel    = 8'h20;
    font_row_sel = 4'd0;
    font_bit     = 3'd0;
    // 仅在文字区内渲染；video_x < X0 时强制空白，
    // 否则 col_idx 恒为0会把首字符最左列画到 x=0~7 形成杂散竖条
    if (in_l1 && (video_x >= X0)) begin
        ascii_sel    = l1_ascii(col_idx, sd_s1);
        font_row_sel = row_l1;
        font_bit     = bit_in_col;
    end else if (in_l2 && sd_s1 && (video_x >= X0)) begin
        // IMG 行只要有卡(sd_s1)就显示；total 由 OSD 侧锁存(tot_show)，
        // 避免上层重扫瞬间 total=0 导致 IMG 行整行消失；无卡时隐藏
        ascii_sel    = l2_ascii(col_idx, idx_s1, tot_show);
        font_row_sel = row_l2;
        font_bit     = bit_in_col;
    end else if (in_l3 && (video_x >= X0)) begin
        ascii_sel    = l3_ascii(col_idx, err_s1);
        font_row_sel = row_l3;
        font_bit     = bit_in_col;
    end else if (in_l4 && psh_s1 && (video_x >= X0)) begin
        // 第14讲：调节后显示 3 秒，自动隐藏
        ascii_sel    = l4_ascii(col_idx, pbr_s1, pct_s1);
        font_row_sel = row_l4;
        font_bit     = bit_in_col;
    end else if (in_l5 && sd_s1 && (video_x >= X0)) begin
        // 第11讲阶段C：有卡就显示音频状态（无卡时整行隐藏，与行2一致）
        ascii_sel    = l5_ascii(col_idx, auf_s1, aud_s1, aun_s1);
        font_row_sel = row_l5;
        font_bit     = bit_in_col;
    end else if (in_l6 && sd_s1 && auf_s1 && (video_x >= X0)) begin
        // 第11讲阶段C：只有找到音频文件才显示进度（没有音频时显示 0/0 没意义）
        ascii_sel    = l6_ascii(col_idx, aps_s1, ats_s1,
                                pos_h, pos_t, pos_o, tot_h, tot_t, tot_o);
        font_row_sel = row_l6;
        font_bit     = bit_in_col;
    end
end

wire [7:0] font_byte = osd_font_row(ascii_sel, font_row_sel);
wire       pixel_on  = font_byte[7 - font_bit];

// ---------- 黑色底条 + 白字（播放器字幕风格）----------
// 描边方案在纯白背景上失效（白字融入白底，只剩黑轮廓碎片，又粗又糊）。
// 改用实心黑色底条垫在文字后面：任何背景下文字都清晰锐利。
// 底条范围：每行文字区域外扩 2px 边距；各行只在有内容时才画底条。
localparam [9:0] PAD = 10'd2;                    // 底条边距
// 各行文字实际宽度（字符数×8）：行1最长 "SD:NO CARD"=10字符，行2 "IMG:1/4"=7，行3 "ERR:38"=6
localparam [9:0] W1 = 10'd10 * CW;
localparam [9:0] W2 = 10'd7  * CW;
localparam [9:0] W3 = 10'd6  * CW;
localparam [9:0] W4 = 10'd11 * CW;               // 第14讲 "B:255 C:128" = 11 字符
localparam [9:0] W5 = 10'd10 * CW;               // 第11讲 "AU:NO FILE" = 10 字符（最长）
localparam [9:0] W6 = 10'd12 * CW;               // 第11讲 "POS 999/999s" = 12 字符

wire l1_show = in_l1;                              // 行1常显（SD 状态）
wire l2_show = in_l2 && sd_s1;                     // 行2有卡才显
wire l3_show = in_l3 && (err_s1 != 8'd0);          // 行3仅出错时显
wire l4_show = in_l4 && psh_s1;                    // 行4仅调节后 3 秒内显
wire l5_show = in_l5 && sd_s1;                     // 行5有卡才显（音频状态）
wire l6_show = in_l6 && sd_s1 && auf_s1;           // 行6找到音频才显（播放进度）

wire box_l1 = l1_show && (video_x >= X0 - PAD) && (video_x < X0 + W1 + PAD);
wire box_l2 = l2_show && (video_x >= X0 - PAD) && (video_x < X0 + W2 + PAD);
wire box_l3 = l3_show && (video_x >= X0 - PAD) && (video_x < X0 + W3 + PAD);
wire box_l4 = l4_show && (video_x >= X0 - PAD) && (video_x < X0 + W4 + PAD);
wire box_l5 = l5_show && (video_x >= X0 - PAD) && (video_x < X0 + W5 + PAD);
wire box_l6 = l6_show && (video_x >= X0 - PAD) && (video_x < X0 + W6 + PAD);

// ---------- 颜色 ----------
wire [23:0] WHITE  = 24'hFFFFFF;
wire [23:0] RED    = 24'hFF0000;
wire [23:0] GREEN  = 24'h00E000;   // 播放中（不用纯 00FF00，过饱和刺眼）
wire [23:0] YELLOW = 24'hFFC000;   // 欠载补静音
wire [23:0] GREY   = 24'h8A8A8A;   // 无音频文件（信息性提示，不报警）
wire [23:0] BLACK  = 24'h000000;

wire [23:0] osd_color;      // 笔画主色
assign osd_color = (in_l1 && !sd_s1) ? RED :            // 无卡红色
                   (in_l3 && (err_s1 != 8'd0)) ? RED :   // 错误红色
                   (in_l5 && !auf_s1) ? GREY :           // 无音频文件：灰
                   (in_l5 && aun_s1)  ? YELLOW :         // 欠载：黄
                   (in_l5 && aud_s1)  ? GREEN :          // 播放中：绿
                   WHITE;                                 // READY 及其它：白

wire osd_active = (l1_show | l2_show | l3_show | l4_show | l5_show | l6_show) && pixel_on && de;
wire box_on     = (box_l1 | box_l2 | box_l3 | box_l4 | box_l5 | box_l6) && de;

// ---------- 输出（组合，不引入额外延迟，保持与 de/video_x 严格同拍）----------
assign video_out = osd_active ? osd_color :
                   box_on     ? BLACK :
                   video_in;

endmodule
