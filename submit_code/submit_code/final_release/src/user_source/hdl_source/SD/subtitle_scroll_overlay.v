// =====================================================================
// subtitle_scroll_overlay.v : 底部滚动字幕层（第 9 讲 OSD 字幕叠加）
//
// 作用：在视频像素流上叠加一条平滑滚动的字幕带（新闻字幕风格），
//       与左上角状态 OSD 共同构成「多图层叠加」：
//            BMP 图片  →  状态 OSD(左上)  →  滚动字幕(底部)
//
// 实现要点（对应讲义各节）：
//   1. 5x7 点阵字模 + 3 倍像素放大（15x21 有效像素），自包含、无外部依赖
//      —— 讲义 2.2 节点阵字体渲染
//      ★ 为什么从 2 倍改成 3 倍：2 倍时字高 14 行，只占 480 的 2.9%，
//        实机上远看偏细、辨识吃力。3 倍后 21 行 = 4.4%，实测明显改善。
//      ★ 3 倍不能用移位实现（除以 3），用 div3() 查表函数综合成纯 LUT，
//        零乘法器；若日后改回 2 倍，把 div3 换回 ">>1" 即可，两边无其它耦合。
//   2. 平滑滚动：每帧左移 SCROLL_STEP 像素（60fps x 1 = 60 px/s）
//      —— 讲义 2.5 节平滑滚动；60 px/s 接近专业新闻字幕节奏
//   3. Alpha 半透明底带：黑底条用「每通道右移 1 位」实现约 50% 透明
//      —— 讲义 2.4 节提到的「移位近似 Alpha 混合」，省 3 个乘法器
//   4. 帧边界更新：滚动位置只在 vsync 边沿更新，一帧内固定 → 绝不撕裂
//      —— 讲义 2.3 节帧边界锁存
//   5. 多条字幕自动循环：一条滚完自动切下一条
//
// 坐标约定：video_x 为 active 对齐坐标 0~639（与 osd_overlay 的 video_x_ref 同源），
//           line_y 由 vsync/de 自重建，不增加视频时序模块接口。
// 输出为纯组合逻辑，不引入额外拍数，与 de/video_x 严格同拍。
// =====================================================================

module subtitle_scroll_overlay #(
    parameter integer H_ACTIVE    = 640,   // 有效宽度
    parameter integer BAND_Y      = 436,   // 字幕带顶部 Y
                                           //   ★ 原来 452，底带 452~472 只剩 8 行贴底，
                                           //     实机看太挤。上移到 436 后 436~464，留 16 行安全区。
    parameter integer SCROLL_STEP = 1      // 每帧左移像素数（1 = 60 px/s）
) (
    input  wire        clk,           // video_clk (25MHz)
    input  wire        rst,           // 高有效复位
    input  wire        de,            // 数据有效
    input  wire        vs,            // 场同步（低有效脉冲）
    input  wire [9:0]  video_x,       // active 对齐水平坐标 0~639
    input  wire [23:0] video_in,      // 输入 RGB888
    input  wire        enable,        // 字幕总开关（1=显示）
    output wire [23:0] video_out,     // 叠加字幕后的 RGB888
    output wire        overlay_active // 本层有输出（文字或底带）
);

    // ---------------- 布局常量（3 倍放大）----------------
    //  ★ CELL_W 取 16 而不是 15：字形实占 15 px，多留 1 px 作字间距。
    //    这样 char_idx = rel_x / 16 = rel_x >> 4，综合成纯移位，
    //    零除法器；若取 15 则要引入常量除法逻辑。
    localparam integer CELL_W = 16;            // 字符格宽（15 px 字形 + 1 px 间隙）
    localparam integer CELL_H = 23;            // 字符行高 = 7 行 x 3 倍 + 2 间距
    localparam integer TEXT_PAD = 4;           // 文字距带上边缘
    localparam integer BAND_H = CELL_H + 2*TEXT_PAD;   // 底带高 = 23 + 8 = 31
    localparam [3:0]  GLYPH_W = 4'd15;         // 字形有效宽 = 5 列 x 3
    localparam [9:0]  GLYPH_H = 10'd21;        // 字形有效高 = 7 行 x 3

    // ---------------- 行计数（垂直坐标 0~479）----------------
    reg [9:0] line_y;
    reg       de_d;
    reg       vs_d;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            line_y <= 10'd0;
            de_d   <= 1'b0;
            vs_d   <= 1'b1;
        end else begin
            de_d <= de;
            vs_d <= vs;
            if (vs_d && !vs)
                line_y <= 10'd0;               // 场同步脉冲开始，回到第 0 行
            else if (de_d && !de)
                line_y <= line_y + 10'd1;      // 一行有效像素结束，行 +1
        end
    end

    wire frame_start = vs_d && !vs;            // 帧边界（一帧只在这一点更新滚动位置）

    // ---------------- 每条字幕的文本宽度（已乘 CELL_W，避免运行时乘法）----------------
    //  ★ CELL_W=16，故宽度 = 字符数 x 16
    //  T0 "EMERGENCY INFO TERMINAL"   23 字符 -> 368
    //  T1 "POLICE 110 AID 120"   18 字符 -> 288
    //  T2 "AUTO PLAY 2000 MS"   17 字符 -> 272
    //  T3 "KEY10 ALARM KEY9 OK"   19 字符 -> 304
    function [11:0] text_width;
        input [1:0] idx;
        begin
            case (idx)
                2'd0:    text_width = 12'd368;
                2'd1:    text_width = 12'd288;
                2'd2:    text_width = 12'd272;
                default: text_width = 12'd304;
            endcase
        end
    endfunction


    // ---------------- 滚动位置 + 当前字幕条 ----------------
    //  char c 的屏幕位置 x = c*CELL_W - scroll_pos + H_ACTIVE
    //  故 scroll_pos=0 时文本全在右侧屏外；随 scroll_pos 增大从右向左滚入
    reg [12:0] scroll_pos;
    reg [1:0]  text_idx;

    wire [12:0] period = text_width(text_idx) + H_ACTIVE[11:0];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            scroll_pos <= 13'd0;
            text_idx   <= 2'd0;
        end else if (frame_start) begin
            if (scroll_pos + SCROLL_STEP[12:0] >= period) begin
                scroll_pos <= 13'd0;
                text_idx   <= text_idx + 2'd1;   // 滚完一条，自动切下一条（2bit 自然回绕）
            end else begin
                scroll_pos <= scroll_pos + SCROLL_STEP[12:0];
            end
        end
    end

    // ---------------- 5x7 点阵字模 ----------------
    //  同一行 5 位，bit4 = 最左列。仅收录字幕用得到的字符集。
    function [4:0] glyph_row;
        input [7:0] ch;
        input [2:0] row;
        begin
            glyph_row = 5'b00000;
            case (ch)
                "0": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10011;3:glyph_row=5'b10101;4:glyph_row=5'b11001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "1": case(row) 0:glyph_row=5'b00100;1:glyph_row=5'b01100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "2": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b00001;3:glyph_row=5'b00010;4:glyph_row=5'b00100;5:glyph_row=5'b01000;6:glyph_row=5'b11111;default:glyph_row=0;endcase
                "3": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b00001;2:glyph_row=5'b00001;3:glyph_row=5'b01110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "4": case(row) 0:glyph_row=5'b00010;1:glyph_row=5'b00110;2:glyph_row=5'b01010;3:glyph_row=5'b10010;4:glyph_row=5'b11111;5:glyph_row=5'b00010;6:glyph_row=5'b00010;default:glyph_row=0;endcase
                "5": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "6": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "7": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00001;2:glyph_row=5'b00010;3:glyph_row=5'b00100;4:glyph_row=5'b01000;5:glyph_row=5'b01000;6:glyph_row=5'b01000;default:glyph_row=0;endcase
                "8": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b01110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "9": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b01111;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "A": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11111;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "B": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "C": case(row) 0:glyph_row=5'b01111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b10000;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b01111;default:glyph_row=0;endcase
                "D": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "E": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b11111;default:glyph_row=0;endcase
                "F": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b10000;default:glyph_row=0;endcase
                "G": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10000;3:glyph_row=5'b10111;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "H": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11111;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "I": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b00100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "J": case(row) 0:glyph_row=5'b00111;1:glyph_row=5'b00010;2:glyph_row=5'b00010;3:glyph_row=5'b00010;4:glyph_row=5'b10010;5:glyph_row=5'b10010;6:glyph_row=5'b01100;default:glyph_row=0;endcase
                "K": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10010;2:glyph_row=5'b10100;3:glyph_row=5'b11000;4:glyph_row=5'b10100;5:glyph_row=5'b10010;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "L": case(row) 0:glyph_row=5'b10000;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b10000;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b11111;default:glyph_row=0;endcase
                "M": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b11011;2:glyph_row=5'b10101;3:glyph_row=5'b10101;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "N": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b11001;2:glyph_row=5'b10101;3:glyph_row=5'b10011;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "O": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "P": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10000;5:glyph_row=5'b10000;6:glyph_row=5'b10000;default:glyph_row=0;endcase
                "Q": case(row) 0:glyph_row=5'b01110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10101;5:glyph_row=5'b10010;6:glyph_row=5'b01101;default:glyph_row=0;endcase
                "R": case(row) 0:glyph_row=5'b11110;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b11110;4:glyph_row=5'b10100;5:glyph_row=5'b10010;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "S": case(row) 0:glyph_row=5'b01111;1:glyph_row=5'b10000;2:glyph_row=5'b10000;3:glyph_row=5'b01110;4:glyph_row=5'b00001;5:glyph_row=5'b00001;6:glyph_row=5'b11110;default:glyph_row=0;endcase
                "T": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00100;2:glyph_row=5'b00100;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b00100;default:glyph_row=0;endcase
                "U": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b10001;6:glyph_row=5'b01110;default:glyph_row=0;endcase
                "V": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10001;4:glyph_row=5'b10001;5:glyph_row=5'b01010;6:glyph_row=5'b00100;default:glyph_row=0;endcase
                "W": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b10001;3:glyph_row=5'b10101;4:glyph_row=5'b10101;5:glyph_row=5'b10101;6:glyph_row=5'b01010;default:glyph_row=0;endcase
                "X": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b01010;3:glyph_row=5'b00100;4:glyph_row=5'b01010;5:glyph_row=5'b10001;6:glyph_row=5'b10001;default:glyph_row=0;endcase
                "Y": case(row) 0:glyph_row=5'b10001;1:glyph_row=5'b10001;2:glyph_row=5'b01010;3:glyph_row=5'b00100;4:glyph_row=5'b00100;5:glyph_row=5'b00100;6:glyph_row=5'b00100;default:glyph_row=0;endcase
                "Z": case(row) 0:glyph_row=5'b11111;1:glyph_row=5'b00001;2:glyph_row=5'b00010;3:glyph_row=5'b00100;4:glyph_row=5'b01000;5:glyph_row=5'b10000;6:glyph_row=5'b11111;default:glyph_row=0;endcase
                " ": glyph_row = 5'b00000;
                default: glyph_row = 5'b00000;
            endcase
        end
    endfunction

    // ---------------- 字幕文本表（4 条，滚完自动循环）----------------
    function [7:0] sub_char;
        input [1:0] tidx;
        input [7:0] ci;
        begin
            sub_char = " ";
            case (tidx)
                // T0 "EMERGENCY INFO TERMINAL" (23)
                2'd0: case (ci)
                    8'd0:sub_char="E"; 8'd1:sub_char="M"; 8'd2:sub_char="E"; 8'd3:sub_char="R"; 8'd4:sub_char="G"; 8'd5:sub_char="E"; 8'd6:sub_char="N"; 8'd7:sub_char="C"; 8'd8:sub_char="Y"; 8'd9:sub_char=" "; 8'd10:sub_char="I"; 8'd11:sub_char="N"; 8'd12:sub_char="F"; 8'd13:sub_char="O"; 8'd14:sub_char=" "; 8'd15:sub_char="T"; 8'd16:sub_char="E"; 8'd17:sub_char="R"; 8'd18:sub_char="M"; 8'd19:sub_char="I"; 8'd20:sub_char="N"; 8'd21:sub_char="A"; 8'd22:sub_char="L";
                    default:sub_char=" ";
                endcase
                // T1 "POLICE 110 AID 120" (18)
                2'd1: case (ci)
                    8'd0:sub_char="P"; 8'd1:sub_char="O"; 8'd2:sub_char="L"; 8'd3:sub_char="I"; 8'd4:sub_char="C"; 8'd5:sub_char="E"; 8'd6:sub_char=" "; 8'd7:sub_char="1"; 8'd8:sub_char="1"; 8'd9:sub_char="0"; 8'd10:sub_char=" "; 8'd11:sub_char="A"; 8'd12:sub_char="I"; 8'd13:sub_char="D"; 8'd14:sub_char=" "; 8'd15:sub_char="1"; 8'd16:sub_char="2"; 8'd17:sub_char="0";
                    default:sub_char=" ";
                endcase
                // T2 "AUTO PLAY 2000 MS" (17)
                2'd2: case (ci)
                    8'd0:sub_char="A"; 8'd1:sub_char="U"; 8'd2:sub_char="T"; 8'd3:sub_char="O"; 8'd4:sub_char=" "; 8'd5:sub_char="P"; 8'd6:sub_char="L"; 8'd7:sub_char="A"; 8'd8:sub_char="Y"; 8'd9:sub_char=" "; 8'd10:sub_char="2"; 8'd11:sub_char="0"; 8'd12:sub_char="0"; 8'd13:sub_char="0"; 8'd14:sub_char=" "; 8'd15:sub_char="M"; 8'd16:sub_char="S";
                    default:sub_char=" ";
                endcase
                // T3 "KEY10 ALARM KEY9 OK" (19)
                default: case (ci)
                    8'd0:sub_char="K"; 8'd1:sub_char="E"; 8'd2:sub_char="Y"; 8'd3:sub_char="1"; 8'd4:sub_char="0"; 8'd5:sub_char=" "; 8'd6:sub_char="A"; 8'd7:sub_char="L"; 8'd8:sub_char="A"; 8'd9:sub_char="R"; 8'd10:sub_char="M"; 8'd11:sub_char=" "; 8'd12:sub_char="K"; 8'd13:sub_char="E"; 8'd14:sub_char="Y"; 8'd15:sub_char="9"; 8'd16:sub_char=" "; 8'd17:sub_char="O"; 8'd18:sub_char="K";
                    default:sub_char=" ";
                endcase
            endcase
        end
    endfunction

    // ---------------- 3 倍放大：除以 3（查表，零乘法器）----------------
    //  3 倍放大不能用移位，直接 /3 会引入除法逻辑。这里用 case 全展开，
    //  综合成纯 LUT，资源比一个除法器小得多。
    //  输入范围 0~20（21 行 / 15 列），输出 0~6。
    function [2:0] div3;
        input [4:0] v;
        begin
            case (v)
                5'd0, 5'd1, 5'd2:      div3 = 3'd0;
                5'd3, 5'd4, 5'd5:      div3 = 3'd1;
                5'd6, 5'd7, 5'd8:      div3 = 3'd2;
                5'd9, 5'd10, 5'd11:    div3 = 3'd3;
                5'd12, 5'd13, 5'd14:   div3 = 3'd4;
                5'd15, 5'd16, 5'd17:   div3 = 3'd5;
                default:               div3 = 3'd6;   // 18~20
            endcase
        end
    endfunction

    // ---------------- 像素渲染（纯组合，与 de/video_x 同拍）----------------
    //  char c 的屏幕位置：x = c*CELL_W - scroll_pos + H_ACTIVE
    //  反解：给定 video_x，rel_x = video_x + scroll_pos - H_ACTIVE，char_idx = rel_x / CELL_W
    wire [12:0] stream_x = {3'd0, video_x} + scroll_pos;
    wire        vis      = (stream_x >= H_ACTIVE[12:0]);          // 该像素落在文本流内
    wire [12:0] rel_x    = stream_x - H_ACTIVE[12:0];
    wire [7:0]  char_idx = vis ? (rel_x / CELL_W[12:0]) : 8'd255; // CELL_W=16：纯右移
    wire [3:0]  col_in   = vis ? (rel_x - char_idx * CELL_W[12:0]) : 4'd0;

    // 行偏移：先按 10 bit 算再饱和到 0~20，避免行号小于 BAND_Y 时下溢成巨大值
    wire [9:0]  row_off  = line_y - BAND_Y[9:0] - TEXT_PAD[9:0];
    wire [4:0]  row_raw  = (row_off > 10'd20) ? 5'd20 : row_off[4:0];

    // 3 倍放大：连续 3 行/列映射到同一个点阵位
    wire [2:0]  dot_row  = div3(row_raw);                        // 0~6
    wire [2:0]  dot_col_r= div3({1'b0, col_in});                 // 0~5
    // col_in 最大 15（不进字形区），clamp 到 4 防止 cur_glyph 越界取位
    wire [2:0]  dot_col  = (dot_col_r > 3'd4) ? 3'd4 : dot_col_r;

    wire in_band = (line_y >= BAND_Y[9:0]) && (line_y < BAND_Y[9:0] + BAND_H[9:0]);
    wire in_text = (line_y >= BAND_Y[9:0] + TEXT_PAD[9:0]) &&
                   (line_y <  BAND_Y[9:0] + TEXT_PAD[9:0] + GLYPH_H[9:0]);

    wire [7:0] cur_char  = sub_char(text_idx, char_idx);
    wire [4:0] cur_glyph = glyph_row(cur_char, dot_row);
    wire       pixel_on  = in_text && vis && (col_in < GLYPH_W[3:0]) && (dot_col < 3'd5) &&
                           cur_glyph[4 - dot_col];

    // ---------------- Alpha 半透明底带 ----------------
    //  每个通道右移 1 位 = 亮度减半，相当于在黑色上做 50% 混合（Alpha≈128）
    //  讲义 2.4 节提到的「移位近似」，省掉 3 个 8x8 乘法器
    wire [23:0] band_rgb = {1'b0, video_in[23:17],
                            1'b0, video_in[15:9],
                            1'b0, video_in[7:1]};

    wire text_on = enable && de && pixel_on;
    wire band_on = enable && de && in_band;

    assign overlay_active = text_on || band_on;
    assign video_out      = text_on ? 24'hFFFFFF :   // 白字
                            band_on ? band_rgb   :   // 半透明黑底带
                            video_in;                // 透传

endmodule
