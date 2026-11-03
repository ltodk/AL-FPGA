// =====================================================================
// uart_logger: 系统事件 -> ASCII 日志字符串（第5讲 Step2 串口可观测性）
//   运行在 sd_card_clk(100MHz) 域，检测以下事件并逐字节送往 uart_tx：
//     - SD 卡初始化完成   -> "SD OK\r\n"
//     - SD 卡拔出/失联    -> "SD OUT\r\n"
//     - 切图（img_idx 变化）-> "IMG:1/4\r\n"（当前编号/总数）
//     - 加载错误           -> "ERR:0x31\r\n"~"ERR:0x38\r\n"（8位细分错误码）
//   每个消息以 CRLF(\r\n) 结尾。事件在发送期间到达会被覆盖（深度1，可接受）。
// =====================================================================
`include "bmp_err_def.vh"

module uart_logger #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BAUD_RATE   = 115200
)(
    input  wire       clk,
    input  wire       rst,

    // 状态输入（均来自 sd_card_clk 域，慢变化）
    input  wire       sd_init_done,
    input  wire [1:0] img_idx,        // 当前图片 0~3
    input  wire [2:0] img_total,      // 已找到图片总数 1~4
    input  wire       load_error,     // 错误脉冲
    input  wire [7:0] bmp_err_code,   // 错误码（8位细分）
    input  wire       ld3s_pulse,     // 3s兜底超时脉冲（诊断）
    input  wire       ld3s_src,       // 兜底时源读完成状态：1=写通道丢完成信号
    input  wire       fat_dbg_pulse,  // FAT解析完成脉冲（诊断）
    input  wire       fat_dbg_ok,     // FAT解析合法性
    input  wire [31:0] fat_dbg_val,   // FAT解析出的数据区起始扇区
    input  wire       scan_diag_pulse, // 扫描结束脉冲（诊断）
    input  wire       scan_diag_zero,  // 1=本轮0图收场
    input  wire [31:0] scan_diag_val,  // 首图扇区(zero=0)或终止扇区(zero=1)

    output wire       uart_tx
);

// ---------- 事件沿检测 ----------
reg sd_init_done_d;
wire sd_init_rise = sd_init_done & ~sd_init_done_d;
wire sd_init_fall = ~sd_init_done & sd_init_done_d;

reg [1:0] img_idx_d;
wire img_changed = (img_idx != img_idx_d);

reg load_error_d;
wire load_error_rise = load_error & ~load_error_d;

reg ld3s_pulse_d;
wire ld3s_rise = ld3s_pulse & ~ld3s_pulse_d;

reg fat_dbg_pulse_d;
wire fat_dbg_rise = fat_dbg_pulse & ~fat_dbg_pulse_d;

reg scan_diag_pulse_d;
wire scan_diag_rise = scan_diag_pulse & ~scan_diag_pulse_d;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        sd_init_done_d  <= 1'b0;
        img_idx_d       <= 2'd0;
        load_error_d    <= 1'b0;
        ld3s_pulse_d    <= 1'b0;
        fat_dbg_pulse_d <= 1'b0;
        scan_diag_pulse_d <= 1'b0;
    end else begin
        sd_init_done_d  <= sd_init_done;
        img_idx_d       <= img_idx;
        load_error_d    <= load_error;
        ld3s_pulse_d    <= ld3s_pulse;
        fat_dbg_pulse_d <= fat_dbg_pulse;
        scan_diag_pulse_d <= scan_diag_pulse;
    end
end

// ---------- 事件锁存（覆盖式，深度1）----------
// 消息模板号 msg_kind 与负载（图片编号/总数/错误码）在事件当拍锁存。
// 若此时正在发送，直接覆盖（可观测性用途可接受）。
reg [3:0]  msg_kind;    // 1=SD OK, 2=SD OUT, 3=ERR, 4=IMG, 5=WR:3S, 6=RD:3S,
                        // 7=DS:ok, 8=DS:FAIL, 9=S1:首图扇区, 10=S0:0图终止扇区
reg [1:0]  msg_idx;     // IMG 消息用：当前编号 0~3
reg [2:0]  msg_total;   // IMG 消息用：总数 1~4
reg [7:0]  msg_err;     // ERR 消息用：8位错误码
reg [31:0] msg_val;     // DS/S1/S0 消息用：扇区号（8位十六进制）
reg        msg_pending;
reg        fs_start;    // 发送状态机取走消息的脉冲

wire evt_any = sd_init_rise | sd_init_fall | img_changed | load_error_rise | ld3s_rise | fat_dbg_rise | scan_diag_rise;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        msg_kind    <= 4'd0;
        msg_idx     <= 2'd0;
        msg_total   <= 3'd0;
        msg_err     <= 8'd0;
        msg_val     <= 32'd0;
        msg_pending <= 1'b0;
    end else begin
        // 事件优先级：SD状态 > 错误 > FAT诊断 > 扫描结束诊断 > 切图 > 3s兜底
        if (sd_init_rise) begin
            msg_kind    <= 4'd1;
            msg_pending <= 1'b1;
        end else if (sd_init_fall) begin
            msg_kind    <= 4'd2;
            msg_pending <= 1'b1;
        end else if (load_error_rise && (bmp_err_code != ERR_NONE)) begin
            msg_kind    <= 4'd3;
            msg_err     <= bmp_err_code;
            msg_pending <= 1'b1;
        end else if (fat_dbg_rise) begin
            // FAT 诊断：kind7="DS:XXXXXXXX"（解析值） / kind8="DS:FAIL"（解析失败）
            msg_kind    <= fat_dbg_ok ? 4'd7 : 4'd8;
            msg_val     <= fat_dbg_val;
            msg_pending <= 1'b1;
        end else if (scan_diag_rise) begin
            // 扫描结束诊断：kind9="S1:XXXXXXXX"（首图扇区） / kind10="S0:XXXXXXXX"（0图终止位置）
            msg_kind    <= scan_diag_zero ? 4'd10 : 4'd9;
            msg_val     <= scan_diag_val;
            msg_pending <= 1'b1;
        end else if (img_changed && sd_init_done && (img_total != 3'd0)) begin
            // img_total==0 说明错误重置正在进行（img_idx/total 同拍清零），
            // 此时打 "IMG:x/0" 无意义，跳过本次事件，等下一次正常切图再打
            msg_kind    <= 4'd4;
            msg_idx     <= img_idx;
            msg_total   <= img_total;
            msg_pending <= 1'b1;
        end else if (ld3s_rise) begin
            // 3s兜底超时诊断：区分写通道/读通道问题
            msg_kind    <= ld3s_src ? 4'd5 : 4'd6;   // 5=WR:3S  6=RD:3S
            msg_pending <= 1'b1;
        end else if (msg_pending && fs_start) begin
            // 消息被发送状态机取走
            msg_pending <= 1'b0;
        end
    end
end

// ---------- 发送状态机 ----------
localparam S_IDLE  = 2'd0;
localparam S_SEND  = 2'd1;   // 逐字符提交，等待 uart_tx
localparam S_WAIT  = 2'd2;   // 等待 uart_tx 释放当前字符

reg [1:0]  fsm;
reg [3:0]  char_idx;    // 当前消息第几个字符（0..8）
reg        saw_busy;    // 握手：必须先见到 tx_busy=1 才能判定字节完成。
                        // 否则 S_WAIT 首拍采样到旧的 tx_busy=0（uart_tx 置忙晚一拍），
                        // 会跳字符导致丢字节（实测丢 'D'/'O'）。

// 消息长度表：kind1 = 7字节(含CRLF), kind2=8, kind3(ERR:0xXX)=10, kind4(IMG)=9
// 字符内容生成（组合逻辑，依赖 msg_kind/msg_idx/msg_total/msg_err/char_idx）
reg [7:0]  char_data;
reg        char_is_last;   // 该字符为 '\n'（消息最后一字节）

// 十六进制数字转 ASCII
function [7:0] hex_ascii;
    input [3:0] h;
    begin
        hex_ascii = (h <= 4'd9) ? (8'h30 + {4'd0, h}) : (8'h37 + {4'd0, h});
    end
endfunction

// ---------- 2026-10-21 时序优化：字符内容提前一拍预算到寄存器 ----------
// 原写法在 char_data 的组合 case 里直接调用 hex_ascii(msg_val[31:28]) 等，综合出
//   「32 位选半字节(3 级) → 转 ASCII(1 级) → 按 msg_kind 选(3 级) → 按 char_idx 选(2 级)」
// 共 9 级 LUT、10.032ns，是 100MHz 下 sd_card_clk 域的最差路径
//（-0.312ns，占 8 个违例端点中的 6 个）。
// msg_val / msg_err / msg_idx / msg_total 在一条消息发送期间保持不变，
// 因此可以把它们的 ASCII 形式提前算好存进寄存器，char_data 只需在寄存器之间做选择，
// 组合深度降到 3~4 级。
// 时序对齐：事件在 N 拍锁存（msg_* 更新）→ N+1 拍算出 ASCII → N+2 拍 S_SEND 取用，正好衔接。
// 注意：本条优化不改变任何字符内容，UART 输出文本与优化前逐字符一致。
reg [7:0] hx7, hx6, hx5, hx4, hx3, hx2, hx1, hx0;   // msg_val[31:28] ... [3:0] 的 ASCII
reg [7:0] er1, er0;                                  // msg_err[7:4] / msg_err[3:0] 的 ASCII
reg [7:0] idx_ascii, tot_ascii;                       // "IMG:x/y" 中 x / y 的 ASCII

always @(posedge clk or posedge rst) begin
    if (rst) begin
        hx7 <= 8'h30;  hx6 <= 8'h30;  hx5 <= 8'h30;  hx4 <= 8'h30;
        hx3 <= 8'h30;  hx2 <= 8'h30;  hx1 <= 8'h30;  hx0 <= 8'h30;
        er1 <= 8'h30;  er0 <= 8'h30;
        idx_ascii <= 8'h31;
        tot_ascii <= 8'h30;
    end else begin
        hx7 <= hex_ascii(msg_val[31:28]);
        hx6 <= hex_ascii(msg_val[27:24]);
        hx5 <= hex_ascii(msg_val[23:20]);
        hx4 <= hex_ascii(msg_val[19:16]);
        hx3 <= hex_ascii(msg_val[15:12]);
        hx2 <= hex_ascii(msg_val[11:8]);
        hx1 <= hex_ascii(msg_val[7:4]);
        hx0 <= hex_ascii(msg_val[3:0]);
        er1 <= hex_ascii(msg_err[7:4]);
        er0 <= hex_ascii(msg_err[3:0]);
        idx_ascii <= 8'h30 + {4'd0, msg_idx} + 8'd1;   // 图片编号从 1 开始显示
        tot_ascii <= 8'h30 + {5'd0, msg_total};
    end
end

always @(*) begin
    char_data  = 8'h0A;
    char_is_last = 1'b1;
    case (msg_kind)
        4'd1: begin   // "SD OK\r\n"
            case (char_idx)
                4'd0: char_data = "S";
                4'd1: char_data = "D";
                4'd2: char_data = " ";
                4'd3: char_data = "O";
                4'd4: char_data = "K";
                4'd5: char_data = 8'h0D;
                4'd6: char_data = 8'h0A;
                default: char_data = 8'h0A;
            endcase
            char_is_last = (char_idx == 4'd6);
        end
        4'd2: begin   // "SD OUT\r\n"
            case (char_idx)
                4'd0: char_data = "S";
                4'd1: char_data = "D";
                4'd2: char_data = " ";
                4'd3: char_data = "O";
                4'd4: char_data = "U";
                4'd5: char_data = "T";
                4'd6: char_data = 8'h0D;
                4'd7: char_data = 8'h0A;
                default: char_data = 8'h0A;
            endcase
            char_is_last = (char_idx == 4'd7);
        end
        4'd3: begin   // "ERR:0x31\r\n" ~ "ERR:0x38\r\n"
            case (char_idx)
                4'd0: char_data = "E";
                4'd1: char_data = "R";
                4'd2: char_data = "R";
                4'd3: char_data = ":";
                4'd4: char_data = "0";
                4'd5: char_data = "x";
                4'd6: char_data = er1;   // 高4位十六进制（已预算，见 er1/er0 寄存器）
                4'd7: char_data = er0;   // 低4位十六进制
                4'd8: char_data = 8'h0D;
                4'd9: char_data = 8'h0A;
                default: char_data = 8'h0A;
            endcase
            char_is_last = (char_idx == 4'd9);
        end
        4'd4: begin   // "IMG:" + (idx+1) + "/" + total + "\r\n"
            case (char_idx)
                4'd0: char_data = "I";
                4'd1: char_data = "M";
                4'd2: char_data = "G";
                4'd3: char_data = ":";
                4'd4: char_data = idx_ascii;   // (idx+1) 的 ASCII，已预算
                4'd5: char_data = "/";
                4'd6: char_data = tot_ascii;   // total 的 ASCII，已预算
                4'd7: char_data = 8'h0D;
                4'd8: char_data = 8'h0A;
                default: char_data = 8'h0A;
            endcase
            char_is_last = (char_idx == 4'd8);
        end
        4'd5: begin   // "WR:3S\r\n" —— 3s兜底：SD读完成但SDRAM写完成信号丢失
            case (char_idx)
                4'd0: char_data = "W";
                4'd1: char_data = "R";
                4'd2: char_data = ":";
                4'd3: char_data = "3";
                4'd4: char_data = "S";
                4'd5: char_data = 8'h0D;
                4'd6: char_data = 8'h0A;
                default: char_data = 8'h0A;
            endcase
            char_is_last = (char_idx == 4'd6);
        end
        4'd6: begin   // "RD:3S\r\n" —— 3s兜底：SD读未完成
            case (char_idx)
                4'd0: char_data = "R";
                4'd1: char_data = "D";
                4'd2: char_data = ":";
                4'd3: char_data = "3";
                4'd4: char_data = "S";
                4'd5: char_data = 8'h0D;
                4'd6: char_data = 8'h0A;
                default: char_data = 8'h0A;
            endcase
            char_is_last = (char_idx == 4'd6);
        end
        4'd7: begin   // "DS:XXXXXXXX\r\n" —— FAT解析出的数据区起始扇区（8位十六进制）
            case (char_idx)
                4'd0 : char_data = "D";
                4'd1 : char_data = "S";
                4'd2 : char_data = ":";
                4'd3 : char_data = hx7;    // msg_val[31:28] 的 ASCII，已预算
                4'd4 : char_data = hx6;
                4'd5 : char_data = hx5;
                4'd6 : char_data = hx4;
                4'd7 : char_data = hx3;
                4'd8 : char_data = hx2;
                4'd9 : char_data = hx1;
                4'd10: char_data = hx0;    // msg_val[3:0]
                4'd11: char_data = 8'h0D;
                4'd12: char_data = 8'h0A;
                default: char_data = 8'h0A;
            endcase
            char_is_last = (char_idx == 4'd12);
        end
        4'd8: begin   // "DS:FAIL\r\n" —— FAT解析失败（将回退从扇区0扫描）
            case (char_idx)
                4'd0: char_data = "D";
                4'd1: char_data = "S";
                4'd2: char_data = ":";
                4'd3: char_data = "F";
                4'd4: char_data = "A";
                4'd5: char_data = "I";
                4'd6: char_data = "L";
                4'd7: char_data = 8'h0D;
                4'd8: char_data = 8'h0A;
                default: char_data = 8'h0A;
            endcase
            char_is_last = (char_idx == 4'd8);
        end
        4'd9: begin   // "S1:XXXXXXXX\r\n" —— 首张好图所在扇区（定位文件真实位置）
            case (char_idx)
                4'd0 : char_data = "S";
                4'd1 : char_data = "1";
                4'd2 : char_data = ":";
                4'd3 : char_data = hx7;    // msg_val[31:28] 的 ASCII，已预算
                4'd4 : char_data = hx6;
                4'd5 : char_data = hx5;
                4'd6 : char_data = hx4;
                4'd7 : char_data = hx3;
                4'd8 : char_data = hx2;
                4'd9 : char_data = hx1;
                4'd10: char_data = hx0;    // msg_val[3:0]
                4'd11: char_data = 8'h0D;
                4'd12: char_data = 8'h0A;
                default: char_data = 8'h0A;
            endcase
            char_is_last = (char_idx == 4'd12);
        end
        4'd10: begin  // "S0:XXXXXXXX\r\n" —— 本轮扫描0图收场，终止于该扇区
            case (char_idx)
                4'd0 : char_data = "S";
                4'd1 : char_data = "0";
                4'd2 : char_data = ":";
                4'd3 : char_data = hx7;    // msg_val[31:28] 的 ASCII，已预算
                4'd4 : char_data = hx6;
                4'd5 : char_data = hx5;
                4'd6 : char_data = hx4;
                4'd7 : char_data = hx3;
                4'd8 : char_data = hx2;
                4'd9 : char_data = hx1;
                4'd10: char_data = hx0;    // msg_val[3:0]
                4'd11: char_data = 8'h0D;
                4'd12: char_data = 8'h0A;
                default: char_data = 8'h0A;
            endcase
            char_is_last = (char_idx == 4'd12);
        end
        default: begin
            char_data   = 8'h0A;
            char_is_last = 1'b1;
        end
    endcase
end

reg [7:0]  tx_byte;
reg        tx_start;
wire       tx_busy;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        fsm      <= S_IDLE;
        char_idx <= 4'd0;
        tx_byte  <= 8'd0;
        tx_start <= 1'b0;
        fs_start <= 1'b0;
        saw_busy <= 1'b0;
    end else begin
        tx_start <= 1'b0;
        fs_start <= 1'b0;
        case (fsm)
            S_IDLE: begin
                if (msg_pending) begin
                    char_idx <= 4'd0;
                    fs_start <= 1'b1;       // 告知事件锁存器取走消息
                    fsm      <= S_SEND;
                end
            end
            S_SEND: begin
                // 提交当前字符（组合生成的 char_data 此时对应 char_idx）
                tx_byte  <= char_data;
                tx_start <= 1'b1;
                saw_busy <= 1'b0;
                fsm      <= S_WAIT;
            end
            S_WAIT: begin
                if (tx_busy) begin
                    saw_busy <= 1'b1;       // uart_tx 已接受本字节
                end else if (saw_busy) begin
                    // 本字节已完整发送
                    if (char_is_last) begin
                        fsm <= S_IDLE;       // 本消息结束
                    end else begin
                        char_idx <= char_idx + 4'd1;
                        fsm      <= S_SEND;
                    end
                end
                // else: tx_start 还未被 uart_tx 采到（tx_busy 尚未拉高），继续等
            end
            default: fsm <= S_IDLE;
        endcase
    end
end

uart_tx #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .BAUD_RATE   (BAUD_RATE)
) u_uart_tx (
    .clk      (clk),
    .rst      (rst),
    .tx_data  (tx_byte),
    .tx_start (tx_start),
    .tx_busy  (tx_busy),
    .uart_tx  (uart_tx)
);

endmodule
