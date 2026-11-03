`timescale 1ns/1ps
// ============================================================================
// sd_spi_card_model.v —— SPI 模式 SD 卡行为模型（仿真专用，不参与综合）
//
// 用途：给 sd_card_top / sd_card_bmp 做拔插卡回归仿真。支持：
//   - 标准 SPI 模式 3（CPOL=1/CPHA=1）：MOSI 在 DCLK 上升沿采样，
//     MISO 在下降沿更新
//   - CMD0/CMD8/CMD55/CMD41(ACMD41)/CMD16/CMD13/CMD17(SINGLE READ)
//   - card_present=0 时：MISO 恒为 1（等效卡被拔走 + 上拉），命令无响应
//   - 重新插入：卡片状态复位成"刚上电的原生 SD 模式"——只响应 CMD0，
//     其它命令一律不应答。这正是真实现象：拔插后 FPGA 不发 CMD0 就再也
//     读不到卡（本工程 bug 的物理层根因之一）。
//
// 磁盘镜像：disk[] 由 testbench 在 initial 里直接层次赋值
//           （u_card.disk[n] = ...），按 512 字节扇区寻址。
// ============================================================================
module sd_spi_card_model #(
    parameter integer MEM_BYTES = 32768          // 最大扇区数 * 512
)(
    input  wire sd_clk,
    input  wire sd_mosi,
    input  wire sd_cs_n,             // 片选（低有效）：真实卡只在 CS 拉低期间解帧
    output wire sd_miso,
    input  wire card_present
);

    reg [7:0] disk [0:MEM_BYTES-1];

    // ------------------------------------------------------------------
    // 接收：DCLK 上升沿采 MOSI，8 位一字节；CS 高时忽略（上电初始化字节、
    // 空闲时的时钟抖动都不会破坏字节对齐）
    // ------------------------------------------------------------------
    reg [7:0] rx_shift;
    reg [2:0] rx_bit;
    reg       byte_stb;      // 收满一字节，脉冲一拍
    reg [7:0] byte_in;

    always @(posedge sd_clk) begin
        if (sd_cs_n) begin
            rx_bit   <= 3'd0;
            byte_stb <= 1'b0;
        end else if (rx_bit == 3'd7) begin
            rx_bit   <= 3'd0;
            byte_stb <= 1'b1;
            byte_in  <= {rx_shift[6:0], sd_mosi};
        end else begin
            rx_shift <= {rx_shift[6:0], sd_mosi};
            rx_bit   <= rx_bit + 3'd1;
            byte_stb <= 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // 发送：DCLK 下降沿更新 MISO（第一个下降沿就给出 bit7）
    // ------------------------------------------------------------------
    reg [7:0] tx_byte;
    reg [2:0] tx_bit;
    reg [7:0] tx_next;
    reg       miso_i;

    always @(negedge sd_clk) begin
        if (sd_cs_n) begin
            tx_bit <= 3'd0;
        end else if (tx_bit == 3'd0) begin
            tx_byte <= tx_next;
            miso_i  <= tx_next[7];
            tx_bit  <= 3'd1;
        end else begin
            miso_i  <= tx_byte[7 - tx_bit];
            tx_bit  <= tx_bit + 3'd1;
        end
    end

    // CS 变化 = 命令边界：字节对齐复位
    always @(sd_cs_n) begin
        rx_bit   <= 3'd0;
        tx_bit   <= 3'd0;
        byte_stb <= 1'b0;
        rx_shift <= 8'hFF;
    end

    // 卡被拔走：MISO 上拉为 1
    assign sd_miso = card_present ? miso_i : 1'b1;

    // ------------------------------------------------------------------
    // 命令接收状态机
    // ------------------------------------------------------------------
    localparam S_CMD = 2'd0;
    localparam S_ARG = 2'd1;

    reg [1:0]  rx_state;
    reg [5:0]  cmd_idx;
    reg [31:0] cmd_arg;
    reg [2:0]  cnt2;          // 0=命令字节 1..4=参数 5=CRC

    // 响应状态机
    localparam R_IDLE  = 3'd0;
    localparam R_R1    = 3'd1;
    localparam R_TOKEN = 3'd2;
    localparam R_DATA  = 3'd3;
    localparam R_EXT   = 3'd4;

    reg [2:0]  rsp_state;
    reg [2:0]  rsp_plan;      // 0=无 1=仅R1 2=R1+token+512 3=R1+4字节R7
    reg [7:0]  rsp_r1;
    reg [8:0]  rsp_cnt;
    reg [31:0] rsp_lba;
    reg [2:0]  ext_cnt;
    reg        spi_mode;      // 0=原生 SD 模式（只认 CMD0）
    reg [1:0]  acmd41_try;

    // 本次要发送的字节（组合）
    always @* begin
        case (rsp_state)
            R_R1   : tx_next = rsp_r1;
            R_TOKEN: tx_next = 8'hFE;
            R_DATA : tx_next = disk[(rsp_lba * 512) + rsp_cnt];
            R_EXT  : case (ext_cnt)
                         3'd0: tx_next = 8'h00;
                         3'd1: tx_next = 8'h00;
                         3'd2: tx_next = 8'h01;
                         3'd3: tx_next = 8'hAA;
                         default: tx_next = 8'hFF;
                     endcase
            default: tx_next = 8'hFF;
        endcase
    end

    // 插拔复位：重新插入的卡 = 刚上电，处于原生 SD 模式
    always @(card_present) begin
        spi_mode    <= 1'b0;
        rx_state    <= S_CMD;
        rsp_state   <= R_IDLE;
        rsp_plan    <= 3'd0;
        acmd41_try  <= 2'd0;
        cnt2        <= 3'd0;
        tx_bit      <= 3'd0;
        rx_bit      <= 3'd0;
        byte_stb    <= 1'b0;
        rsp_cnt     <= 9'd0;
        ext_cnt     <= 3'd0;
    end

    always @(posedge sd_clk) begin
        if (byte_stb) begin
            // ---- 1) 响应字节推进（每发完一字节前进一拍）----
            case (rsp_state)
                R_R1: begin
                    if (rsp_plan == 3'd2)      rsp_state <= R_TOKEN;
                    else if (rsp_plan == 3'd3) rsp_state <= R_EXT;
                    else                       rsp_state <= R_IDLE;
                end
                R_TOKEN: begin
                    rsp_state <= R_DATA;
                    rsp_cnt   <= 9'd0;
                end
                R_DATA: begin
                    if (rsp_cnt == 9'd511) rsp_state <= R_IDLE;
                    else                   rsp_cnt   <= rsp_cnt + 9'd1;
                end
                R_EXT: begin
                    if (ext_cnt == 3'd3) rsp_state <= R_IDLE;
                    else                 ext_cnt   <= ext_cnt + 3'd1;
                end
                default: rsp_state <= R_IDLE;
            endcase

            // ---- 2) 接收并执行命令 ----
            case (rx_state)
                S_CMD: begin
                    if (byte_in[7:6] == 2'b01 && card_present) begin
                        cmd_idx  <= byte_in[5:0];
                        cmd_arg  <= 32'd0;
                        cnt2     <= 3'd0;
                        rx_state <= S_ARG;
                    end
                end
                S_ARG: begin
                    if (cnt2 < 3'd4) begin
                        cmd_arg <= {cmd_arg[23:0], byte_in};
                        cnt2    <= cnt2 + 3'd1;
                    end else begin
                        rx_state <= S_CMD;
                        cnt2     <= 3'd0;
                        // ===== 命令执行（覆盖上面的响应推进赋值）=====
                        if (cmd_idx == 6'd0) begin
                            // CMD0：进入 SPI 模式
                            spi_mode  <= 1'b1;
                            rsp_r1    <= 8'h01;
                            rsp_plan  <= 3'd1;
                            rsp_state <= R_R1;
                        end else if (!spi_mode) begin
                            // 原生 SD 模式：不响应 SPI 命令（模拟真实拔插后的现象）
                            rsp_plan  <= 3'd0;
                            rsp_state <= R_IDLE;
                        end else begin
                            case (cmd_idx)
                                6'd8: begin      // SEND_IF_COND
                                    rsp_r1   <= 8'h01;
                                    rsp_plan <= 3'd3;
                                    rsp_state<= R_R1;
                                    ext_cnt  <= 3'd0;
                                end
                                6'd55: begin     // APP_CMD
                                    rsp_r1   <= 8'h01;
                                    rsp_plan <= 3'd1;
                                    rsp_state<= R_R1;
                                end
                                6'd41: begin     // ACMD41：前两次 busy，之后好了
                                    if (acmd41_try < 2'd2) begin
                                        rsp_r1     <= 8'h01;
                                        acmd41_try <= acmd41_try + 2'd1;
                                    end else begin
                                        rsp_r1     <= 8'h00;
                                    end
                                    rsp_plan <= 3'd1;
                                    rsp_state<= R_R1;
                                end
                                6'd16: begin     // SET_BLOCKLEN
                                    rsp_r1   <= 8'h00;
                                    rsp_plan <= 3'd1;
                                    rsp_state<= R_R1;
                                end
                                6'd13: begin     // SEND_STATUS
                                    rsp_r1   <= 8'h00;
                                    rsp_plan <= 3'd1;
                                    rsp_state<= R_R1;
                                end
                                6'd17: begin     // READ_SINGLE_BLOCK
                                    rsp_r1   <= 8'h00;
                                    rsp_plan <= 3'd2;
                                    rsp_lba  <= cmd_arg;
                                    rsp_cnt  <= 9'd0;
                                    rsp_state<= R_R1;
                                end
                                default: begin
                                    rsp_r1   <= 8'h00;
                                    rsp_plan <= 3'd1;
                                    rsp_state<= R_R1;
                                end
                            endcase
                        end
                    end
                end
                default: rx_state <= S_CMD;
            endcase
        end
    end

endmodule
