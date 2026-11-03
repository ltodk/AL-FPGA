`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// rgb_video_adapter —— RGB 视频流适配器（第8讲 阶段B）
// 位于 reader（像素流输出）与 OSD / HDMI 发送之间的最后一级。
// 职责：
//   1) 整组寄存一拍：hs/vs/de/rgb 同拍寄存，让下游拿到干净的寄存器输出
//      （本工程 de/data 已由 reader 的 prefetch 对齐到同一拍，这里只做收尾寄存）。
//   2) 极性适配：本工程 color_bar 的 hs/vs 是高有效（HS/V = 1 表示同步脉冲期），
//      HDMI PHY 侧沿用高有效，故 HS/V_IN_ACTIVE_HIGH=1 直接透传，不翻转。
//   3) 消隐防护：de=0 时 rgb 强制为 0，上游脏数据出不了门。
//
// 关键：本模块不承担"20 拍对齐"——那已由 media_frame_reader 的 fifo_prefetch_2
//      取代。这里只做 1 拍寄存，与方案一"video_x 改 1 拍"对应。
// -----------------------------------------------------------------------------
module rgb_video_adapter #(
    parameter HS_IN_ACTIVE_HIGH  = 1,   // 本工程 hs 高有效
    parameter VS_IN_ACTIVE_HIGH  = 1,
    parameter HS_OUT_ACTIVE_HIGH = 1,   // 输出维持高有效
    parameter VS_OUT_ACTIVE_HIGH = 1
)(
    input  wire        pclk,       // 像素时钟（video_clk）
    input  wire        rst_n,      // 低有效复位（video_rst_n）
    input  wire        hs_in,
    input  wire        vs_in,
    input  wire        de_in,
    input  wire [23:0] rgb_in,
    output wire        video_clk,  // = pclk 直通
    output reg         video_hs,
    output reg         video_vs,
    output reg         video_de,
    output reg  [23:0] video_rgb
);

    // 归一化为"高有效"内部语义
    wire hs_act = HS_IN_ACTIVE_HIGH ? hs_in : ~hs_in;
    wire vs_act = VS_IN_ACTIVE_HIGH ? vs_in : ~vs_in;

    assign video_clk = pclk;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            video_hs  <= HS_OUT_ACTIVE_HIGH ? 1'b0 : 1'b1;
            video_vs  <= VS_OUT_ACTIVE_HIGH ? 1'b0 : 1'b1;
            video_de  <= 1'b0;
            video_rgb <= 24'h000000;
        end else begin
            video_hs  <= HS_OUT_ACTIVE_HIGH ? hs_act : ~hs_act;
            video_vs  <= VS_OUT_ACTIVE_HIGH ? vs_act : ~vs_act;
            video_de  <= de_in;
            video_rgb <= de_in ? rgb_in : 24'h000000;   // 消隐防护
        end
    end

endmodule
