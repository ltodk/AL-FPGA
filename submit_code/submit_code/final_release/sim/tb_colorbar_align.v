`timescale 1ns/1ps
// 精确模拟 color_bar（我的工程）的时序 + video_x_ref 转换 + h_cnt_d0 对齐
// 验证：我的坐标转换是否能让音频正常
module tb_colorbar_align;

    reg clk = 0;
    reg rst_n = 0;

    // ---- 模拟 color_bar 内部 ----
    // 参数：H_FP=16, H_SYNC=96, H_BP=48, H_ACTIVE=640, H_TOTAL=800
    // 顺序：FP(0~15) -> SYNC(16~111) -> BP(112~159) -> ACTIVE(160~799)
    reg [11:0] h_cnt = 0;
    reg h_active = 0;
    reg hs_reg = 0;
    reg hs_reg_d0 = 0;
    reg video_active_d0 = 0;   // de 的输出（延迟1拍）
    reg [11:0] h_cnt_d0 = 0;   // 我加的 h_cnt 延迟1拍

    wire video_active = h_active;  // 简化：假设 v_active=1

    always #20 clk = ~clk;

    // h_cnt 计数 0~799
    always @(posedge clk or posedge rst_n) begin
        if (!rst_n) h_cnt <= 0;
        else if (h_cnt == 799) h_cnt <= 0;
        else h_cnt <= h_cnt + 1;
    end

    // h_active：h_cnt==159 时置1，h_cnt==799 时置0
    always @(posedge clk or posedge rst_n) begin
        if (!rst_n) h_active <= 0;
        else if (h_cnt == 159) h_active <= 1;
        else if (h_cnt == 799) h_active <= 0;
    end

    // hs_reg：h_cnt==15 置0（HS_POL=0），h_cnt==111 置1
    always @(posedge clk or posedge rst_n) begin
        if (!rst_n) hs_reg <= 0;
        else if (h_cnt == 15) hs_reg <= 0;
        else if (h_cnt == 111) hs_reg <= 1;
    end

    // 延迟1拍：hs_reg_d0, video_active_d0(=de), h_cnt_d0
    always @(posedge clk or posedge rst_n) begin
        if (!rst_n) begin
            hs_reg_d0 <= 0;
            video_active_d0 <= 0;
            h_cnt_d0 <= 0;
        end else begin
            hs_reg_d0 <= hs_reg;
            video_active_d0 <= video_active;
            h_cnt_d0 <= h_cnt;
        end
    end

    // de / hs / video_x（顶层信号，h_cnt_out = h_cnt_d0）
    wire de = video_active_d0;
    wire hs = hs_reg_d0;
    wire vs = 1'b1;  // 简化，只看水平方向
    wire [11:0] video_x_raw = h_cnt_d0;  // 顶层 video_x（12bit，延迟1拍）

    // video_x_ref 转换（顶层）
    wire [9:0] video_x_ref = (video_x_raw >= 12'd160) ? (video_x_raw[9:0] - 10'd160)
                                                       : (video_x_raw[9:0] + 10'd640);

    reg [23:0] video_rgb = 24'h123456;
    reg [1:0] media_id = 0;
    reg [7:0] volume = 128;

    wire [9:0] ch0, ch1, ch2;
    wire seq_err, tone_ovf, blip, pcm_tap_valid;
    wire signed [15:0] pcm_tap_left;

    media_hdmi_audio_bridge u_bridge (
        .pixel_clk(clk), .pixel_rst_n(rst_n),
        .video_hsync(hs), .video_vsync(vs), .video_de(de),
        .video_x(video_x_ref), .video_rgb(video_rgb),
        .media_id(media_id), .volume(volume),
        .channel0_symbol(ch0), .channel1_symbol(ch1), .channel2_symbol(ch2),
        .sequence_error(seq_err), .tone_overflow(tone_ovf),
        .pcm_contract_error(), .blip_active(blip),
        .pcm_tap_valid(pcm_tap_valid), .pcm_tap_left(pcm_tap_left)
    );

    integer sample_consumed = 0;
    always @(posedge clk) begin
        if (rst_n && pcm_tap_valid) sample_consumed = sample_consumed + 1;
    end

    // 打印坐标关键点
    initial begin
        $dumpfile("tb_colorbar_align.vcd");
        $dumpvars(0, tb_colorbar_align);
        rst_n = 0;
        repeat (10) @(posedge clk);
        rst_n = 1;

        // 观察一行的坐标映射
        // 等待到 h_cnt 走过几个关键点
        repeat (800*4) @(posedge clk);

        $display("=== 坐标映射验证 ===");
        $display("sample_consumed = %0d", sample_consumed);
        $display("sequence_error  = %b", seq_err);
        $display("tone_overflow   = %b", tone_ovf);
        $display("blip_active     = %b", blip);

        if (sample_consumed == 0)
            $display(">>> 失败：我的坐标转换下无样本消费");
        else
            $display(">>> 成功：我的坐标转换下音频正常（%0d 样本）", sample_consumed);

        $finish;
    end

    initial begin
        #50000000; $display("TIMEOUT"); $finish;
    end

endmodule
