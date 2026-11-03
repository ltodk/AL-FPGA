`timescale 1ns/1ps
// 完整链路仿真：color_bar + video_timing_data + video_delay + media_hdmi_audio_bridge
// 用真实 RTL，验证音频是否正常
module tb_full_chain;

    reg clk = 0;
    reg rst = 0;   // 高有效复位（color_bar 用高有效）

    always #20 clk = ~clk;

    // ---- color_bar 输出 ----
    wire cb_hs, cb_vs, cb_de;
    wire [11:0] cb_h_cnt;
    wire [7:0] cb_r, cb_g, cb_b;

    // ---- video_timing_data 输出 ----
    wire vt_hs, vt_vs, vt_de;
    wire [11:0] vt_h_cnt;
    wire vt_read_req;

    // ---- video_delay 输出 ----
    wire vd_hs, vd_vs, vd_de;
    wire [11:0] vd_h_cnt;
    wire [23:0] vd_data;

    // ---- 顶层信号 ----
    wire [11:0] video_x = vd_h_cnt;
    wire [9:0] video_x_ref = (video_x >= 12'd160) ? (video_x[9:0] - 10'd160)
                                                   : (video_x[9:0] + 10'd640);

    reg [23:0] video_rgb = 24'h123456;
    reg [1:0] media_id = 0;
    reg [7:0] volume = 128;

    wire [9:0] ch0, ch1, ch2;
    wire seq_err, tone_ovf, blip, pcm_tap_valid;
    wire signed [15:0] pcm_tap_left;

    // color_bar（真实代码）
    color_bar u_cb (
        .clk(clk), .rst(rst),
        .hs(cb_hs), .vs(cb_vs), .de(cb_de), .h_cnt_out(cb_h_cnt),
        .rgb_r(cb_r), .rgb_g(cb_g), .rgb_b(cb_b)
    );

    // video_timing_data（真实代码）
    video_timing_data u_vt (
        .video_clk(clk), .rst(rst),
        .read_req(vt_read_req), .read_req_ack(1'b1),
        .hs(vt_hs), .vs(vt_vs), .de(vt_de), .h_cnt_out(vt_h_cnt)
    );

    // video_delay（真实代码）
    video_delay u_vd (
        .video_clk(clk), .rst(rst),
        .read_en(), .read_data(24'hABCDEF),
        .hs(vt_hs), .vs(vt_vs), .de(vt_de), .h_cnt_in(vt_h_cnt),
        .hs_r(vd_hs), .vs_r(vd_vs), .de_r(vd_de), .h_cnt_out(vd_h_cnt),
        .vout_data(vd_data)
    );

    // bridge
    media_hdmi_audio_bridge u_bridge (
        .pixel_clk(clk), .pixel_rst_n(~rst),
        .video_hsync(vd_hs), .video_vsync(vd_vs), .video_de(vd_de),
        .video_x(video_x_ref), .video_rgb(video_rgb),
        .media_id(media_id), .volume(volume),
        .channel0_symbol(ch0), .channel1_symbol(ch1), .channel2_symbol(ch2),
        .sequence_error(seq_err), .tone_overflow(tone_ovf),
        .pcm_contract_error(), .blip_active(blip),
        .pcm_tap_valid(pcm_tap_valid), .pcm_tap_left(pcm_tap_left)
    );

    integer sample_consumed = 0;
    always @(posedge clk) begin
        if (!rst && pcm_tap_valid) sample_consumed = sample_consumed + 1;
    end

    initial begin
        $dumpfile("tb_full_chain.vcd");
        $dumpvars(0, tb_full_chain);
        rst = 1;
        repeat (10) @(posedge clk);
        rst = 0;

        // 跑 2 帧
        repeat (800*525*2) @(posedge clk);

        $display("=== 完整链路结果 ===");
        $display("sample_consumed = %0d", sample_consumed);
        $display("sequence_error  = %b", seq_err);
        $display("tone_overflow   = %b", tone_ovf);
        $display("blip_active     = %b", blip);

        if (sample_consumed == 0)
            $display(">>> 失败：完整链路无样本消费");
        else
            $display(">>> 成功：完整链路音频正常（%0d 样本）", sample_consumed);

        $finish;
    end

    initial begin
        #100000000; $display("TIMEOUT"); $finish;
    end

endmodule
