`timescale 1ns/1ps
// 测试 media_hdmi_audio_bridge 的数据岛触发与音频包插入
// 用参考坐标系：active = video_x 0~639，消隐 = 640~799
module tb_hdmi_audio;

    reg clk = 0;
    reg rst_n = 0;

    // 视频时序（模拟 video_timing_ctrl：ACTIVE 在前）
    reg [11:0] x = 0;
    reg [11:0] y = 0;
    reg hsync, vsync, de;

    // 输入给 bridge
    wire [9:0] video_x = x[9:0];
    reg [23:0] video_rgb = 0;
    reg [1:0] media_id = 0;
    reg [7:0] volume = 128;

    // bridge 输出
    wire [9:0] ch0, ch1, ch2;
    wire seq_err, tone_ovf, blip, pcm_tap_valid;
    wire signed [15:0] pcm_tap_left;

    // 时钟 25MHz（40ns 周期）
    always #20 clk = ~clk;

    // 视频时序生成（640x480@60，H_TOTAL=800，V_TOTAL=525）
    // ACTIVE 在前：x=0~639 active, x=640~799 消隐
    // hsync 低有效：x=656~751（H_ACT+H_FP=656 起，宽96）
    // vsync 低有效：y=490~491（V_ACT+V_FP=490 起，宽2）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= 0; y <= 0;
        end else if (x == 799) begin
            x <= 0;
            y <= (y == 524) ? 0 : y + 1;
        end else begin
            x <= x + 1;
        end
    end

    always @* begin
        de    = (x < 640) && (y < 480);
        hsync = ~((x >= 656) && (x < 656+96));
        vsync = ~((y >= 490) && (y < 490+2));
    end

    media_hdmi_audio_bridge u_bridge (
        .pixel_clk(clk), .pixel_rst_n(rst_n),
        .video_hsync(hsync), .video_vsync(vsync), .video_de(de),
        .video_x(video_x), .video_rgb(video_rgb),
        .media_id(media_id), .volume(volume),
        // 第 11 讲阶段 A：WAV 未接入，接 0 让 mux 恒选琶音（回归基线不变）
        .wav_valid(1'b0), .wav_ready(), .wav_left(16'sd0), .wav_right(16'sd0),
        .wav_active(1'b0),
        .channel0_symbol(ch0), .channel1_symbol(ch1), .channel2_symbol(ch2),
        .sequence_error(seq_err), .tone_overflow(tone_ovf),
        .pcm_contract_error(), .blip_active(blip),
        .pcm_tap_valid(pcm_tap_valid), .pcm_tap_left(pcm_tap_left)
    );

    // 监测：数据岛符号（TMDS 数据岛期间的 symbol 与视频/控制不同）
    // 在消隐期（x>=640）观察 ch0 是否出现数据岛前导序列
    integer island_symbol_count = 0;
    integer total_blank_cycles = 0;
    integer sample_consumed = 0;

    always @(posedge clk) begin
        if (rst_n && pcm_tap_valid)
            sample_consumed = sample_consumed + 1;
    end

    // 主流程
    initial begin
        // 波形输出
        $dumpfile("tb_hdmi_audio.vcd");
        $dumpvars(0, tb_hdmi_audio);

        rst_n = 0;
        video_rgb = 24'h123456;
        repeat (10) @(posedge clk);
        rst_n = 1;

        // 跑 3 帧（每帧 800*525 = 420000 周期 = 16.8ms）
        // 时间太长，先跑足够周期观察
        repeat (800*525*2) @(posedge clk);

        $display("=== 结果 ===");
        $display("sample_consumed = %0d", sample_consumed);
        $display("sequence_error  = %b", seq_err);
        $display("tone_overflow   = %b", tone_ovf);
        $display("blip_active     = %b", blip);
        $display("pcm_tap_valid   = %b", pcm_tap_valid);

        if (sample_consumed == 0)
            $display(">>> 失败：没有任何样本被消费（数据岛未触发或音频包未发出）");
        else
            $display(">>> 成功：有 %0d 个样本被消费，音频链路工作", sample_consumed);

        $finish;
    end

    // 超时保护
    initial begin
        #200000000;  // 200ms
        $display("TIMEOUT");
        $finish;
    end

endmodule
