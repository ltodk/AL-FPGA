`timescale 1ns/1ps
// ============================================================================
// tb_osd_audio.v —— 第11讲阶段C：OSD 行5（音频状态）+ 行6（播放进度）验证
//
// 行5（y=80..95）：音频状态四态
//   S1  found=0                       -> "AU:NO FILE"  灰色 0x8A8A8A
//   S2  found=1, active=1, under=0    -> "AU:PLAY   "  绿色 0x00E000
//   S3  found=1, under=1              -> "AU:UNDER  "  黄色 0xFFC000
//   S4  found=1, active=0, under=0    -> "AU:READY  "  白色 0xFFFFFF
//   S5  sd_init_done=0（无卡）        -> 整行隐藏
//
// 行6（y=98..113）：播放进度
//   S1/S7  found=0                    -> 整行隐藏
//   S6     pos=3,   tot=6             -> "POS   3/  6s"
//   S8     pos=125, tot=300           -> "POS 125/300s"
//
// 做法：逐像素扫一帧，从 DUT 内部 `ascii_sel` 逐列采样实际输出的字符码，
//       拼成字符串整体比对 —— 不依赖 osd_font 的具体点阵。
// ============================================================================
module tb_osd_audio;

    reg         clk;
    reg         rst;
    reg         de;
    reg         vs;
    reg  [9:0]  x;
    wire [23:0] vout;

    reg         sd_init_done;
    reg         audio_found;
    reg         audio_active;
    reg         audio_underrun;
    reg  [9:0]  audio_pos_sec;
    reg  [9:0]  audio_tot_sec;

    osd_overlay u_osd (
        .clk            (clk),
        .rst            (rst),
        .de             (de),
        .vs             (vs),
        .video_x        (x),
        .video_in       (24'd0),
        .sd_init_done   (sd_init_done),
        .img_idx        (2'd0),
        .img_total      (3'd4),
        .bmp_err_code   (8'd0),
        .param_bright   (8'd255),
        .param_contrast (8'd128),
        .param_show     (1'b0),
        .audio_found    (audio_found),
        .audio_active   (audio_active),
        .audio_underrun (audio_underrun),
        .audio_pos_sec  (audio_pos_sec),
        .audio_tot_sec  (audio_tot_sec),
        .video_out      (vout)
    );

    initial begin
        clk = 1'b0;
        forever #20 clk = ~clk;      // 25MHz
    end

    // ---- 视频时序：扫 vcnt 0..129，覆盖 y=80..113（行5+行6）----
    reg [11:0] hcnt;
    reg [9:0]  vcnt;

    always @(posedge clk) begin
        if (rst) begin
            hcnt <= 12'd0;
            vcnt <= 10'd0;
        end else begin
            if (hcnt == 12'd799) begin
                hcnt <= 12'd0;
                if (vcnt == 10'd129) vcnt <= 10'd0;
                else                 vcnt <= vcnt + 10'd1;
            end else begin
                hcnt <= hcnt + 12'd1;
            end
        end
    end

    always @(*) begin
        de = (hcnt >= 12'd160) && (vcnt < 10'd120);
        vs = ~((vcnt >= 10'd124) && (vcnt <= 10'd125));
        x  = (hcnt >= 12'd160) ? (hcnt[9:0] - 10'd160) : (hcnt[9:0] + 10'd640);
    end

    // ---- 采样 ----
    reg [7:0]      obs5 [0:9];
    reg [7:0]      obs6 [0:11];
    reg [8*10-1:0] str5;
    reg [8*12-1:0] str6;
    reg [23:0]     obs_color;
    integer        l5_px;
    integer        l5_box_px;
    integer        l6_px;
    reg            frame_start;
    reg            vs_d;

    integer jc;
    integer jp;
    integer ji;

    always @(*) begin
        for (jc = 0; jc < 10; jc = jc + 1)
            str5[8*(9-jc) +: 8] = obs5[jc];
    end

    always @(*) begin
        for (jc = 0; jc < 12; jc = jc + 1)
            str6[8*(11-jc) +: 8] = obs6[jc];
    end

    always @(posedge clk) begin
        if (rst) begin
            for (jp = 0; jp < 10; jp = jp + 1) obs5[jp] <= 8'h20;
            for (jp = 0; jp < 12; jp = jp + 1) obs6[jp] <= 8'h20;
            obs_color <= 24'd0;
            l5_px     <= 0;
            l5_box_px <= 0;
            l6_px     <= 0;
            vs_d      <= 1'b1;
            frame_start <= 1'b0;
        end else begin
            vs_d        <= vs;
            frame_start <= (vs_d && !vs);

            if (vs_d && !vs) begin
                l5_px     <= 0;
                l5_box_px <= 0;
                l6_px     <= 0;
            end

            // 行5 带内逐列抓字符码
            if (u_osd.in_l5 && (x >= 10'd8) && (x < 10'd88) && de)
                obs5[(x - 10'd8) >> 3] <= u_osd.ascii_sel;
            // 行6 带内逐列抓字符码
            if (u_osd.in_l6 && (x >= 10'd8) && (x < 10'd104) && de)
                obs6[(x - 10'd8) >> 3] <= u_osd.ascii_sel;

            if (u_osd.in_l5 && u_osd.osd_active) begin
                obs_color <= u_osd.osd_color;
                l5_px     <= l5_px + 1;
            end
            if (u_osd.in_l5 && u_osd.box_on)  l5_box_px <= l5_box_px + 1;
            if (u_osd.in_l6 && u_osd.osd_active) l6_px <= l6_px + 1;
        end
    end

    // ---- 检查工具 ----
    integer errors;

    task check_str5;
        input [8*10-1:0] label;
        input [8*10-1:0] exp;
        begin
            if (str5 === exp) $display("  PASS  %0s", label);
            else begin
                $display("  FAIL  %0s  got=\"%0s\" exp=\"%0s\"", label, str5, exp);
                errors = errors + 1;
            end
        end
    endtask

    task check_str6;
        input [8*10-1:0] label;
        input [8*12-1:0] exp;
        begin
            if (str6 === exp) $display("  PASS  %0s", label);
            else begin
                $display("  FAIL  %0s  got=\"%0s\" exp=\"%0s\"", label, str6, exp);
                errors = errors + 1;
            end
        end
    endtask

    task check_color;
        input [8*10-1:0] label;
        input [23:0]     exp;
        begin
            if (obs_color === exp) $display("  PASS  %0s color=%06h", label, obs_color);
            else begin
                $display("  FAIL  %0s color=%06h exp=%06h", label, obs_color, exp);
                errors = errors + 1;
            end
        end
    endtask

    task check_int;
        input [8*10-1:0] label;
        input integer    got;
        input integer    exp;
        begin
            if (got === exp) $display("  PASS  %0s = %0d", label, got);
            else begin
                $display("  FAIL  %0s = %0d exp %0d", label, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    task clear_obs;
        begin
            for (ji = 0; ji < 10; ji = ji + 1) obs5[ji] = 8'h20;
            for (ji = 0; ji < 12; ji = ji + 1) obs6[ji] = 8'h20;
            obs_color = 24'd0;
        end
    endtask

    task run_frames;
        input integer cnt;
        integer f;
        integer g;
        reg     seen;
        begin
            for (f = 0; f < cnt; f = f + 1) begin
                seen = 1'b0;
                g = 0;
                while (!seen && g < 200000) begin
                    @(posedge clk);
                    if (frame_start) seen = 1'b1;
                    g = g + 1;
                end
                g = 0;
                while (!(vcnt == 10'd122 && hcnt == 12'd799) && g < 200000) begin
                    @(posedge clk);
                    g = g + 1;
                end
            end
        end
    endtask

    // ========================================================================
    initial begin
        errors         = 0;
        rst            = 1'b1;
        sd_init_done   = 1'b1;
        audio_found    = 1'b0;
        audio_active   = 1'b0;
        audio_underrun = 1'b0;
        audio_pos_sec  = 10'd0;
        audio_tot_sec  = 10'd0;

        repeat (10) @(posedge clk);
        rst = 1'b0;
        repeat (10) @(posedge clk);

        // ---------------- S1：无音频文件 ----------------
        $display("");
        $display("===== S1  found=0  ->  AU:NO FILE (GREY) + line6 hidden =====");
        sd_init_done = 1'b1; audio_found = 1'b0;
        audio_active = 1'b0; audio_underrun = 1'b0;
        audio_pos_sec = 10'd3; audio_tot_sec = 10'd6;   // 故意给值，验证会被隐藏
        repeat (6) @(posedge clk);
        clear_obs;
        run_frames(2);
        check_str5("S1 line5", "AU:NO FILE");
        check_color("S1", 24'h8A8A8A);
        check_int("S1 line5 text px>0", (l5_px > 0) ? 1 : 0, 1);
        check_int("S1 line6 text px", l6_px, 0);
        check_str6("S1 line6", "            ");

        // ---------------- S2：正常播放（6 秒素材，播到第 3 秒）----------------
        $display("");
        $display("===== S2/S6  found=1 active=1, pos=3 tot=6  ->  AU:PLAY + POS   3/  6s =====");
        audio_found = 1'b1; audio_active = 1'b1; audio_underrun = 1'b0;
        audio_pos_sec = 10'd3; audio_tot_sec = 10'd6;
        repeat (6) @(posedge clk);
        clear_obs;
        run_frames(2);
        check_str5("S2 line5", "AU:PLAY   ");
        check_color("S2", 24'h00E000);
        check_str6("S6 line6", "POS   3/  6s");
        if (l6_px > 0) $display("  PASS  S6 line6 text px = %0d", l6_px);
        else begin $display("  FAIL  S6 line6 no pixels"); errors = errors + 1; end

        // ---------------- S3：欠载补静音 ----------------
        $display("");
        $display("===== S3  found=1 underrun=1  ->  AU:UNDER (YELLOW) =====");
        audio_underrun = 1'b1;
        repeat (6) @(posedge clk);
        clear_obs;
        run_frames(2);
        check_str5("S3 line5", "AU:UNDER  ");
        check_color("S3", 24'hFFC000);

        // ---------------- S4：已找到但还没开声 ----------------
        $display("");
        $display("===== S4  found=1 active=0  ->  AU:READY (WHITE) =====");
        audio_underrun = 1'b0; audio_active = 1'b0;
        repeat (6) @(posedge clk);
        clear_obs;
        run_frames(2);
        check_str5("S4 line5", "AU:READY  ");
        check_color("S4", 24'hFFFFFF);

        // ---------------- S5：无卡，两行都隐藏 ----------------
        $display("");
        $display("===== S5  sd_init_done=0  ->  line5/line6 hidden =====");
        sd_init_done = 1'b0;
        audio_found = 1'b1; audio_active = 1'b1;
        repeat (6) @(posedge clk);
        clear_obs;
        run_frames(2);
        check_int("S5 line5 text px", l5_px, 0);
        check_int("S5 line5 box px",  l5_box_px, 0);
        check_str5("S5 line5", "          ");
        // 注意：行6 的显隐条件里没有 sd_s1，但 sd_s1=0 时 auf_s1 仍为1；
        // 本工程语义是"无卡就什么都别显示"，所以行6 也必须靠 sd_s1 关掉。
        // 这里只检查行6 不画文字像素即可（sd_s1 由 l6_show 里的 sd_s1 保证）。
        check_int("S5 line6 text px", l6_px, 0);

        // ---------------- S7：三位置进度 ----------------
        $display("");
        $display("===== S7  pos=125 tot=300  ->  POS 125/300s =====");
        sd_init_done = 1'b1;
        audio_found = 1'b1; audio_active = 1'b1; audio_underrun = 1'b0;
        audio_pos_sec = 10'd125; audio_tot_sec = 10'd300;
        repeat (6) @(posedge clk);
        clear_obs;
        run_frames(2);
        check_str6("S7 line6", "POS 125/300s");

        // ---------------- S8：个位/十位边界 ----------------
        $display("");
        $display("===== S8  pos=9 tot=10  ->  POS   9/ 10s =====");
        audio_pos_sec = 10'd9; audio_tot_sec = 10'd10;
        repeat (6) @(posedge clk);
        clear_obs;
        run_frames(2);
        check_str6("S8 line6", "POS   9/ 10s");

        // ---------------- 结束 ----------------
        $display("");
        if (errors == 0)
            $display(">>> OSD AUDIO STATUS+PROGRESS REGRESSION PASS");
        else
            $display(">>> OSD AUDIO STATUS+PROGRESS REGRESSION FAIL (%0d checks)", errors);
        $display("");
        $finish;
    end

    initial begin
        #500000000;
        $display(">>> GLOBAL TIMEOUT");
        $finish;
    end

endmodule
