// tb_osd_img: 验证有卡状态下 IMG 行是否在 img_idx=0~3 都渲染
// 直接驱动 osd_overlay（简化：手工造 de/vs/video_x 时序）
`timescale 1ns/1ps

module tb_osd_img;

reg clk = 0;
reg rst = 1;
always #20 clk = ~clk;   // 25MHz

// 模拟 640x480 时序：行 800 clk，帧 525 行
reg de, vs;
reg [9:0] x;             // video_x 0~799
reg [11:0] hcnt;         // 0~799
reg [9:0] vcnt;          // 0~524

reg [1:0] img_idx;
reg [2:0] img_total;

wire [23:0] vout;
osd_overlay u_osd (
    .clk(clk), .rst(rst),
    .de(de), .vs(vs), .video_x(x),
    .video_in(24'd0),
    .sd_init_done(1'b1),
    .img_idx(img_idx), .img_total(img_total), .bmp_err_code(3'd0),
    .video_out(vout)
);

// 时序生成
always @(posedge clk) begin
    if (rst) begin
        hcnt <= 0; vcnt <= 0;
    end else begin
        if (hcnt == 12'd799) begin
            hcnt <= 0;
            if (vcnt == 10'd524) vcnt <= 0;
            else vcnt <= vcnt + 1;
        end else
            hcnt <= hcnt + 1;
    end
end

// de: active 区域 hcnt 160~799, vcnt 0~479
always @(*) begin
    de = (hcnt >= 12'd160) && (vcnt < 10'd480);
    // vs 低有效：vsync 期间 vcnt 490~491
    vs = ~((vcnt >= 10'd490) && (vcnt <= 10'd491));
    // video_x_ref: active 0~639, 消隐 640~799
    x  = (hcnt >= 12'd160) ? (hcnt[9:0] - 10'd160) : (hcnt[9:0] + 10'd640);
end

// 检测 IMG 行是否渲染（在 line 26~41, x 8~64 区域）
// 统计每帧中 IMG 行区域 osd_active 的像素数
integer osd_px;
integer frame_cnt;
integer imgs_rendered [0:3];  // 每个 img_idx 是否渲染过 IMG 行
reg vs_d;

always @(posedge clk) begin
    vs_d <= vs;
    if (rst) begin
        osd_px <= 0; frame_cnt <= 0;
        imgs_rendered[0] <= 0; imgs_rendered[1] <= 0;
        imgs_rendered[2] <= 0; imgs_rendered[3] <= 0;
    end else begin
        if (u_osd.osd_active) osd_px <= osd_px + 1;
        // vs 上升沿（场结束）
        if (vs && !vs_d) begin
            $display("FRAME %0d: img_idx=%0d total=%0d osd_active_px=%0d",
                     frame_cnt, img_idx, img_total, osd_px);
            if (osd_px > 100) imgs_rendered[img_idx] <= 1;
            osd_px <= 0;
            frame_cnt <= frame_cnt + 1;
        end
    end
end

initial begin
    rst = 1;
    img_idx = 0; img_total = 4;
    repeat (10) @(posedge clk);
    rst = 0;

    // 每帧 800*525 = 420000 clk。每个 img_idx 保持 5 帧，充分覆盖。
    img_idx = 0;
    repeat (420000*5) @(posedge clk);  // 第1张 img_idx=0
    img_idx = 1;
    repeat (420000*5) @(posedge clk);  // 第2张 img_idx=1
    img_idx = 2;
    repeat (420000*5) @(posedge clk);  // 第3张 img_idx=2
    img_idx = 3;
    repeat (420000*5) @(posedge clk);  // 第4张 img_idx=3

    $display("=== SUMMARY: idx0=%0d idx1=%0d idx2=%0d idx3=%0d ===",
             imgs_rendered[0], imgs_rendered[1], imgs_rendered[2], imgs_rendered[3]);
    $finish;
end

endmodule
