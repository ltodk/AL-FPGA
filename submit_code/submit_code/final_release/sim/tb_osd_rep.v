// tb_osd_rep: 复现 OSD 水平/垂直重复现象
// 链路: color_bar -> video_delay -> video_x_ref 映射 -> osd_overlay
`timescale 1ns/1ps
`define VIDEO_640_480

module tb_osd_rep;

reg clk = 0;
reg rst = 1;
always #20 clk = ~clk;   // ~25MHz, 周期比不影响结论

// DUT 前级
wire hs_c, vs_c, de_c;
wire [11:0] hcnt_c;
wire [7:0] r_c, g_c, b_c;

color_bar #(
    .H_ACTIVE(16'd640), .H_FP(16'd16), .H_SYNC(16'd96), .H_BP(16'd48),
    .V_ACTIVE(16'd480), .V_FP(16'd10), .V_SYNC(16'd2),  .V_BP(16'd33),
    .HS_POL(1'b0), .VS_POL(1'b0)
) u_color_bar (
    .clk(clk), .rst(rst),
    .hs(hs_c), .vs(vs_c), .de(de_c), .h_cnt_out(hcnt_c),
    .rgb_r(r_c), .rgb_g(g_c), .rgb_b(b_c)
);

// video_delay
wire hs_r, vs_r, de_r;
wire [11:0] video_x;
wire [23:0] vout_data_raw;

video_delay #(.DATA_WIDTH(24)) u_delay (
    .video_clk(clk), .rst(rst),
    .read_en(), .read_data({r_c, g_c, b_c}),
    .hs(hs_c), .vs(vs_c), .de(de_c), .h_cnt_in(hcnt_c),
    .hs_r(hs_r), .vs_r(vs_r), .de_r(de_r),
    .h_cnt_out(video_x), .vout_data(vout_data_raw)
);

// 顶层 video_x_ref 映射
wire [9:0] video_x_ref;
assign video_x_ref = (video_x >= 12'd160) ? (video_x[9:0] - 10'd160)
                                         : (video_x[9:0] + 10'd640);

// OSD（无卡状态: sd_init_done=0）
wire [23:0] vout_osd;
osd_overlay u_osd (
    .clk(clk), .rst(rst),
    .de(de_r), .vs(vs_r), .video_x(video_x_ref),
    .video_in(24'd0),
    .sd_init_done(1'b0),
    .img_idx(2'd0), .img_total(3'd0), .bmp_err_code(3'd0),
    .video_out(vout_osd)
);

// 内部信号观察
wire [9:0] dbg_line = u_osd.line_cnt;

// ---------- 检测 ----------
integer run_cnt;        // 当前行 osd_active 游程数
integer last_run_x;     // 上一个游程起点
integer total_runs;     // 整帧游程总数
integer prev_line_runs;
reg de_r_d;
reg in_active;          // 上一拍是否在游程中
reg [9:0] run_start_x;
integer frame_lines_with_osd;
reg [9:0] first_osd_line, second_osd_line;

always @(posedge clk) begin
    de_r_d <= de_r;
    if (rst) begin
        run_cnt <= 0; total_runs <= 0; in_active <= 0;
        frame_lines_with_osd <= 0;
    end else begin
        // de 下降沿: 一行结束
        if (de_r_d && !de_r) begin
            if (run_cnt > 0) begin
                total_runs <= total_runs + run_cnt;
                frame_lines_with_osd <= frame_lines_with_osd + 1;
                if (frame_lines_with_osd == 0) first_osd_line <= dbg_line;
                if (frame_lines_with_osd == 1) second_osd_line <= dbg_line;
                $display("[LINE end] line=%0d runs=%0d", dbg_line, run_cnt);
            end
            run_cnt <= 0;
        end
        // 游程检测
        if (u_osd.osd_active && !in_active) begin
            run_cnt <= run_cnt + 1;
            run_start_x <= video_x_ref;
            $display("  run start: line=%0d x_ref=%0d t=%0t", dbg_line, video_x_ref, $time);
        end
        in_active <= u_osd.osd_active;
    end
end

reg vs_d2;
always @(posedge clk) vs_d2 <= vs_r;
always @(posedge clk) begin
    if (vs_r && !vs_d2) begin
        $display("=== FRAME END: total_runs=%0d lines_with_osd=%0d first_line=%0d second_line=%0d ===",
                 total_runs, frame_lines_with_osd, first_osd_line, second_osd_line);
        total_runs <= 0;
        frame_lines_with_osd <= 0;
    end
end

initial begin
    rst = 1;
    repeat (10) @(posedge clk);
    rst = 0;
    // 跑 2 帧 + 余量 (525 行 * 800 像素 * 2 ≈ 840000 周期)
    # (850000 * 40);
    $display("=== SIM DONE ===");
    $finish;
end

endmodule
