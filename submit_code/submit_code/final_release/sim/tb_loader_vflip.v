`timescale 1ns/1ps
// =============================================================================
// tb_loader_vflip —— 验证 media_frame_loader 的垂直翻转地址映射
//
// 用小尺寸（W=4, H=3, 每帧 12 字）跑整帧，便于逐字核对地址序列：
//   BMP 文件自下而上存储 → 第 0 个像素属于屏幕最底行（内存第 H-1 行）
//   期望写入偏移序列：8, 9,10,11, 4, 5, 6, 7, 0, 1, 2, 3
//   即每行 4 个递增，行末跳 -(2W-1) = -7
//
// 同时核对：
//   ① 写入的 32 位字是 {8'h00, pixel_rgb}（RGB 在低 24 位）
//   ② image_index=2 时基址 = 2*IMAGE_WORDS = 24
//   ③ 一整帧收完 → load_done 脉冲 + image_complete[2]=1，无错误码
//   ④ 短帧（少给像素就 commit）→ ERR_SHORT_FRAME = 0x45
// =============================================================================
module tb_loader_vflip;

localparam integer W = 4;
localparam integer H = 3;
localparam integer N = W * H;      // 12

reg         clk;
reg         rst_n;
reg         load_start;
reg  [1:0]  image_index;
reg         pixel_valid;
reg  [23:0] pixel_rgb;
reg         frame_commit;
reg  [7:0]  decode_error;

wire        load_busy;
wire        load_done;
wire [3:0]  image_complete;
wire [7:0]  memory_error;
wire        app_wr_en;
wire [20:0] app_wr_addr;
wire [3:0]  app_wr_mask;
wire [31:0] app_wr_data;

integer     errors;
integer     wr_count;
integer     addr_log [0:63];
integer     data_log [0:63];
integer     pix_idx;
integer     i;
reg  [7:0]  exp_addr [0:N-1];

media_frame_loader #(
    .ADDR_WIDTH    (21),
    .IMAGE_WORDS   (N),
    .FIFO_DEPTH    (16),
    .BURST_SIZE    (4),
    .STALL_TIMEOUT (200),
    .WRITE_V_FLIP  (1),
    .FRAME_WIDTH   (W),
    .FRAME_HEIGHT  (H)
) dut (
    .clk                 (clk),
    .rst_n               (rst_n),
    .load_start          (load_start),
    .image_index         (image_index),
    .pixel_valid         (pixel_valid),
    .pixel_rgb           (pixel_rgb),
    .frame_commit        (frame_commit),
    .decode_error        (decode_error),
    .load_busy           (load_busy),
    .load_done           (load_done),
    .image_complete      (image_complete),
    .memory_error        (memory_error),
    .controller_init_done(1'b1),
    .controller_busy     (1'b0),
    .refresh_active      (1'b0),
    .app_wr_en           (app_wr_en),
    .app_wr_addr         (app_wr_addr),
    .app_wr_mask         (app_wr_mask),
    .app_wr_data         (app_wr_data),
    .out_full            (1'b0)                  // 本 tb 直连 sink，无反压
);

always #5 clk = ~clk;

// 采集每一次写
always @(posedge clk) begin
    if (app_wr_en) begin
        addr_log[wr_count] = app_wr_addr;
        data_log[wr_count] = app_wr_data;
        wr_count = wr_count + 1;
    end
end

// 送一个像素（带 1 拍空闲，模拟解码抖动）
task send_pixel;
    input [7:0] v;
    begin
        @(negedge clk);
        pixel_valid = 1'b1;
        pixel_rgb   = {v, v + 8'd1, v + 8'd2};
        @(negedge clk);
        pixel_valid = 1'b0;
    end
endtask

initial begin
    clk          = 1'b0;
    rst_n        = 1'b1;
    load_start   = 1'b0;
    image_index  = 2'd0;
    pixel_valid  = 1'b0;
    pixel_rgb    = 24'd0;
    frame_commit = 1'b0;
    decode_error = 8'd0;
    errors       = 0;
    wr_count     = 0;
    pix_idx      = 0;

    // 期望偏移（bank 内）：8,9,10,11, 4,5,6,7, 0,1,2,3
    exp_addr[0]  = 21'd8;  exp_addr[1]  = 21'd9;
    exp_addr[2]  = 21'd10; exp_addr[3]  = 21'd11;
    exp_addr[4]  = 21'd4;  exp_addr[5]  = 21'd5;
    exp_addr[6]  = 21'd6;  exp_addr[7]  = 21'd7;
    exp_addr[8]  = 21'd0;  exp_addr[9]  = 21'd1;
    exp_addr[10] = 21'd2;  exp_addr[11] = 21'd3;

    #20;
    rst_n = 1'b0;
    #40;
    rst_n = 1'b1;
    #50;

    // ============ 用例 1：完整一帧写进 bank2 ============
    $display("---- case 1: full frame -> bank2 (V_FLIP) ----");
    image_index = 2'd2;
    @(negedge clk);
    load_start = 1'b1;
    @(negedge clk);
    load_start = 1'b0;

    for (pix_idx = 0; pix_idx < N; pix_idx = pix_idx + 1) begin
        send_pixel(pix_idx[7:0]);
        if (pix_idx == N - 1) begin
            // 最后一个像素送出后打 frame_commit
            @(negedge clk);
            frame_commit = 1'b1;
            @(negedge clk);
            frame_commit = 1'b0;
        end
    end

    // 等 load_done
    i = 0;
    while (!load_done && i < 3000) begin
        @(posedge clk);
        i = i + 1;
    end

    if (!load_done) begin
        $display("FAIL: load_done 未出现 (timeout)");
        errors = errors + 1;
    end

    if (wr_count != N) begin
        $display("FAIL: 写入字数 %0d，期望 %0d", wr_count, N);
        errors = errors + 1;
    end

    if (memory_error != 8'h00) begin
        $display("FAIL: 错误码 0x%02x，期望 0x00", memory_error);
        errors = errors + 1;
    end

    if (image_complete[2] !== 1'b1) begin
        $display("FAIL: image_complete[2]=%b，期望 1", image_complete[2]);
        errors = errors + 1;
    end

    // 逐字核对地址与数据
    for (i = 0; i < N && i < wr_count; i = i + 1) begin
        if (addr_log[i] !== (2 * N + exp_addr[i])) begin
            $display("FAIL: 第 %0d 字地址 %0d，期望 %0d",
                     i, addr_log[i], 2 * N + exp_addr[i]);
            errors = errors + 1;
        end
        if (data_log[i] !== {8'h00, i[7:0], i[7:0] + 8'd1, i[7:0] + 8'd2}) begin
            $display("FAIL: 第 %0d 字数据 0x%08x，期望 0x%08x",
                     i, data_log[i],
                     {8'h00, i[7:0], i[7:0] + 8'd1, i[7:0] + 8'd2});
            errors = errors + 1;
        end
    end
    $display("    地址序列: %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
             addr_log[0],  addr_log[1],  addr_log[2],  addr_log[3],
             addr_log[4],  addr_log[5],  addr_log[6],  addr_log[7],
             addr_log[8],  addr_log[9],  addr_log[10], addr_log[11]);

    // ============ 用例 2：短帧 → ERR_SHORT_FRAME ============
    $display("---- case 2: short frame -> ERR_SHORT_FRAME ----");
    wr_count = 0;
    image_index = 2'd1;
    @(negedge clk);
    load_start = 1'b1;
    @(negedge clk);
    load_start = 1'b0;

    for (pix_idx = 0; pix_idx < N - 2; pix_idx = pix_idx + 1)
        send_pixel(pix_idx[7:0]);

    @(negedge clk);
    frame_commit = 1'b1;
    @(negedge clk);
    frame_commit = 1'b0;

    i = 0;
    while (!load_done && i < 3000) begin
        @(posedge clk);
        i = i + 1;
    end

    if (!load_done) begin
        $display("FAIL: case2 load_done 未出现");
        errors = errors + 1;
    end
    if (memory_error !== 8'h45) begin
        $display("FAIL: case2 错误码 0x%02x，期望 0x45 (ERR_SHORT_FRAME)", memory_error);
        errors = errors + 1;
    end
    if (image_complete[1] !== 1'b0) begin
        $display("FAIL: case2 image_complete[1]=%b，期望 0", image_complete[1]);
        errors = errors + 1;
    end

    repeat (20) @(posedge clk);
    $display("========================================");
    if (errors == 0)
        $display("RESULT: PASS  (V_FLIP 地址映射 / 像素计数 / 错误码 全部正确)");
    else
        $display("RESULT: FAIL  错误数 %0d", errors);
    $display("========================================");
    $finish;
end

endmodule
