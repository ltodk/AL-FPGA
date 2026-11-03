`timescale 1ns/1ps
// =============================================================================
// tb_wr_cdc_chain —— 验证 100MHz loader → 125MHz CDC → SDRAM 写端口的整条链
//
// 这是阶段A最关键的一步：两个时钟频率不同、相位无关，必须确认
//   ① 数据不丢不重（写出的字数 = 帧像素数）
//   ② 地址序列在目的域与源域一致（V_FLIP 映射没有被跨域打乱）
//   ③ 数据内容逐字正确
//   ④ dst_idle 只在全部搬完之后才拉高（顶层靠它判"真正写完"）
// =============================================================================
module tb_wr_cdc_chain;

localparam integer W = 4;
localparam integer H = 3;
localparam integer N = W * H;      // 12

reg  src_clk;
reg  dst_clk;
reg  rst_n;

reg         load_start;
reg         pixel_valid;
reg  [23:0] pixel_rgb;
reg         frame_commit;

wire        load_done;
wire [3:0]  image_complete;
wire [7:0]  memory_error;
wire        src_wr_en;
wire [20:0] src_wr_addr;
wire [3:0]  src_wr_mask;
wire [31:0] src_wr_data;
wire        src_full;
wire        src_overflow;

wire        dst_app_wr_en;
wire [20:0] dst_app_wr_addr;
wire [3:0]  dst_app_wr_mask;
wire [31:0] dst_app_wr_data;
wire        dst_idle;

integer     errors;
integer     wr_count;
integer     addr_log [0:63];
integer     data_log [0:63];
integer     pix_idx;
integer     i;
reg  [7:0]  exp_addr [0:N-1];

// load_done 是 src 域单拍脉冲（10ns），在 dst 域的等待循环里可能被漏采，
// 这里在源域先锁存成电平。
reg         ld_seen;
always @(posedge src_clk) begin
    if (!rst_n)  ld_seen <= 1'b0;
    else if (load_done) ld_seen <= 1'b1;
end

// ---------------- loader（源域 100MHz）----------------
media_frame_loader #(
    .ADDR_WIDTH          (21),
    .IMAGE_WORDS         (N),
    .FIFO_DEPTH          (16),
    .BURST_SIZE          (4),
    .STALL_TIMEOUT       (4000),   // CDC 让位窗口内 FIFO 会短暂满，放宽
    .WRITE_V_FLIP        (1),
    .FRAME_WIDTH         (W),
    .FRAME_HEIGHT        (H),
    .IDLE_TIMEOUT_CYCLES (1000000)
) u_loader (
    .clk                 (src_clk),
    .rst_n               (rst_n),
    .load_start          (load_start),
    .image_index         (2'd3),
    .pixel_valid         (pixel_valid),
    .pixel_rgb           (pixel_rgb),
    .frame_commit        (frame_commit),
    .decode_error        (8'd0),
    .load_busy           (),
    .load_done           (load_done),
    .image_complete      (image_complete),
    .memory_error        (memory_error),
    .controller_init_done(1'b1),
    .controller_busy     (1'b0),
    .refresh_active      (1'b0),
    .app_wr_en           (src_wr_en),
    .app_wr_addr         (src_wr_addr),
    .app_wr_mask         (src_wr_mask),
    .app_wr_data         (src_wr_data),
    .out_full            (src_full)
);

// ---------------- CDC（100MHz → 125MHz）----------------
media_sdram_write_cdc #(
    .ADDR_WIDTH      (21),
    .FIFO_ADDR_WIDTH (4),
    .BURST_SIZE      (4)
) u_cdc (
    .src_clk         (src_clk),
    .src_rst_n       (rst_n),
    .src_wr_en       (src_wr_en),
    .src_wr_addr     (src_wr_addr),
    .src_wr_mask     (src_wr_mask),
    .src_wr_data     (src_wr_data),
    .src_full        (src_full),
    .src_overflow    (src_overflow),
    .src_frame_start (load_start),
    .dst_clk         (dst_clk),
    .dst_rst_n       (rst_n),
    .dst_app_wr_en   (dst_app_wr_en),
    .dst_app_wr_addr (dst_app_wr_addr),
    .dst_app_wr_mask (dst_app_wr_mask),
    .dst_app_wr_data (dst_app_wr_data),
    .dst_idle        (dst_idle),
    .dst_rd_busy     (1'b0),        // 本用例只测数据搬运，仲裁由 tb_wr_arbiter 覆盖
    .dst_ref_vld  (1'b0),
    .dst_blanking     (1'b1),          // new: tb 里恒为消隐期(老行为)
    .dst_wr_hold      (1'b0),          // 2026-10-29 端口（老 tb 里没接，必须补）
    .dst_wr_active    (),
    .dst_wr_busy     (), .guard_dis (1'b0)
);

always #5   src_clk = ~src_clk;    // 100 MHz
always #4   dst_clk = ~dst_clk;    // 125 MHz

// 采集目的域真正写出的每一次
always @(posedge dst_clk) begin
    if (dst_app_wr_en) begin
        addr_log[wr_count] = dst_app_wr_addr;
        data_log[wr_count] = dst_app_wr_data;
        wr_count = wr_count + 1;
    end
end

task send_pixel;
    input [7:0] v;
    begin
        @(negedge src_clk);
        pixel_valid = 1'b1;
        pixel_rgb   = {v, v + 8'd1, v + 8'd2};
        @(negedge src_clk);
        pixel_valid = 1'b0;
    end
endtask

initial begin
    src_clk      = 1'b0;
    dst_clk      = 1'b0;
    rst_n        = 1'b1;
    load_start   = 1'b0;
    pixel_valid  = 1'b0;
    pixel_rgb    = 24'd0;
    frame_commit = 1'b0;
    errors       = 0;
    wr_count     = 0;
    pix_idx      = 0;
    ld_seen      = 1'b0;

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
    #60;

    $display("---- loader(100M) -> CDC -> SDRAM(125M)，写 bank3 ----");
    @(negedge src_clk);
    load_start = 1'b1;
    @(negedge src_clk);
    load_start = 1'b0;

    for (pix_idx = 0; pix_idx < N; pix_idx = pix_idx + 1) begin
        send_pixel(pix_idx[7:0]);
        if (pix_idx == N - 1) begin
            @(negedge src_clk);
            frame_commit = 1'b1;
            @(negedge src_clk);
            frame_commit = 1'b0;
        end
    end

    // 等 dst 侧全部搬完
    i = 0;
    while (!(ld_seen && dst_idle) && i < 5000) begin
        @(posedge dst_clk);
        i = i + 1;
    end

    if (!ld_seen) begin
        $display("FAIL: load_done 未出现");
        errors = errors + 1;
    end
    if (!dst_idle) begin
        $display("FAIL: dst_idle 未拉高（CDC 没有排空）");
        errors = errors + 1;
    end
    if (src_overflow) begin
        $display("FAIL: CDC 源侧溢出");
        errors = errors + 1;
    end
    if (memory_error != 8'h00) begin
        $display("FAIL: loader 错误码 0x%02x", memory_error);
        errors = errors + 1;
    end
    if (image_complete[3] !== 1'b1) begin
        $display("FAIL: image_complete[3]=%b，期望 1", image_complete[3]);
        errors = errors + 1;
    end
    if (wr_count != N) begin
        $display("FAIL: 目的域写出 %0d 字，期望 %0d", wr_count, N);
        errors = errors + 1;
    end

    // 目的域地址序列必须和源域一致（bank3 基址 = 3*12 = 36）
    for (i = 0; i < N && i < wr_count; i = i + 1) begin
        if (addr_log[i] !== (3 * N + exp_addr[i])) begin
            $display("FAIL: 第 %0d 字地址 %0d，期望 %0d",
                     i, addr_log[i], 3 * N + exp_addr[i]);
            errors = errors + 1;
        end
        if (data_log[i] !== {8'h00, i[7:0], i[7:0] + 8'd1, i[7:0] + 8'd2}) begin
            $display("FAIL: 第 %0d 字数据 0x%08x", i, data_log[i]);
            errors = errors + 1;
        end
    end

    $display("    目的域地址序列: %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
             addr_log[0],  addr_log[1],  addr_log[2],  addr_log[3],
             addr_log[4],  addr_log[5],  addr_log[6],  addr_log[7],
             addr_log[8],  addr_log[9],  addr_log[10], addr_log[11]);

    repeat (30) @(posedge dst_clk);
    $display("========================================");
    if (errors == 0)
        $display("RESULT: PASS  (跨域数据完整 / 地址一致 / dst_idle 正确)");
    else
        $display("RESULT: FAIL  错误数 %0d", errors);
    $display("========================================");
    $finish;
end

endmodule
