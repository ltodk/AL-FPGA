`timescale 1ns/1ps
// =============================================================================
// tb_write_two_frames —— 连续两帧的完整提交链路验证（第8讲）
//
// 为什么需要它：
//   实机上"首图能出、按切换报 0x38（切不了图）"。首帧与次帧的差别只有
//   image_index(bank) 和 CDC/loader 的历史状态，单帧用例覆盖不到。
//   这里用真实 RTL（loader + CDC + media_write_commit）连跑两帧，
//   直接观察：load_done 是否两次都来、memory_error 是否都是 0、
//   frame_write_toggle 是否恰好翻两次（sd_card_bmp 就靠它判写完）。
//
// 覆盖的历史坑：
//   · 待翻计数每拍翻转 → cnt=2 时互相抵消 → 上层收不到写完成
//   · cdc_idle 同步旧值导致 toggle 提前翻转
//   · 读写无仲裁（本 tb 里 dst_rd_busy 固定 0，仲裁由 tb_wr_arbiter 覆盖）
// =============================================================================
module tb_write_two_frames;

    localparam integer W     = 640;
    localparam integer H     = 480;
    localparam integer WORDS = W * H;

    reg sd_clk  = 1'b0;
    reg dst_clk = 1'b0;
    reg rst_n   = 1'b0;

    always #5.000 sd_clk  = ~sd_clk;    // 100 MHz
    always #4.000 dst_clk = ~dst_clk;   // 125 MHz

    // ---------------- DUT 接口 ----------------
    reg         load_start  = 1'b0;
    reg  [1:0]  image_index = 2'd0;
    reg         pixel_valid = 1'b0;
    reg  [23:0] pixel_rgb   = 24'd0;

    wire        load_busy, load_done;
    wire [3:0]  image_complete;
    wire [7:0]  memory_error;

    wire        ldr_wr_en;
    wire [20:0] ldr_wr_addr;
    wire [3:0]  ldr_wr_mask;
    wire [31:0] ldr_wr_data;

    wire        ldr_pixel_ready;
    wire        cdc_full;
    wire        cdc_overflow;
    wire        cdc_wr_en;
    wire [20:0] cdc_wr_addr;
    wire [3:0]  cdc_wr_mask;
    wire [31:0] cdc_wr_data;
    wire        cdc_idle;

    wire        frame_commit;
    wire        frame_toggle;

    // CDC 排空同步回 100M 域（与顶层一致）
    reg cdc_idle_s1, cdc_idle_s2;
    always @(posedge sd_clk or negedge rst_n) begin
        if (!rst_n) begin
            cdc_idle_s1 <= 1'b0;
            cdc_idle_s2 <= 1'b0;
        end else begin
            cdc_idle_s1 <= cdc_idle;
            cdc_idle_s2 <= cdc_idle_s1;
        end
    end

    // ---------------- DUT 例化 ----------------
    media_frame_loader #(
        .ADDR_WIDTH    (21),
        .IMAGE_WORDS   (WORDS),
        .FIFO_DEPTH    (512),
        .BURST_SIZE    (128),
        .STALL_TIMEOUT (65535),
        .WRITE_V_FLIP  (1),
        .FRAME_WIDTH   (W),
        .FRAME_HEIGHT  (H)
    ) u_loader (
        .clk                 (sd_clk),
        .rst_n               (rst_n),
        .load_start          (load_start),
        .image_index         (image_index),
        .pixel_valid         (pixel_valid),
        .pixel_rgb           (pixel_rgb),
        .frame_commit        (frame_commit),
        .decode_error        (8'd0),
        .load_busy           (load_busy),
        .load_done           (load_done),
        .image_complete      (image_complete),
        .memory_error        (memory_error),
        .controller_init_done(1'b1),
        .controller_busy     (1'b0),
        .refresh_active      (1'b0),
        .app_wr_en           (ldr_wr_en),
        .app_wr_addr         (ldr_wr_addr),
        .app_wr_mask         (ldr_wr_mask),
        .app_wr_data         (ldr_wr_data),
        .out_full            (cdc_full),             // 下游 CDC 满则暂停出队
        .pixel_ready         (ldr_pixel_ready)       // 反压：本 tb 必须遵守
    );

    media_sdram_write_cdc #(
        .ADDR_WIDTH      (21),
        .FIFO_ADDR_WIDTH (9),
        .BURST_SIZE      (128)
    ) u_cdc (
        .src_clk         (sd_clk),
        .src_rst_n       (rst_n),
        .src_wr_en       (ldr_wr_en),
        .src_wr_addr     (ldr_wr_addr),
        .src_wr_mask     (ldr_wr_mask),
        .src_wr_data     (ldr_wr_data),
        .src_full        (cdc_full),
        .src_overflow    (cdc_overflow),
        .src_frame_start (load_start),
        .dst_clk         (dst_clk),
        .dst_rst_n       (rst_n),
        .dst_app_wr_en   (cdc_wr_en),
        .dst_app_wr_addr (cdc_wr_addr),
        .dst_app_wr_mask (cdc_wr_mask),
        .dst_app_wr_data (cdc_wr_data),
        .dst_idle        (cdc_idle),
        .dst_rd_busy     (1'b0),
        .dst_ref_vld  (1'b0),
        .dst_blanking     (1'b1),          // new: tb 里恒为消隐期(老行为)
        .dst_wr_hold      (1'b0),          // 2026-10-29 端口（老 tb 里没接，必须补）
        .dst_wr_active    (),
        .dst_wr_busy     (), .guard_dis (1'b0)
    );

    media_write_commit #(
        .FRAME_PIXELS       (19'd307200),
        .DRAIN_GUARD_CYCLES (8'd255)
    ) u_commit (
        .clk                (sd_clk),
        .rst_n              (rst_n),
        .load_start         (load_start),
        .pixel_valid        (pixel_valid),
        .load_done          (load_done),
        .memory_error       (memory_error),
        .cdc_idle           (cdc_idle_s2),
        .frame_commit       (frame_commit),
        .frame_write_toggle (frame_toggle)
    );

    // ---------------- 监视 ----------------
    integer frame_writes [0:3];     // 每 bank 的写入次数
    integer bank0_oob, bank1_oob;
    integer toggle_flips = 0;
    reg     toggle_d = 1'b0;
    integer errs = 0;

    initial begin
        frame_writes[0] = 0;
        frame_writes[1] = 0;
        frame_writes[2] = 0;
        frame_writes[3] = 0;
        bank0_oob = 0;
        bank1_oob = 0;
    end

    always @(posedge dst_clk) begin
        if (rst_n && cdc_wr_en) begin
            if (cdc_wr_addr < WORDS)
                frame_writes[0] = frame_writes[0] + 1;
            else if (cdc_wr_addr < 2 * WORDS)
                frame_writes[1] = frame_writes[1] + 1;
            else
                bank1_oob = bank1_oob + 1;
        end
    end

    always @(posedge sd_clk) begin
        if (rst_n) begin
            toggle_d <= frame_toggle;
            if (frame_toggle != toggle_d) begin
                toggle_flips = toggle_flips + 1;
                $display("  toggle flip #%0d @ %0t", toggle_flips, $time);
            end
        end
    end

    // ---------------- 驱动 ----------------
    integer n, f;
    integer done_cnt;

    task run_frame(input [1:0] bank);
        integer k;
        begin
            $display("---- frame for bank %0d ----", bank);
            @(posedge sd_clk);
            load_start  <= 1'b1;
            image_index <= bank;
            @(posedge sd_clk);
            load_start  <= 1'b0;
            repeat (5) @(posedge sd_clk);

            // 像素流：每 4 拍一个，模拟 SD 卡真实字节节奏
            // ★ 必须遵守 pixel_ready：写通路的吞吐只有 128 字 / 640 拍（让位窗口
            //   512 拍），比"每 4 拍一个"更慢。早先不看 ready 硬灌 → loader FIFO
            //   溢出丢像素 → pixels_received != 307200 → ERR_SHORT_FRAME(0x45)，
            //   正是实机"首图能出、切图报 0x38"的同一类问题。
            for (k = 0; k < WORDS; k = k + 1) begin
                while (!ldr_pixel_ready) @(posedge sd_clk);
                pixel_valid <= 1'b1;
                pixel_rgb   <= {(k >> 8) & 8'hFF, k & 8'hFF, ((k >> 4) ^ k) & 8'hFF};
                @(posedge sd_clk);
                pixel_valid <= 1'b0;
                repeat (3) @(posedge sd_clk);
            end

            // 等 loader 完成（单拍脉冲，用计数器锁存）
            done_cnt = 0;
            for (k = 0; k < 20000 && done_cnt == 0; k = k + 1) begin
                @(posedge sd_clk);
                if (load_done) done_cnt = done_cnt + 1;
            end
            if (done_cnt == 0) begin
                $display("  [FAIL] bank %0d: load_done 没来", bank);
                errs = errs + 1;
            end else begin
                if (memory_error != 8'd0) begin
                    $display("  [FAIL] bank %0d: memory_error=%02h", bank, memory_error);
                    errs = errs + 1;
                end else begin
                    $display("  [PASS] bank %0d: load_done, memory_error=0", bank);
                end
            end

            // 等 CDC 排空
            for (k = 0; k < 200000 && !cdc_idle; k = k + 1) @(posedge dst_clk);
            repeat (600) @(posedge sd_clk);   // 覆盖 255 拍 guard + 同步
            $display("  bank %0d: cdc_idle=%b, toggle=%b", bank, cdc_idle, frame_toggle);
        end
    endtask

    initial begin
        repeat (10) @(posedge sd_clk);
        rst_n = 1'b1;
        repeat (10) @(posedge sd_clk);

        run_frame(2'd0);
        $display("  帧1后 toggle_flips=%0d", toggle_flips);
        run_frame(2'd1);
        $display("  帧2后 toggle_flips=%0d", toggle_flips);

        $display("=========================================================");
        $display(" bank0 写入字数 : %0d (期望 %0d)", frame_writes[0], WORDS);
        $display(" bank1 写入字数 : %0d (期望 %0d)", frame_writes[1], WORDS);
        $display(" 越界写入       : %0d", bank1_oob);
        $display(" toggle 翻转次数: %0d (期望 2)", toggle_flips);
        if (frame_writes[0] != WORDS) begin
            $display(" [FAIL] bank0 字数不符"); errs = errs + 1;
        end
        if (frame_writes[1] != WORDS) begin
            $display(" [FAIL] bank1 字数不符"); errs = errs + 1;
        end
        if (bank1_oob != 0) begin
            $display(" [FAIL] 越界写入"); errs = errs + 1;
        end
        if (toggle_flips != 2) begin
            $display(" [FAIL] toggle 没有恰好翻两次 —— 上层会报 0x38"); errs = errs + 1;
        end
        if (errs == 0)
            $display(" RESULT: PASS  (两帧都完成 / 两 bank 各 307200 字 / toggle 翻 2 次)");
        else
            $display(" RESULT: FAIL  错误数 %0d", errs);
        $display("=========================================================");
        $finish;
    end

    initial begin
        #500_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
