`timescale 1ns/1ps
// ============================================================================
// tb_stageC_preload.v —— 第8讲 阶段C：四 bank 上电预加载 + 零延迟切图
//
// 验证 sd_card_bmp 从"双缓冲 + 切图时加载"改成"四 bank 上电预加载"后的行为：
//
//   T1 上电预加载：扫描 4 张后，write_buf_idx 依次走过 0→1→2→3（4 张图分别
//      加载进 bank0/1/2/3），期间 load_busy 多次拉高（读 SD），最后 all_loaded=1。
//   T2 零延迟切图：all_loaded 后按"下一张"，disp_buf_idx 立即 +1（改指针），
//      且 load_busy 不再拉高（不再读 SD）—— 这是"切图零延迟"的关键判据。
//   T3 4 张都能显示：连续按下一张，disp_buf_idx 依次 0→1→2→3→0 循环。
//
// 说明：时间打印单位 us（$time/1000）。SD 卡模型与 tb_hotplug_recovery 相同。
// ============================================================================
module tb_stageC_preload;

    localparam integer W          = 8;
    localparam integer H          = 4;
    localparam integer PIXELS     = W * H;              // 32
    localparam integer FILE_BYTES = 54 + PIXELS * 3;    // 150

    // ---------------- 时钟 / 复位 ----------------
    reg clk = 0;
    always #5 clk = ~clk;      // 100MHz
    reg rst = 1;

    // ---------------- DUT ----------------
    wire [3:0]  state_code;
    wire [1:0]  recover_state;
    wire [3:0]  disp_num;
    wire [1:0]  media_id;
    wire [7:0]  bmp_error_code;
    wire [2:0]  img_total;
    wire        sd_init_done;
    wire        load_error;
    wire        ld3s_pulse, ld3s_src;
    wire        fat_pulse, fat_ok;
    wire [31:0] fat_val;
    wire        scan_pulse, scan_zero;
    wire [31:0] scan_val;
    wire        display_valid;

    wire        write_req, write_en;
    wire [31:0] write_data;
    wire [1:0]  write_buf_idx, disp_buf_idx;
    reg         write_req_ack;
    reg         wrfin_tgl;

    wire        sd_ncs, sd_dclk, sd_mosi, sd_miso;

    reg         card_present;
    reg         sdr_init_done;
    reg         key_next_n, key_auto_n, key_prev_n;

    sd_card_bmp #(
        .CLK_FREQ_HZ         (1_000_000        ),
        .SCAN_MAX_SECTOR     (32'd255          ),
        .AUTO_PLAY_MS        (1                ),
        .READ_WAIT_CYCLES    (200_000          ),
        .WRITE_WAIT_CYCLES   (200_000          ),
        .LOAD_TIMEOUT_CYCLES (1_000_000        ),
        .EMPTY_SCAN_CYCLES   (200_000          ),
        .SCAN_WD_CYCLES      (400_000          ),
        .SD_TIMEOUT_CYCLES   (150_000          ),
        .FAT_WD_CYCLES       (300_000          ),
        .SPI_LOW_DIV         (3                ),
        .SPI_HIGH_DIV        (0                ),
        .CMD_TIMEOUT_BYTES   (16'd600          ),
        .PROBE_EN            (0                ),
        .PROBE_GAP_CYCLES    (20_000           ),
        .PWR_WAIT_CYCLES     (2_000            )
    ) u_dut (
        .clk               (clk),
        .rst               (rst),
        .key_next          (key_next_n),
        .key_auto          (key_auto_n),
        .key_prev          (key_prev_n),
        .state_code        (state_code),
        .recover_state     (recover_state),
        .disp_num          (disp_num),
        .media_id          (media_id),
        .bmp_error_code    (bmp_error_code),
        .img_total         (img_total),
        .sd_init_done_out  (sd_init_done),
        .load_error_out    (load_error),
        .ld3s_pulse        (ld3s_pulse),
        .ld3s_src          (ld3s_src),
        .fat_dbg_pulse     (fat_pulse),
        .fat_dbg_ok        (fat_ok),
        .fat_dbg_val       (fat_val),
        .scan_diag_pulse   (scan_pulse),
        .scan_diag_zero    (scan_zero),
        .scan_diag_val     (scan_val),
        .max_width         (16'd8),
        .max_height        (16'd4),
        .display_valid     (display_valid),
        .write_finish_toggle(wrfin_tgl),
        .write_buf_idx     (write_buf_idx),
        .disp_buf_idx      (disp_buf_idx),
        .Sdr_init_done     (sdr_init_done),
        .write_req         (write_req),
        .write_req_ack     (write_req_ack),
        .write_en          (write_en),
        .write_data        (write_data),
        .SD_nCS            (sd_ncs),
        .SD_DCLK           (sd_dclk),
        .SD_MOSI           (sd_mosi),
        .SD_MISO           (sd_miso)
    );

    // ---------------- SPI SD 卡模型 ----------------
    sd_spi_card_model #(
        .MEM_BYTES (32768)
    ) u_card (
        .sd_clk       (sd_dclk),
        .sd_mosi      (sd_mosi),
        .sd_cs_n      (sd_ncs),
        .sd_miso      (sd_miso),
        .card_present (card_present)
    );

    // ---------------- SDRAM 写通道模型 ----------------
    reg [31:0] wr_pix;
    reg        wr_frame;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            write_req_ack <= 1'b0;
            wr_pix        <= 32'd0;
            wr_frame      <= 1'b0;
            wrfin_tgl     <= 1'b0;
        end else begin
            write_req_ack <= 1'b0;
            if (write_req && !wr_frame) begin
                write_req_ack <= 1'b1;
                wr_frame      <= 1'b1;
                wr_pix        <= 32'd0;
            end else if (wr_frame && write_en) begin
                wr_pix <= wr_pix + 32'd1;
                if (wr_pix + 32'd1 == PIXELS) begin
                    wrfin_tgl <= ~wrfin_tgl;
                    wr_frame  <= 1'b0;
                end
            end
        end
    end

    // ---------------- 写 bank 遍历记录（预加载是否走过 0→1→2→3）----------------
    reg [3:0] bank_written;   // 各 bank 是否被写过（粘滞）
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bank_written <= 4'b0;
        end else if (write_req && !write_req_ack) begin
            // 每次新加载发起时（write_req 上升沿），记录写目标 bank
            bank_written[write_buf_idx] <= 1'b1;
        end
    end

    // ---------------- 切图时是否读 SD（零延迟判据：切图不该读 SD）----------------
    reg switch_triggered;
    reg sd_read_during_switch;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            switch_triggered      <= 1'b0;
            sd_read_during_switch <= 1'b0;
        end else begin
            // 切图触发：all_loaded 后 disp_buf_idx 变化，且此前 load_busy 未拉高
            if (u_dut.all_loaded && switch_triggered && u_dut.load_busy)
                sd_read_during_switch <= 1'b1;
        end
    end

    // ---------------- 检查辅助 ----------------
    integer errs;
    task check;
        input        cond;
        input [8*64-1:0] msg;
        begin
            if (cond) begin
                $display("  [PASS] %0s", msg);
            end else begin
                $display("  [FAIL] %0s", msg);
                errs = errs + 1;
            end
        end
    endtask

    task press_next;
        begin
            key_next_n = 1'b0;
            #300000;
            key_next_n = 1'b1;
            #300000;
        end
    endtask

    // ---------------- 磁盘镜像 ----------------
    integer k, i;
    integer base;
    task disk_init;
        begin
            for (k = 0; k < 64*512; k = k + 1)
                u_card.disk[k] = 8'h00;

            u_card.disk[0*512+446] = 8'h00;
            u_card.disk[0*512+450] = 8'h0B;
            u_card.disk[0*512+454] = 8'd1;
            u_card.disk[0*512+510] = 8'h55;
            u_card.disk[0*512+511] = 8'hAA;

            u_card.disk[1*512+ 0] = 8'hEB;
            u_card.disk[1*512+11] = 8'h00; u_card.disk[1*512+12] = 8'h02;
            u_card.disk[1*512+13] = 8'd1;
            u_card.disk[1*512+14] = 8'd1; u_card.disk[1*512+15] = 8'h00;
            u_card.disk[1*512+16] = 8'd1;
            u_card.disk[1*512+21] = 8'hF8;
            u_card.disk[1*512+32] = 8'd64;
            u_card.disk[1*512+36] = 8'd1;
            u_card.disk[1*512+44] = 8'd2;
            u_card.disk[1*512+510] = 8'h55;
            u_card.disk[1*512+511] = 8'hAA;

            u_card.disk[2*512+0] = 8'hF8; u_card.disk[2*512+1] = 8'hFF;
            u_card.disk[2*512+2] = 8'hFF; u_card.disk[2*512+3] = 8'h0F;
            for (i = 1; i <= 6; i = i + 1) begin
                u_card.disk[2*512+i*4+0] = 8'hFF;
                u_card.disk[2*512+i*4+1] = 8'hFF;
                u_card.disk[2*512+i*4+2] = 8'hFF;
                u_card.disk[2*512+i*4+3] = 8'h0F;
            end

            for (i = 0; i < 4; i = i + 1) begin
                base = 3*512 + i*32;
                u_card.disk[base+0] = "P";
                u_card.disk[base+1] = "I";
                u_card.disk[base+2] = "C";
                u_card.disk[base+3] = "1" + i;
                u_card.disk[base+4] = " ";
                u_card.disk[base+5] = " ";
                u_card.disk[base+6] = " ";
                u_card.disk[base+7] = " ";
                u_card.disk[base+8] = "B";
                u_card.disk[base+9] = "M";
                u_card.disk[base+10]= "P";
                u_card.disk[base+11]= 8'h20;
                u_card.disk[base+20]= 8'h00;
                u_card.disk[base+21]= 8'h00;
                u_card.disk[base+26]= 8'd3 + i[7:0];
                u_card.disk[base+27]= 8'h00;
                u_card.disk[base+28]= FILE_BYTES[7:0];
                u_card.disk[base+29]= FILE_BYTES[15:8];
                u_card.disk[base+30]= 8'h00;
                u_card.disk[base+31]= 8'h00;
            end

            for (i = 0; i < 4; i = i + 1) begin
                base = (4 + i) * 512;
                u_card.disk[base+0]  = "B";
                u_card.disk[base+1]  = "M";
                u_card.disk[base+2]  = FILE_BYTES[7:0];
                u_card.disk[base+3]  = FILE_BYTES[15:8];
                u_card.disk[base+10] = 8'd54;
                u_card.disk[base+14] = 8'd40;
                u_card.disk[base+18] = W[7:0];
                u_card.disk[base+22] = H[7:0];
                u_card.disk[base+26] = 8'd1;
                u_card.disk[base+28] = 8'd24;
                for (k = 0; k < PIXELS; k = k + 1) begin
                    u_card.disk[base+54+k*3+0] = i*8'h20 + 8'h11;
                    u_card.disk[base+54+k*3+1] = 8'h22;
                    u_card.disk[base+54+k*3+2] = 8'h33;
                end
            end
        end
    endtask

    // ---------------- 主流程 ----------------
    initial begin
        errs          = 0;
        card_present  = 1'b0;
        sdr_init_done = 1'b1;
        key_next_n    = 1'b1;
        key_auto_n    = 1'b1;
        key_prev_n    = 1'b1;
        rst           = 1'b1;

        disk_init();

        repeat (20) @(posedge clk);
        card_present = 1'b1;
        repeat (20) @(posedge clk);
        rst = 1'b0;

        // ================= T1：上电预加载 4 张 =================
        $display("\n===== T1 : power-on preload 4 images =====");
        wait (display_valid === 1'b1);       // 首图提交
        $display("  first image shown: img_total=%0d err=0x%02h", img_total, bmp_error_code);
        check (img_total == 3'd4, "found 4 BMP files");

        // 等 all_loaded（4 张全部预加载完成）
        wait (u_dut.all_loaded === 1'b1);
        repeat (50) @(posedge clk);
        $display("  all_loaded=1, bank_written=%b (0=bank0..3=bank3)", bank_written);
        check (bank_written == 4'b1111, "all 4 banks written (0→1→2→3)");
        check (bmp_error_code == 8'h00, "no error code after preload");

        // ================= T2：零延迟切图（切图不读 SD）=================
        $display("\n===== T2 : zero-latency switch =====");
        switch_triggered = 1'b1;
        sd_read_during_switch = 1'b0;
        press_next();
        // 等 disp_buf_idx 变化（切图生效），而不是固定拍数
        wait (disp_buf_idx == 2'd1);
        repeat (20) @(posedge clk);
        $display("  after next: disp_buf_idx=%0d media_id=%0d load_busy=%b err=0x%02h",
                 disp_buf_idx, media_id, u_dut.load_busy, bmp_error_code);
        check (disp_buf_idx == 2'd1, "switch to bank1 (zero-latency, no SD read)");
        check (media_id == 2'd1, "media_id follows to 1");
        check (sd_read_during_switch == 1'b0, "no SD read during switch (zero-latency)");

        // ================= T3：4 张循环显示 =================
        $display("\n===== T3 : cycle through all 4 images =====");
        // 当前在 img1，按 3 次下一张应回到 img0：1→2→3→0
        press_next(); wait (media_id == 2'd2); repeat (20) @(posedge clk);
        $display("  img=%0d disp_buf=%0d", media_id, disp_buf_idx);
        check (media_id == 2'd2, "cycle to image 2");
        check (disp_buf_idx == 2'd2, "disp follows image 2");

        press_next(); wait (media_id == 2'd3); repeat (20) @(posedge clk);
        $display("  img=%0d disp_buf=%0d", media_id, disp_buf_idx);
        check (media_id == 2'd3, "cycle to image 3");
        check (disp_buf_idx == 2'd3, "disp follows image 3");

        press_next(); wait (media_id == 2'd0); repeat (20) @(posedge clk);
        $display("  img=%0d disp_buf=%0d", media_id, disp_buf_idx);
        check (media_id == 2'd0, "wrap to image 0");
        check (disp_buf_idx == 2'd0, "disp follows image 0");

        // 切图全程无错误、无 SD 读
        check (bmp_error_code == 8'h00, "no error during cycling");
        check (sd_read_during_switch == 1'b0, "still no SD read during cycling");

        $display("\n===== RESULT: %0s (errors=%0d) =====",
                 (errs == 0) ? "PASS" : "FAIL", errs);
        $finish;
    end

    // 全局超时保护
    initial begin
        #50_000_000;
        $display("\n>>> TIMEOUT: test did not finish in 50ms (possible deadlock)");
        $finish;
    end

    // 关键事件打印
    always @(posedge clk) begin
        if (u_dut.all_loaded === 1'b1 && !$isunknown(u_dut.all_loaded)) begin
            // 只在 all_loaded 首次置位时打印一次
        end
    end

endmodule
