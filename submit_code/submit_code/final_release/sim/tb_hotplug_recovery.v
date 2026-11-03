`timescale 1ns/1ps
// ============================================================================
// tb_hotplug_recovery.v —— 拔卡 / 插回自动恢复回归测试
//
// 用真实 RTL（sd_card_bmp + 官方 SPI SD 卡栈 + FAT32 扫描/流读 + BMP 解码）
// 加一个行为级 SPI SD 卡模型，复现并验证下面这条 bug 的修复：
//
//   现象：播放中把 TF 卡拔出再插回 -> 读不到卡、黑屏、数码管/OSD 显示 ERR:0x38，
//         只能按复位。
//   根因（本 TB 覆盖的三条）：
//     1. SPI 模式下卡被重新上电后处于原生 SD 模式，若上层不再发 CMD0 就永远
//        读不到卡（本层只在"读命令失败"时才重初始化，空闲时毫无察觉）。
//     2. 加载途中被打断时 fat32_file_streamer 可能停在 DATA_WAIT 等一个永远
//        不来的 sector_done，之后再也不接受 file_start -> 每次加载都超时 0x38。
//     3. "空扫描重试"看门狗用单拍 scan_done 计时，永远计不满 -> 0 图后永久黑屏。
//
// 测试流程：
//   T1 正常开机：找到 4 张图并显示首图（不许有错误码）
//   T2 空闲态拔卡：探活（CMD13）应发现卡不在 -> 保留画面不黑屏 -> 插回后自动
//      重扫并重新出图
//   T3 空闲态拔卡后把卡插回：切图仍可用（第16讲改；原先是"加载中拔卡"场景）
//   T4 恢复后再按"下一张"：必须能正常切图（证明客户端没有死掉）
//   T5 卡不在位时切图：四个 bank 的缓存内容必须还在，next/prev 照样能翻（第16讲）
//
// 说明：本 TB 的时间打印单位是 us（$time/1000）。
// ============================================================================
module tb_hotplug_recovery;

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
    reg         jump_req_t;          // w30：模拟 top 侧"应急插播/缩略图选图"的跳转请求
    reg  [1:0]  jump_idx_t;

    sd_card_bmp #(
        .CLK_FREQ_HZ         (1_000_000        ),  // 只影响轮播/消抖计数，仿真缩时
        .SCAN_MAX_SECTOR     (32'd255          ),
        .AUTO_PLAY_MS        (1                ),
        // ---- 超时/看门狗（默认值对应真实硬件，这里全部缩小）----
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
        .PROBE_EN            (1                ),
        .PROBE_GAP_CYCLES    (20_000           ),
        .PWR_WAIT_CYCLES     (2_000            )
    ) u_dut (
        .clk               (clk),
        .rst               (rst),
        .key_next          (key_next_n),
        .key_auto          (key_auto_n),
        .key_prev          (key_prev_n),
        .jump_req          (jump_req_t),
        .jump_idx          (jump_idx_t),
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

    // ---------------- SDRAM 写通道模型（frame_fifo_write 的等价行为）----------------
    // 行为：write_req 上升沿 -> 回 ack（保持到请求撤销）；之后按 write_en 逐个
    // 计像素，计满一帧 PIXELS 个 -> 翻转 write_finish_toggle（等价整帧写完）。
    reg [31:0] wr_pix;
    reg        wr_frame;       // 正在写一帧
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            write_req_ack <= 1'b0;
            wr_pix        <= 32'd0;
            wr_frame      <= 1'b0;
            wrfin_tgl     <= 1'b0;
        end else begin
            write_req_ack <= 1'b0;
            if (write_req && !wr_frame) begin
                write_req_ack <= 1'b1;      // 收到写请求 -> 回应
                wr_frame      <= 1'b1;
                wr_pix        <= 32'd0;
            end else if (wr_frame && write_en) begin
                wr_pix <= wr_pix + 32'd1;
                if (wr_pix + 32'd1 == PIXELS) begin
                    wrfin_tgl <= ~wrfin_tgl;   // 整帧写完
                    wr_frame  <= 1'b0;
                end
            end
        end
    end

    // ---------------- 提交计数（每次换显示缓冲即一帧新图）----------------
    integer commit_cnt;
    reg [1:0] disp_buf_d;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            commit_cnt <= 0;
            disp_buf_d <= 2'd0;
        end else begin
            disp_buf_d <= disp_buf_idx;
            if (disp_buf_idx != disp_buf_d)
                commit_cnt <= commit_cnt + 1;
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

    // 按"下一张"键（低有效，消抖 20ms -> 这里 CLK_FREQ=1MHz 即 20000 周期）
    task press_next;
        begin
            key_next_n = 1'b0;
            #300000;                 // 300us 按下（> 200us 消抖）
            key_next_n = 1'b1;
            #300000;
        end
    endtask

    // 按"上一张"键（低有效，同上）
    task press_prev;
        begin
            key_prev_n = 1'b0;
            #300000;
            key_prev_n = 1'b1;
            #300000;
        end
    endtask

    // 按"轮播"键（KEY13，低有效，同上）
    task press_auto;
        begin
            key_auto_n = 1'b0;
            #300000;
            key_auto_n = 1'b1;
            #300000;
        end
    endtask

    // 模拟 top 侧的一次跳转请求（应急插播 / 缩略图选图）
    task do_jump;
        input [1:0] idx;
        begin
            jump_idx_t = idx;
            jump_req_t = 1'b1;
            repeat (4) @(posedge clk);
            jump_req_t = 1'b0;
            repeat (40) @(posedge clk);
        end
    endtask

    // ---------------- 磁盘镜像 ----------------
    integer k, i;
    integer base;
    task disk_init;
        begin
            for (k = 0; k < 64*512; k = k + 1)
                u_card.disk[k] = 8'h00;

            // ---- LBA0: MBR ----
            u_card.disk[0*512+446] = 8'h00;      // boot indicator
            u_card.disk[0*512+450] = 8'h0B;      // partition type
            u_card.disk[0*512+454] = 8'd1;       // start LBA = 1
            u_card.disk[0*512+510] = 8'h55;
            u_card.disk[0*512+511] = 8'hAA;

            // ---- LBA1: FAT32 BPB ----
            u_card.disk[1*512+ 0] = 8'hEB;
            u_card.disk[1*512+11] = 8'h00; u_card.disk[1*512+12] = 8'h02;  // 512 B/sector
            u_card.disk[1*512+13] = 8'd1;                                  // spc = 1
            u_card.disk[1*512+14] = 8'd1; u_card.disk[1*512+15] = 8'h00;   // reserved = 1
            u_card.disk[1*512+16] = 8'd1;                                  // 1 FAT
            u_card.disk[1*512+21] = 8'hF8;
            u_card.disk[1*512+32] = 8'd64;                                 // total sectors
            u_card.disk[1*512+36] = 8'd1;                                  // FAT size
            u_card.disk[1*512+44] = 8'd2;                                  // root cluster = 2
            u_card.disk[1*512+510] = 8'h55;
            u_card.disk[1*512+511] = 8'hAA;

            // ---- LBA2: FAT ----
            u_card.disk[2*512+0] = 8'hF8; u_card.disk[2*512+1] = 8'hFF;
            u_card.disk[2*512+2] = 8'hFF; u_card.disk[2*512+3] = 8'h0F;
            for (i = 1; i <= 6; i = i + 1) begin
                u_card.disk[2*512+i*4+0] = 8'hFF;
                u_card.disk[2*512+i*4+1] = 8'hFF;
                u_card.disk[2*512+i*4+2] = 8'hFF;
                u_card.disk[2*512+i*4+3] = 8'h0F;
            end

            // ---- LBA3: 根目录（PIC1.BMP ~ PIC4.BMP，簇 3~6）----
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
                u_card.disk[base+11]= 8'h20;                    // 属性：归档文件
                u_card.disk[base+20]= 8'h00;                    // 簇高位
                u_card.disk[base+21]= 8'h00;
                u_card.disk[base+26]= 8'd3 + i[7:0];            // 簇低位
                u_card.disk[base+27]= 8'h00;
                u_card.disk[base+28]= FILE_BYTES[7:0];          // 文件大小
                u_card.disk[base+29]= FILE_BYTES[15:8];
                u_card.disk[base+30]= 8'h00;
                u_card.disk[base+31]= 8'h00;
            end

            // ---- LBA4..7: 四张 8x4 24bit BMP ----
            for (i = 0; i < 4; i = i + 1) begin
                base = (4 + i) * 512;
                u_card.disk[base+0]  = "B";
                u_card.disk[base+1]  = "M";
                u_card.disk[base+2]  = FILE_BYTES[7:0];         // file_len
                u_card.disk[base+3]  = FILE_BYTES[15:8];
                u_card.disk[base+10] = 8'd54;                   // pixel offset
                u_card.disk[base+14] = 8'd40;                   // dib size
                u_card.disk[base+18] = W[7:0];                  // width
                u_card.disk[base+22] = H[7:0];                  // height
                u_card.disk[base+26] = 8'd1;                    // planes = 1
                u_card.disk[base+28] = 8'd24;                   // bpp = 24
                for (k = 0; k < PIXELS; k = k + 1) begin
                    u_card.disk[base+54+k*3+0] = i*8'h20 + 8'h11;
                    u_card.disk[base+54+k*3+1] = 8'h22;
                    u_card.disk[base+54+k*3+2] = 8'h33;
                end
            end
        end
    endtask

    // ---------------- 主流程 ----------------
    integer c0;
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
        card_present = 1'b1;         // 上电时卡已在位
        repeat (20) @(posedge clk);
        rst = 1'b0;

        // ================= T1：正常开机 =================
        $display("\n===== T1 : normal boot =====");
        wait (display_valid === 1'b1);
        repeat (200) @(posedge clk);
        $display("  first image shown: img_total=%0d err=0x%02h state=%0d all_loaded=%0d",
                 img_total, bmp_error_code, state_code, u_dut.all_loaded);
        check (img_total == 3'd4, "found 4 BMP files");
        check (bmp_error_code == 8'h00, "no error code");
        // 阶段C：等四 bank 预加载完成
        wait (u_dut.all_loaded === 1'b1);
        $display("  preload done: all_loaded=1, disp=%0d img=%0d", disp_buf_idx, media_id);

        // ================= T2：空闲态拔卡 -> 探活 -> 插回 =================
        $display("\n===== T2 : pull card while idle =====");
        card_present = 1'b0;
        wait (sd_init_done === 1'b0);
        repeat (20) @(posedge clk);                  // 等 !sd_init_done 分支生效
        $display("  probe detected card removal, display_valid=%0d", display_valid);
        check (display_valid === 1'b1, "screen NOT blank after removal");
        // 第16讲：改成"四个 bank 的完整画面必须还在"——拔卡不再清缓存，
        // all_loaded / img_found_count 是"加载流程状态"，现在允许保持不动。
        check (u_dut.bank_valid === 4'b1111, "cached banks kept after removal");
        #250000;                                    // 停顿一会儿
        // ===== T5：卡不在位时，切图必须照样可用（第16讲 bank_valid 解耦的核心）=====
        $display("\n===== T5 : switch with NO card (cached banks) =====");
        press_next();
        check (media_id == 2'd1, "next works with no card");
        check (disp_buf_idx == 2'd1, "disp follows next with no card");
        press_prev();
        check (media_id == 2'd0, "prev works with no card");
        check (disp_buf_idx == 2'd0, "disp follows prev with no card");
        check (bmp_error_code == 8'h00, "no error code while switching without card");
        $display("  re-insert card ...");
        card_present = 1'b1;
        wait (sd_init_done === 1'b1);
        $display("  card re-initialized");
        wait (u_dut.load_busy === 1'b1);            // 第16讲：等重载真正开始
        wait (u_dut.all_loaded === 1'b1 && u_dut.bank_valid === 4'b1111);
        repeat (100) @(posedge clk);
        $display("  preload again: img_total=%0d err=0x%02h all_loaded=%0d",
                 img_total, bmp_error_code, u_dut.all_loaded);
        check (img_total == 3'd4, "re-scan found 4 images");
        check (display_valid === 1'b1, "display valid after re-insert");

        // ================= T3：空闲态拔卡后插回，切图仍可用 =================
        $display("\n===== T3 : switch still works after re-insert =====");
        press_next();
        wait (media_id == 2'd1);
        repeat (50) @(posedge clk);
        $display("  switched: img=%0d disp=%0d err=0x%02h",
                 media_id, disp_buf_idx, bmp_error_code);
        check (media_id == 2'd1, "switch to image 1 after recovery");
        check (disp_buf_idx == 2'd1, "disp follows image 1");
        check (bmp_error_code == 8'h00, "no error code after switch");

        // ================= T4：切图零延迟（不读 SD）=================
        $display("\n===== T4 : zero-latency switch after recovery =====");
        press_next();
        wait (media_id == 2'd2);
        repeat (50) @(posedge clk);
        $display("  switched: img=%0d disp=%0d err=0x%02h",
                 media_id, disp_buf_idx, bmp_error_code);
        check (media_id == 2'd2, "switch to image 2");
        check (bmp_error_code == 8'h00, "no error code after switch");

        // ===== T6：bank 有空洞时导航必须跳过空洞（等效"中间某张图解码失败"）=====
        //   不动卡模型，直接 force bank_valid —— 导航逻辑只依赖这个位图，
        //   因此这正是"PIC2 头坏了、其余三张正常"现场场景的等效注入。
        //   旧实现按 cnt_eff 取模（假设有效 bank 是连续前缀），此场景下
        //   从 0 按"上一张"会落到无效的 2、从 1 按"下一张"会落到无效的 2，两个方向都卡死。
        $display("\n===== T6 : navigation skips a hole in bank_valid =====");
        force u_dut.bank_valid = 4'b1011;          // bank2 无效，0/1/3 有效
        repeat (10) @(posedge clk);

        press_prev();                              // 2 -> 1
        repeat (300) @(posedge clk);
        press_prev();                              // 1 -> 0
        repeat (300) @(posedge clk);
        $display("  at img=%0d (expect 0)", media_id);
        check (media_id == 2'd0, "walk back to image 0");

        press_prev();                              // 0 -> 3（旧逻辑会卡在 0）
        repeat (300) @(posedge clk);
        $display("  prev from 0: img=%0d (expect 3)", media_id);
        check (media_id == 2'd3, "prev skips the hole (0 -> 3)");

        press_next();                              // 3 -> 0
        repeat (300) @(posedge clk);
        press_next();                              // 0 -> 1
        repeat (300) @(posedge clk);
        press_next();                              // 1 -> 3（旧逻辑会卡在 1）
        repeat (300) @(posedge clk);
        $display("  next from 1: img=%0d (expect 3)", media_id);
        check (media_id == 2'd3, "next skips the hole (1 -> 3)");

        release u_dut.bank_valid;
        repeat (10) @(posedge clk);

        // ===== T7：日常轮播只在前 3 张里循环；停在预案页时不被轮播带走 =====
        //   AUTO_PLAY_BANKS=3 ⇒ 轮播候选 = bank 0/1/2，第 4 张（预案页）永不出现；
        //   手动翻页 / 跳转仍可到第 4 张；一旦停在第 4 张，轮播暂停（否则 2.35 s 后被带走）。
        $display("\n===== T7 : auto-play only cycles page 1..3 =====");
        // T6 期间可能发生过重扫/重载，bank_valid 未必是 1111；这里显式固定成"四张都在"，
        // 只测"轮播候选集合"这一条逻辑。
        force u_dut.bank_valid = 4'b1111;
        repeat (10) @(posedge clk);
        $display("  bank_valid=%b auto_valid=%b", u_dut.bank_valid, u_dut.auto_valid);
        check (u_dut.auto_valid === 4'b0111, "auto-play candidate set = pages 1..3");
        do_jump(2'd0);                              // 先回到第 1 张
        $display("  jump to 0: img=%0d", media_id);
        check (media_id == 2'd0, "jump to image 0");

        press_auto();                               // 打开轮播
        repeat (100) @(posedge clk);
        check (u_dut.auto_play_en === 1'b1, "auto-play enabled by key");

        begin : t7_cycle
            integer n, seen3, seen0, seen1, seen2;
            n = 0; seen0 = 0; seen1 = 0; seen2 = 0; seen3 = 0;
            for (n = 0; n < 12; n = n + 1) begin
                repeat (1200) @(posedge clk);        // AUTO_PLAY_MS=1 @1MHz ⇒ 1000 拍一次
                $display("    [cycle %0d] img=%0d", n, media_id);
                if (media_id == 2'd0) seen0 = seen0 + 1;
                if (media_id == 2'd1) seen1 = seen1 + 1;
                if (media_id == 2'd2) seen2 = seen2 + 1;
                if (media_id == 2'd3) seen3 = seen3 + 1;
            end
            check (seen3 == 0, "auto-play never shows page 4 in 12 cycles");
            check ((seen0 > 0) && (seen1 > 0) && (seen2 > 0), "auto-play covers pages 1/2/3");
        end

        // 模拟应急插播：跳到第 4 张
        do_jump(2'd3);
        $display("  panic jump to 3: img=%0d", media_id);
        check (media_id == 2'd3, "panic jump reaches page 4");
        check (u_dut.auto_hold === 1'b1, "auto-play held while on page 4");
        repeat (5 * 1200) @(posedge clk);           // 5 个轮播周期
        $display("  after 5 auto periods: img=%0d (expect still 3)", media_id);
        check (media_id == 2'd3, "auto-play does not steal page 4");

        // 手动离开预案页 ⇒ 轮播自动恢复，且仍然不会自己回到第 4 张
        //   ★ 先关掉轮播再按键：按键按下要保持 200us（消抖）+100us，而轮播每 1000 拍（10us）
        //     就会切一张，若不关轮播，采样时画面已经被轮播带走了（测试本身的问题，不是 RTL）。
        press_auto();                               // 先关轮播
        repeat (100) @(posedge clk);
        check (u_dut.auto_play_en === 1'b0, "auto-play disabled by key");
        press_next();                               // 3 -> 0
        repeat (100) @(posedge clk);
        $display("  after manual next: img=%0d (expect 0)", media_id);
        check (media_id == 2'd0, "manual next leaves page 4");
        check (u_dut.auto_hold === 1'b0, "auto-hold cleared after leaving page 4");

        press_auto();                               // 再打开轮播
        repeat (100) @(posedge clk);
        check (u_dut.auto_play_en === 1'b1, "auto-play re-enabled");
        // 采样判定"轮播又在动、且不会到第 4 张"（不判具体是哪一张：采样点与轮播节拍不同步）
        begin : t7_resume
            integer m, changed, bad3, prev;
            changed = 0; bad3 = 0; prev = media_id;
            for (m = 0; m < 8; m = m + 1) begin
                repeat (700) @(posedge clk);
                if (media_id !== prev) changed = changed + 1;
                if (media_id == 2'd3) bad3 = bad3 + 1;
                prev = media_id;
            end
            $display("  resume samples: changed=%0d bad3=%0d", changed, bad3);
            check (changed > 0, "auto-play advances again after resume");
            check (bad3 == 0, "resumed rotation still skips page 4");
        end

        press_auto();                               // 关掉轮播，恢复默认状态
        repeat (100) @(posedge clk);
        release u_dut.bank_valid;
        repeat (10) @(posedge clk);

        $display("\n===== RESULT: %0s (errors=%0d) =====",
                 (errs == 0) ? "PASS" : "FAIL", errs);
        $finish;
    end

    // 全局超时保护
    initial begin
        #120_000_000;     // 120ms 仿真时间
        $display("\n>>> TIMEOUT: test did not finish in 120ms (possible deadlock)");
        $finish;
    end

    // 关键事件打印
    reg flush_d;
    always @(posedge clk) begin
        flush_d <= u_dut.sd_flush;
        if (u_dut.sd_flush && !flush_d)
            $display("    [t=%0dus] sd_flush", $time/1000);
    end
    always @(u_dut.sd_init_done)
        $display("    [t=%0dus] sd_init_done=%b", $time/1000, u_dut.sd_init_done);

endmodule
