`timescale 1ns/1ps
// ============================================================================
// tb_fat32_scan_multi.v —— 第 11 讲阶段 C：根目录多扇区扫描验证
//
// 背景：原 scanner 只读根目录**第一个扇区**（16 个目录项），卡上文件一多，
//       MUSIC.PCM 掉出前 16 项就再也找不到。本次扩展为读满根目录第一个簇，
//       并在见到 0x00 空闲项时提前收尾。
//
// 覆盖 4 个子用例：
//   M1  spc=8、MUSIC.PCM 在**第 40 项**（第 3 个扇区）→ 必须能跨扇区找到
//   M2  spc=8、第 5 项是 0x00 结束标记 → 必须只读 1 个扇区就收尾
//   M3  spc=8、卡上没有音频 → 必须读满 8 个扇区、resource_valid=4'hf、audio_valid=0、
//       且 fs_error=0（缺音频不算扫描失败）
//   M4  spc=2 簇边界 → MUSIC.PCM 在**第 20 项**（第 2 个扇区），必须读到第 2 扇区就停
//
// 关键断言：读出扇区数 rd_cnt。这是"到底扫了几个扇区"最直接的证据，
//           比只看结果对不对更能证明多扇区逻辑真的在跑。
//           ★ rd_cnt 统计的是**全部** CMD17 次数，前面还有 2 次固定开销
//             （读 LBA0 判 MBR + 读引导扇区取 BPB），所以
//             根目录扇区数 = rd_cnt - 2。断言的期望值都写成 2+N 的形式。
// ============================================================================
module tb_fat32_scan_multi;

    // ---- DUT 端口 ----
    reg         clk;
    reg         rst_n;
    reg         scan_start;
    wire        scan_busy;
    wire        scan_done;
    wire [7:0]  fs_error;
    wire [31:0] partition_lba;
    wire [7:0]  sectors_per_cluster;
    wire [2:0]  spc_shift_out;
    wire [31:0] fat_start_lba;
    wire [31:0] data_start_lba;
    wire [31:0] max_cluster;
    wire [3:0]  resource_valid;
    wire [31:0] resource0_cluster, resource1_cluster, resource2_cluster, resource3_cluster;
    wire [31:0] resource0_size, resource1_size, resource2_size, resource3_size;
    wire        audio_valid;
    wire [31:0] audio_cluster, audio_size;

    wire        sector_req;
    wire [31:0] sector_lba;
    reg         sector_busy;
    reg         sector_valid;
    reg  [7:0]  sector_byte;
    reg  [8:0]  sector_byte_index;
    reg         sector_done;
    wire [7:0]  sector_error;

    assign sector_error = 8'd0;

    fat32_volume_scanner u_scanner (
        .clk(clk), .rst_n(rst_n), .abort(1'b0), .scan_start(scan_start),
        .scan_busy(scan_busy), .scan_done(scan_done), .fs_error(fs_error),
        .partition_lba(partition_lba), .sectors_per_cluster(sectors_per_cluster),
        .sectors_per_cluster_shift(spc_shift_out),
        .fat_start_lba(fat_start_lba), .data_start_lba(data_start_lba),
        .max_cluster(max_cluster), .resource_valid(resource_valid),
        .resource0_cluster(resource0_cluster), .resource1_cluster(resource1_cluster),
        .resource2_cluster(resource2_cluster), .resource3_cluster(resource3_cluster),
        .resource0_size(resource0_size), .resource1_size(resource1_size),
        .resource2_size(resource2_size), .resource3_size(resource3_size),
        .audio_valid(audio_valid), .audio_cluster(audio_cluster), .audio_size(audio_size),
        .sector_req(sector_req), .sector_lba(sector_lba),
        .sector_busy(sector_busy), .sector_valid(sector_valid),
        .sector_byte(sector_byte), .sector_byte_index(sector_byte_index),
        .sector_done(sector_done), .sector_error(sector_error)
    );

    // ========================================================================
    // 磁盘镜像
    //   partition_lba = 1，reserved = 32，fat_count = 2，fat_sectors = 8
    //   → data_start_lba = 1 + 32 + 16 = 49
    //   root_cluster = 2 → root_lba = 49
    //   根目录第一簇占 LBA 49..49+spc-1（最大 spc=8 → 到 56）
    // ========================================================================
    localparam integer ROOT_LBA   = 49;
    localparam integer TOTAL_SEC  = 96;
    reg [7:0] mem [0:TOTAL_SEC*512-1];

    // ---- sector 响应模型 ----
    localparam SRV_IDLE = 2'd0;
    localparam SRV_BUSY = 2'd1;
    localparam SRV_DONE = 2'd2;
    reg [1:0]  srv_state;
    reg [9:0]  byte_cnt;
    reg [31:0] cur_lba;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            srv_state <= SRV_IDLE;
            byte_cnt <= 10'd0;
            cur_lba <= 32'd0;
            sector_busy <= 1'b0;
            sector_valid <= 1'b0;
            sector_byte <= 8'd0;
            sector_byte_index <= 9'd0;
            sector_done <= 1'b0;
        end else begin
            sector_done <= 1'b0;
            case (srv_state)
                SRV_IDLE: begin
                    sector_valid <= 1'b0;
                    if (sector_req) begin
                        cur_lba <= sector_lba;
                        byte_cnt <= 10'd0;
                        sector_busy <= 1'b1;
                        srv_state <= SRV_BUSY;
                    end
                end
                SRV_BUSY: begin
                    if (byte_cnt < 10'd512) begin
                        sector_valid <= 1'b1;
                        sector_byte <= mem[cur_lba*512 + byte_cnt];
                        sector_byte_index <= byte_cnt[8:0];
                        byte_cnt <= byte_cnt + 10'd1;
                    end else begin
                        sector_valid <= 1'b0;
                        sector_done <= 1'b1;
                        sector_busy <= 1'b0;
                        srv_state <= SRV_DONE;
                    end
                end
                SRV_DONE: begin
                    sector_valid <= 1'b0;
                    sector_busy <= 1'b0;
                    srv_state <= SRV_IDLE;
                end
                default: srv_state <= SRV_IDLE;
            endcase
        end
    end

    // ---- 读出扇区计数：多扇区扫描是否真的生效，就靠这个 ----
    integer rd_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rd_cnt <= 0;
        else if (sector_req) rd_cnt <= rd_cnt + 1;
    end

    // ========================================================================
    // 镜像构造工具
    // ========================================================================
    integer k;
    task clear_mem;
        begin
            for (k = 0; k < TOTAL_SEC*512; k = k + 1)
                mem[k] = 8'd0;
        end
    endtask

    task setup_bpb;
        input [7:0] spc;
        integer b;
        begin
            // ---- LBA0 = MBR ----
            mem[446] = 8'h00;
            mem[450] = 8'h0C;                       // FAT32 LBA
            mem[454] = 8'h01;                       // 分区起始 LBA = 1
            mem[455] = 8'h00; mem[456] = 8'h00; mem[457] = 8'h00;
            mem[510] = 8'h55; mem[511] = 8'hAA;
            // ---- LBA1 = 引导扇区 ----
            b = 512;
            mem[b+0]  = 8'hEB;
            mem[b+11] = 8'h00; mem[b+12] = 8'h02;   // 512 字节/扇区
            mem[b+13] = spc;                        // 每簇扇区数
            mem[b+14] = 8'h20; mem[b+15] = 8'h00;   // 保留扇区 32
            mem[b+16] = 8'h02;                      // FAT 份数 2
            mem[b+17] = 8'h00; mem[b+18] = 8'h00;   // FAT32: root_entries=0
            mem[b+22] = 8'h00; mem[b+23] = 8'h00;   // FAT32: fat_size16=0
            mem[b+32] = 8'hE8; mem[b+33] = 8'h03;   // total_sectors = 1000
            mem[b+34] = 8'h00; mem[b+35] = 8'h00;
            mem[b+36] = 8'h08; mem[b+37] = 8'h00;   // fat_sectors = 8
            mem[b+38] = 8'h00; mem[b+39] = 8'h00;
            mem[b+44] = 8'h02; mem[b+45] = 8'h00;   // root_cluster = 2
            mem[b+46] = 8'h00; mem[b+47] = 8'h00;
            mem[b+510] = 8'h55; mem[b+511] = 8'hAA;
        end
    endtask

    // 把根目录全部 128 项的首字节填成 marker（0xE5 = 已删除 / 0x00 = 结束）
    task fill_entries;
        input [7:0] marker;
        input integer count;
        integer j;
        begin
            for (j = 0; j < count; j = j + 1)
                mem[ROOT_LBA*512 + j*32] = marker;
        end
    endtask

    // 写一个 8.3 目录项
    task put_entry;
        input integer         idx;
        input [8*11-1:0]      name11;      // "PIC1    BMP" / "MUSIC   PCM"
        input [7:0]           attr;
        input [31:0]          cluster;
        input [31:0]          fsize;
        integer b, j;
        begin
            b = ROOT_LBA*512 + idx*32;
            for (j = 0; j < 11; j = j + 1)
                mem[b+j] = name11[8*(10-j) +: 8];
            mem[b+11] = attr;
            mem[b+20] = cluster[23:16];
            mem[b+21] = cluster[31:24];
            mem[b+26] = cluster[7:0];
            mem[b+27] = cluster[15:8];
            mem[b+28] = fsize[7:0];
            mem[b+29] = fsize[15:8];
            mem[b+30] = fsize[23:16];
            mem[b+31] = fsize[31:24];
        end
    endtask

    // ========================================================================
    // 通用动作
    // ========================================================================
    task reset_dut;
        begin
            rst_n = 1'b1;
            scan_start = 1'b0;
            #10;
            rst_n = 1'b0;
            #40;
            rst_n = 1'b1;
            #20;
        end
    endtask

    task start_scan;
        begin
            @(negedge clk);
            scan_start = 1'b1;
            @(negedge clk);
            scan_start = 1'b0;
        end
    endtask

    task wait_done;
        integer n;
        begin
            n = 0;
            while (!scan_done && n < 40000) begin
                @(posedge clk);
                n = n + 1;
            end
            if (!scan_done)
                $display("  >>> TIMEOUT: scan_done never asserted");
            repeat (3) @(posedge clk);
        end
    endtask

    integer errors;
    task check_eq;
        input [255:0] label;
        input integer got;
        input integer exp;
        begin
            if (got === exp)
                $display("  PASS  %0s = %0d", label, got);
            else begin
                $display("  FAIL  %0s = %0d (expected %0d)", label, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    // ========================================================================
    // 主流程
    // ========================================================================
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    initial begin
        errors = 0;
        rst_n = 1'b1;
        scan_start = 1'b0;

        // ------------------------------------------------------------------
        // M1：spc=8，MUSIC.PCM 放在第 40 项（第 3 个扇区，簇内偏移 2）
        //     必须能跨扇区找到 —— 这是本次改动的核心目标
        // ------------------------------------------------------------------
        $display("");
        $display("===== M1  spc=8, MUSIC.PCM at entry 40 (root sector 3) =====");
        clear_mem;
        setup_bpb(8'd8);
        fill_entries(8'hE5, 128);          // 全部标为已删除，避免 0x00 提前结束
        put_entry( 0, "PIC1    BMP", 8'h20, 32'd3, 32'd100);
        put_entry( 1, "PIC2    BMP", 8'h20, 32'd4, 32'd200);
        put_entry( 2, "PIC3    BMP", 8'h20, 32'd5, 32'd300);
        put_entry( 3, "PIC4    BMP", 8'h20, 32'd6, 32'd400);
        put_entry(40, "MUSIC   PCM", 8'h20, 32'd7, 32'd576000);
        reset_dut;
        start_scan;
        wait_done;
        $display("  rd_cnt(total) = %0d  -> root sectors = %0d", rd_cnt, rd_cnt - 2);
        check_eq("M1 fs_error",        fs_error,        0);
        check_eq("M1 resource_valid",  resource_valid,  15);
        check_eq("M1 PIC1 cluster",    resource0_cluster, 3);
        check_eq("M1 PIC4 cluster",    resource3_cluster, 6);
        check_eq("M1 audio_valid",     audio_valid,     1);
        check_eq("M1 audio_cluster",   audio_cluster,   7);
        check_eq("M1 audio_size",      audio_size,      576000);
        // 音频在第 40 项 = 根目录第 3 个扇区，找到后四条图+音频齐全即提前收尾
        // 总读出 = LBA0 + 引导扇区 + 根目录 3 个扇区 = 5
        check_eq("M1 rd_cnt(=2+3)",    rd_cnt,          5);

        // ------------------------------------------------------------------
        // M2：spc=8，第 5 项是 0x00 结束标记 → 只读 1 个扇区就收尾
        // ------------------------------------------------------------------
        $display("");
        $display("===== M2  spc=8, entry 5 = 0x00 end marker (early exit) =====");
        clear_mem;
        setup_bpb(8'd8);
        fill_entries(8'hE5, 128);
        put_entry(0, "PIC1    BMP", 8'h20, 32'd3, 32'd100);
        put_entry(1, "PIC2    BMP", 8'h20, 32'd4, 32'd200);
        put_entry(2, "PIC3    BMP", 8'h20, 32'd5, 32'd300);
        put_entry(3, "PIC4    BMP", 8'h20, 32'd6, 32'd400);
        put_entry(4, "MUSIC   PCM", 8'h20, 32'd7, 32'd1024);
        mem[ROOT_LBA*512 + 5*32] = 8'h00;   // 第 5 项 = 空闲，后续全空
        reset_dut;
        start_scan;
        wait_done;
        $display("  rd_cnt(total) = %0d  -> root sectors = %0d", rd_cnt, rd_cnt - 2);
        check_eq("M2 fs_error",       fs_error,        0);
        check_eq("M2 resource_valid", resource_valid,  15);
        check_eq("M2 audio_valid",    audio_valid,     1);
        check_eq("M2 audio_cluster",  audio_cluster,   7);
        // 第 5 项就是 0x00 → 根目录只读 1 个扇区；总读出 = 2 + 1 = 3
        check_eq("M2 rd_cnt(=2+1)",   rd_cnt,          3);

        // ------------------------------------------------------------------
        // M3：spc=8，卡上没有音频 → 读满 8 扇区，图片正常，扫描不报错
        // ------------------------------------------------------------------
        $display("");
        $display("===== M3  spc=8, no audio file (scan whole cluster, images ok) =====");
        clear_mem;
        setup_bpb(8'd8);
        fill_entries(8'hE5, 128);
        put_entry(0, "PIC1    BMP", 8'h20, 32'd3, 32'd100);
        put_entry(1, "PIC2    BMP", 8'h20, 32'd4, 32'd200);
        put_entry(2, "PIC3    BMP", 8'h20, 32'd5, 32'd300);
        put_entry(3, "PIC4    BMP", 8'h20, 32'd6, 32'd400);
        reset_dut;
        start_scan;
        wait_done;
        $display("  rd_cnt(total) = %0d  -> root sectors = %0d", rd_cnt, rd_cnt - 2);
        check_eq("M3 fs_error",       fs_error,        0);
        check_eq("M3 resource_valid", resource_valid,  15);
        check_eq("M3 PIC2 cluster",   resource1_cluster, 4);
        check_eq("M3 audio_valid(exp 0)", audio_valid, 0);
        // 无 0x00 结束标记、也永远等不到 audio_valid → 必须读满整个簇 8 个扇区
        // 总读出 = 2 + 8 = 10
        check_eq("M3 rd_cnt(=2+8)",   rd_cnt,          10);

        // ------------------------------------------------------------------
        // M4：spc=2 簇边界，MUSIC.PCM 在第 20 项（第 2 个扇区）
        //     必须读到第 2 扇区就停，不能越界读到后面簇
        // ------------------------------------------------------------------
        $display("");
        $display("===== M4  spc=2 boundary, MUSIC.PCM at entry 20 (root sector 2) =====");
        clear_mem;
        setup_bpb(8'd2);
        fill_entries(8'hE5, 32);           // spc=2 → 只有 32 个目录项
        put_entry(0, "PIC1    BMP", 8'h20, 32'd3, 32'd100);
        put_entry(1, "PIC2    BMP", 8'h20, 32'd4, 32'd200);
        put_entry(2, "PIC3    BMP", 8'h20, 32'd5, 32'd300);
        put_entry(3, "PIC4    BMP", 8'h20, 32'd6, 32'd400);
        put_entry(20, "MUSIC   PCM", 8'h20, 32'd9, 32'd2880);
        reset_dut;
        start_scan;
        wait_done;
        $display("  rd_cnt(total) = %0d  -> root sectors = %0d", rd_cnt, rd_cnt - 2);
        check_eq("M4 fs_error",       fs_error,        0);
        check_eq("M4 resource_valid", resource_valid,  15);
        check_eq("M4 audio_valid",    audio_valid,     1);
        check_eq("M4 audio_cluster",  audio_cluster,   9);
        check_eq("M4 audio_size",     audio_size,      2880);
        // spc=2 → 根目录簇只有 2 个扇区（32 项）。音频在第 20 项 = 第 2 扇区，
        // 找到后四图+音频齐全即收尾；总读出 = 2 + 2 = 4，绝不能越界读第 3 个扇区
        check_eq("M4 rd_cnt(=2+2)",   rd_cnt,          4);

        // ------------------------------------------------------------------
        $display("");
        if (errors == 0)
            $display(">>> MULTI-SECTOR SCAN REGRESSION PASS");
        else
            $display(">>> MULTI-SECTOR SCAN REGRESSION FAIL (%0d checks)", errors);
        $display("");
        $finish;
    end

    // 全局超时
    initial begin
        #5000000;
        $display(">>> GLOBAL TIMEOUT");
        $finish;
    end

endmodule
