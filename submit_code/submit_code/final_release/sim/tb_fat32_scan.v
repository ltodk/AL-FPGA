`timescale 1ns/1ps
// ============================================================================
// tb_fat32_scan.v —— 第七讲 FAT32 卷扫描器仿真验证
//
// 构造一个最小 FAT32 磁盘镜像（MBR + BPB + 根目录 4 个 BMP 目录项），
// 用一个 sector 响应模型模拟 SD 卡，验证 fat32_volume_scanner 能正确：
//   1. 识别 MBR 分区表（offset450=0x0C）
//   2. 解析 BPB（sectors_per_cluster/fat_start_lba/data_start_lba）
//   3. 扫根目录，按文件名匹配 PIC1~4.BMP，输出 4 个簇号和文件大小
// ============================================================================
module tb_fat32_scan;

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

    wire        sector_req;
    wire [31:0] sector_lba;
    reg         sector_busy;
    reg         sector_valid;
    reg  [7:0]  sector_byte;
    reg  [8:0]  sector_byte_index;
    reg         sector_done;
    wire [7:0]  sector_error;

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
        .sector_req(sector_req), .sector_lba(sector_lba),
        .sector_busy(sector_busy), .sector_valid(sector_valid),
        .sector_byte(sector_byte), .sector_byte_index(sector_byte_index),
        .sector_done(sector_done), .sector_error(sector_error)
    );

    // ---- 磁盘镜像：60 个扇区 ----
    reg [7:0] mem [0:60*512-1];
    integer i;

    // ---- sector 响应模型状态机 ----
    reg [1:0] srv_state;
    reg [9:0] byte_cnt;
    reg [31:0] cur_lba;
    localparam SRV_IDLE = 2'd0;
    localparam SRV_BUSY = 2'd1;
    localparam SRV_DONE = 2'd2;

    assign sector_error = 8'd0;

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

    // ---- 填充 FAT32 镜像 ----
    task fill_mem;
        input integer addr;
        input [7:0] val;
        begin
            mem[addr] = val;
        end
    endtask

    integer k;
    integer base;
    initial begin
        // 清 0
        for (k = 0; k < 60*512; k = k + 1)
            mem[k] = 8'd0;

        // ---- LBA0 = MBR ----
        // offset0 = 0x00（不是 EB/E9，走 MBR 分支）
        // offset446 = 分区表项1
        mem[446] = 8'h00;             // boot flag
        mem[450] = 8'h0C;             // 分区类型 FAT32 LBA
        // offset454 = 分区起始 LBA = 1（小端）
        mem[454] = 8'h01;
        mem[455] = 8'h00;
        mem[456] = 8'h00;
        mem[457] = 8'h00;
        // offset510/511 = 0x55AA
        mem[510] = 8'h55;
        mem[511] = 8'hAA;

        // ---- LBA1 = 引导扇区（BPB）----
        // offset0 = 0xEB（跳转，直接引导标志；但我们已经走 MBR 分支到 BOOT_REQ 读 LBA1）
        mem[512 + 0] = 8'hEB;
        // offset11 = bytes_per_sector = 512（0x0200 小端）
        mem[512 + 11] = 8'h00;
        mem[512 + 12] = 8'h02;
        // offset13 = sectors_per_cluster = 1
        mem[512 + 13] = 8'h01;
        // offset14 = reserved_sectors = 32（0x0020 小端）
        mem[512 + 14] = 8'h20;
        mem[512 + 15] = 8'h00;
        // offset16 = fat_count = 2
        mem[512 + 16] = 8'h02;
        // offset17 = root_entries = 0（FAT32）
        mem[512 + 17] = 8'h00;
        mem[512 + 18] = 8'h00;
        // offset22 = fat_size16 = 0（FAT32 用 fat_size32）
        mem[512 + 22] = 8'h00;
        mem[512 + 23] = 8'h00;
        // offset32 = total_sectors = 1000（0x3E8 小端）
        mem[512 + 32] = 8'hE8;
        mem[512 + 33] = 8'h03;
        mem[512 + 34] = 8'h00;
        mem[512 + 35] = 8'h00;
        // offset36 = fat_sectors = 8（0x08 小端）
        mem[512 + 36] = 8'h08;
        mem[512 + 37] = 8'h00;
        mem[512 + 38] = 8'h00;
        mem[512 + 39] = 8'h00;
        // offset44 = root_cluster = 2（小端）
        mem[512 + 44] = 8'h02;
        mem[512 + 45] = 8'h00;
        mem[512 + 46] = 8'h00;
        mem[512 + 47] = 8'h00;
        // offset510/511 = 0x55AA
        mem[512 + 510] = 8'h55;
        mem[512 + 511] = 8'hAA;

        // ---- 数据区起点 = partition_lba(1) + reserved(32) + fat_count(2)*fat_sectors(8) = 1+32+16 = 49 ----
        // 根目录在簇2 = LBA 49 + (2-2)*1 = 49
        // ---- 目录项1 = PIC1.BMP，起始簇 3，大小 100 ----
        begin
            base = 49*512 + 0*32;
            // 文件名 "PIC1    "
            mem[base+0] = "P"; mem[base+1] = "I"; mem[base+2] = "C"; mem[base+3] = "1";
            mem[base+4] = " "; mem[base+5] = " "; mem[base+6] = " "; mem[base+7] = " ";
            // 扩展名 "BMP"
            mem[base+8] = "B"; mem[base+9] = "M"; mem[base+10] = "P";
            // 属性 0x20
            mem[base+11] = 8'h20;
            // 起始簇高16位 = 0
            mem[base+20] = 8'h00; mem[base+21] = 8'h00;
            // 起始簇低16位 = 3
            mem[base+26] = 8'h03; mem[base+27] = 8'h00;
            // 文件大小 = 100（0x64）
            mem[base+28] = 8'h64; mem[base+29] = 8'h00;
            mem[base+30] = 8'h00; mem[base+31] = 8'h00;
        end
        // ---- 目录项2 = PIC2.BMP，起始簇 4，大小 200 ----
        begin
            base = 49*512 + 1*32;
            mem[base+0] = "P"; mem[base+1] = "I"; mem[base+2] = "C"; mem[base+3] = "2";
            mem[base+4] = " "; mem[base+5] = " "; mem[base+6] = " "; mem[base+7] = " ";
            mem[base+8] = "B"; mem[base+9] = "M"; mem[base+10] = "P";
            mem[base+11] = 8'h20;
            mem[base+26] = 8'h04; mem[base+27] = 8'h00;
            mem[base+28] = 8'hC8; mem[base+29] = 8'h00;  // 200
        end
        // ---- 目录项3 = PIC3.BMP，起始簇 5，大小 300 ----
        begin
            base = 49*512 + 2*32;
            mem[base+0] = "P"; mem[base+1] = "I"; mem[base+2] = "C"; mem[base+3] = "3";
            mem[base+4] = " "; mem[base+5] = " "; mem[base+6] = " "; mem[base+7] = " ";
            mem[base+8] = "B"; mem[base+9] = "M"; mem[base+10] = "P";
            mem[base+11] = 8'h20;
            mem[base+26] = 8'h05; mem[base+27] = 8'h00;
            mem[base+28] = 8'h2C; mem[base+29] = 8'h01;  // 300
        end
        // ---- 目录项4 = PIC4.BMP，起始簇 6，大小 400 ----
        begin
            base = 49*512 + 3*32;
            mem[base+0] = "P"; mem[base+1] = "I"; mem[base+2] = "C"; mem[base+3] = "4";
            mem[base+4] = " "; mem[base+5] = " "; mem[base+6] = " "; mem[base+7] = " ";
            mem[base+8] = "B"; mem[base+9] = "M"; mem[base+10] = "P";
            mem[base+11] = 8'h20;
            mem[base+26] = 8'h06; mem[base+27] = 8'h00;
            mem[base+28] = 8'h90; mem[base+29] = 8'h01;  // 400
        end

        // 调试：确认填充正确
        $display("  mem[0]=%h mem[450]=%h mem[510]=%h mem[511]=%h",
                 mem[0], mem[450], mem[510], mem[511]);
        $display("  mem[513]=%h mem[524]=%h mem[525]=%h mem[526]=%h",
                 mem[512+1], mem[512+11], mem[512+12], mem[512+13]);
    end

    // ---- 时钟 ----
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 100MHz
    end

    // ---- 测试主流程 ----
    reg done_flag;
    // 调试：监控关键信号
    always @(posedge clk) begin
        if (sector_req)
            $display("  [t=%0t] sector_req=1 lba=%0d busy=%0d", $time, sector_lba, sector_busy);
        if (sector_done)
            $display("  [t=%0t] sector_done=1", $time);
    end
    always @(scan_busy) $display("  [t=%0t] scan_busy -> %0d", $time, scan_busy);
    always @(scan_start) $display("  [t=%0t] scan_start -> %0d", $time, scan_start);
    initial begin
        rst_n = 1;
        scan_start = 0;
        done_flag = 0;
        #10;
        rst_n = 0;   // 拉低复位（产生 negedge）
        #30;
        rst_n = 1;   // 释放复位
        #20;
        scan_start = 1;
        #10;
        scan_start = 0;

        // 等待 scan_done（最多 20us）
        wait (scan_done == 1'b1);
        #10;
        done_flag = 1;

        // 检查结果
        if (resource_valid == 4'hf) begin
            $display("PASS: resource_valid = %b", resource_valid);
        end else begin
            $display("FAIL: resource_valid = %b (期望 1111)", resource_valid);
        end

        if (resource0_cluster == 32'd3 && resource1_cluster == 32'd4 &&
            resource2_cluster == 32'd5 && resource3_cluster == 32'd6)
            $display("PASS: clusters = %0d %0d %0d %0d",
                     resource0_cluster, resource1_cluster, resource2_cluster, resource3_cluster);
        else
            $display("FAIL: clusters = %0d %0d %0d %0d",
                     resource0_cluster, resource1_cluster, resource2_cluster, resource3_cluster);

        if (resource0_size == 32'd100 && resource1_size == 32'd200 &&
            resource2_size == 32'd300 && resource3_size == 32'd400)
            $display("PASS: sizes = %0d %0d %0d %0d",
                     resource0_size, resource1_size, resource2_size, resource3_size);
        else
            $display("FAIL: sizes = %0d %0d %0d %0d",
                     resource0_size, resource1_size, resource2_size, resource3_size);

        if (data_start_lba == 32'd49 && fat_start_lba == 32'd33 && sectors_per_cluster == 8'd1)
            $display("PASS: data_start=%0d fat_start=%0d spc=%0d",
                     data_start_lba, fat_start_lba, sectors_per_cluster);
        else
            $display("FAIL: data_start=%0d fat_start=%0d spc=%0d",
                     data_start_lba, fat_start_lba, sectors_per_cluster);

        if (fs_error == 8'd0)
            $display("PASS: fs_error = 0");
        else
            $display("FAIL: fs_error = 0x%0h", fs_error);

        #100;
        $display("TEST DONE");
        $finish;
    end

    // 超时保护
    initial begin
        #200000;  // 200us
        if (!done_flag) begin
            $display("TIMEOUT: scan_done 未在 200us 内拉高，fs_error=0x%0h", fs_error);
            $finish;
        end
    end

endmodule
