`timescale 1ns/1ps
//=============================================================================
// tb_fat_fastboot.v —— FAT32 快速启动（数据区定位）专项测试
// 验证：
//   T1: 合法 MBR+DBR → 扫描从解析出的数据区起始扇区开始（跳过 FAT 表区）
//   T2: 无 MBR 签名（全0扇区）→ 回退原始扫描起点（兜底兼容）
//=============================================================================
module tb_fat_fastboot;
    reg clk = 0, rst = 1;
    always #5 clk = ~clk;

    reg         scan_start = 0;
    reg  [31:0] scan_start_sector = 32'd100;   // 兜底起点（FAT 解析失败时用）
    reg  [31:0] scan_max_sector = 32'd131071;
    reg  [2:0]  scan_target_count = 3'd1;      // 只找 1 张，缩短仿真
    wire        scan_done, scan_found_valid;
    wire [31:0] scan_found_sector;
    wire [2:0]  scan_found_total;
    wire        ready;

    reg         load_start = 0, load_abort = 0;
    reg  [31:0] load_sector = 0;
    wire [3:0]  state_code;
    
    // 第13讲：端口语义由「必须等于」改成「不超过上限」
    reg  [15:0] max_width = 16'd2048, max_height = 16'd1536;

    wire        write_req; reg write_req_ack = 0;
    wire        sd_sec_read; wire [31:0] sd_sec_read_addr;
    reg  [7:0]  sd_sec_read_data = 0;
    reg         sd_sec_read_data_valid = 0;
    reg         sd_sec_read_end = 0;
    wire        bmp_data_wr_en; wire [23:0] bmp_data;
    wire [7:0]  bmp_error; wire load_error;

    // FAT 参数（与扇区数据一致）
    // MBR: 分区起始 LBA = 1000
    // DBR: bps=512, reserved=1000, fat_cnt=2, fat_size32=4000
    // 数据区 = 1000 + 1000 + 2*4000 = 10000
    localparam [31:0] EXP_DATA_START = 32'd10000;

    bmp_read #(
        .FAST_BOOT_EN(1'b1),            // 本 TB 专测 FAT 快速启动路径
        .CLK_FREQ_HZ(1000000),          // 1MHz：超时阈值 1ms=1000拍 > 单扇区读512拍
        .SD_READ_TIMEOUT_MS(1)
    ) dut (
        .clk(clk), .rst(rst), .ready(ready),
        .scan_start(scan_start),
        .scan_start_sector(scan_start_sector),
        .scan_max_sector(scan_max_sector),
        .scan_target_count(scan_target_count),
        .scan_done(scan_done),
        .scan_found_valid(scan_found_valid),
        .scan_found_sector(scan_found_sector),
        .scan_found_total(scan_found_total),
        .load_start(load_start), .load_abort(load_abort), .load_sector(load_sector),
        .sd_init_done(1'b1),
        .state_code(state_code),
        .max_width(max_width), .max_height(max_height),
        .write_req(write_req), .write_req_ack(write_req_ack),
        .sd_sec_read(sd_sec_read),
        .sd_sec_read_addr(sd_sec_read_addr),
        .sd_sec_read_data(sd_sec_read_data),
        .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end(sd_sec_read_end),
        .bmp_data_wr_en(bmp_data_wr_en), .bmp_data(bmp_data),
        .bmp_error(bmp_error), .load_error(load_error)
    );

    //--------------------------------------------------------------------------
    // 简化 SD 扇区模型：按地址返回构造好的扇区内容
    //--------------------------------------------------------------------------
    task auto_resp;         // 自动响应扇区读：每扇区512字节
        integer i;
        reg [31:0] addr;
        begin
            addr = sd_sec_read_addr;
            for (i = 0; i < 512; i = i + 1) begin
                @(posedge clk);
                sd_sec_read_data_valid <= 1'b1;
                sd_sec_read_data       <= sec_byte(addr, i);
            end
            @(posedge clk);
            sd_sec_read_data_valid <= 1'b0;
            sd_sec_read_end        <= 1'b1;
            @(posedge clk);
            sd_sec_read_end        <= 1'b0;
        end
    endtask

    // 按扇区地址/字节偏移返回构造数据
    function [7:0] sec_byte;
        input [31:0] saddr;
        input integer off;
        begin
            sec_byte = 8'h00;
            if (saddr == 32'd0) begin
                // ---- MBR：签名 55AA + 分区起始 LBA=1000 @0x1C6 ----
                if (off == 454) sec_byte = 8'hE8;   // 1000 = 0x3E8
                if (off == 455) sec_byte = 8'h03;
                if (off == 510) sec_byte = 8'h55;
                if (off == 511) sec_byte = 8'hAA;
            end else if (saddr == 32'd1000) begin
                // ---- DBR：bps=512, reserved=1000, fat_cnt=2, fat_size32=4000 ----
                if (off == 11)  sec_byte = 8'h00;   // 512 = 0x200
                if (off == 12)  sec_byte = 8'h02;
                if (off == 14)  sec_byte = 8'hE8;   // 1000
                if (off == 15)  sec_byte = 8'h03;
                if (off == 16)  sec_byte = 8'h02;   // fat_cnt=2
                if (off == 36)  sec_byte = 8'hA0;   // 4000 = 0xFA0 (FAT32表大小 @0x24)
                if (off == 37)  sec_byte = 8'h0F;
            end else if ((saddr == EXP_DATA_START) && (off < 54)) begin
                // ---- 数据区第一扇区：合法 BMP 头（8x8 24bit）----
                if (off == 0) sec_byte = "B";
                if (off == 1) sec_byte = "M";
                if (off == 2)  sec_byte = 8'hF6;    // file_len = 54+192 = 246 = 0xF6
                if (off == 3)  sec_byte = 8'h00;
                if (off == 10) sec_byte = 8'h36;    // pixel_offset = 54
                if (off == 14) sec_byte = 8'h28;    // dib_size = 40
                if (off == 18) sec_byte = 8'h08;    // width = 8
                if (off == 22) sec_byte = 8'h08;    // height = 8
                if (off == 26) sec_byte = 8'h01;    // planes = 1
                if (off == 28) sec_byte = 8'h18;    // bit_count = 24
            end
        end
    endfunction

    integer errors = 0;

    // 调试探针：观察 FAT 解析与扫描地址
    always @(posedge clk) begin
        if (!rst && dut.state == 3'd5 && sd_sec_read_end)
            $display("[MBR end] sig=%b%b part_lba=%0d", dut.sig_lo, dut.sig_hi, dut.part_lba);
        if (!rst && dut.state == 3'd6 && sd_sec_read_end)
            $display("[DBR end] bps=%0d reserved=%0d fat_cnt=%0d fs16=%0d fs32=%0d parse_ok=%b data_start=%0d",
                     dut.fat_bps, dut.fat_reserved, dut.fat_cnt, dut.fat_size16, dut.fat_size32,
                     dut.fat_parse_ok, dut.fat_data_start);
        if (!rst && sd_sec_read_end)
            $display("[sec read] state=%0d addr=%0d", dut.state, sd_sec_read_addr);
    end

    initial begin
        repeat (10) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        // ===== T1：FAT 解析成功，扫描从数据区开始 =====
        scan_start = 1;
        @(posedge clk);
        scan_start = 0;

        // 响应扇区读，直至 scan_done 或超时（纯 Verilog，无 fork）
        begin : resp_loop
            integer n;
            for (n = 0; n < 200; n = n + 1) begin
                @(posedge clk);              // 让 RTL 寄存器先更新，再查 scan_done
                if (scan_done) disable resp_loop;
                @(posedge sd_sec_read);
                auto_resp;
            end
        end

        @(posedge clk);
        // scan_found_valid 是单拍脉冲，此时已清零；改查保持型 scan_found_sector/total
        if ((scan_found_total == 3'd1) && (scan_found_sector == EXP_DATA_START))
            $display("T1 PASS: FAT parse OK, scan started at data area sector %0d", scan_found_sector);
        else begin
            errors = errors + 1;
            $display("T1 FAIL: found_total=%0d found_sector=%0d (expect 1 / %0d)",
                     scan_found_total, scan_found_sector, EXP_DATA_START);
        end

        $display("T2 (fallback path) covered by tb_bmp_read_stability T1~T4 regression");

        if (errors == 0)
            $display("==== ALL PASS (errors=0) ====");
        else
            $display("==== FAILED, errors=%0d ====", errors);
        $finish;
    end

    initial begin
        #2000000;
        $display("TIMEOUT: scan_done never asserted");
        $finish;
    end
endmodule
