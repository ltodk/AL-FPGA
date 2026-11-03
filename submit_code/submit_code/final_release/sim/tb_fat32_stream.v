`timescale 1ns/1ps
// ============================================================================
// tb_fat32_stream.v —— 第七讲 FAT32 文件流读取器仿真验证
//
// 构造一个跨 2 簇的碎片化文件，验证 fat32_file_streamer 能按 FAT 链遍历：
//   文件从簇 3 开始，FAT[3]=4（下一簇），FAT[4]=EOF，文件大小 600 字节，
//   跨簇 3（512 字节）+ 簇 4（88 字节），验证输出字节流正确 + 按 FAT 链跳簇。
// ============================================================================
module tb_fat32_stream;

    reg         clk;
    reg         rst_n;
    reg         file_start;
    reg  [31:0] first_cluster;
    reg  [31:0] file_size;
    wire        file_busy;
    wire        file_done;
    wire        file_valid;
    wire [7:0]  file_byte;
    wire [31:0] file_byte_offset;
    wire [7:0]  fs_error;

    wire        sector_req;
    wire [31:0] sector_lba;
    reg         sector_busy;
    reg         sector_valid;
    reg  [7:0]  sector_byte;
    reg  [8:0]  sector_byte_index;
    reg         sector_done;
    wire [7:0]  sector_error;

    fat32_file_streamer u_streamer (
        .clk(clk), .rst_n(rst_n), .abort(1'b0), .file_start(file_start),
        .first_cluster(first_cluster), .file_size(file_size),
        .sectors_per_cluster(8'd1), .sectors_per_cluster_shift(3'd0),
        .fat_start_lba(32'd33),
        .data_start_lba(32'd49), .max_cluster(32'd100),
        .file_busy(file_busy), .file_done(file_done),
        .file_valid(file_valid), .file_byte(file_byte),
        .file_byte_offset(file_byte_offset), .fs_error(fs_error),
        .sector_req(sector_req), .sector_lba(sector_lba),
        .sector_busy(sector_busy), .sector_valid(sector_valid),
        .sector_byte(sector_byte), .sector_byte_index(sector_byte_index),
        .sector_done(sector_done), .sector_error(sector_error)
    );

    // ---- 磁盘镜像 ----
    reg [7:0] mem [0:60*512-1];
    integer k;

    // ---- sector 响应模型 ----
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

    // ---- 填充磁盘镜像 ----
    initial begin
        for (k = 0; k < 60*512; k = k + 1)
            mem[k] = 8'd0;

        // FAT 表在 LBA33。FAT[3] = 4（下一簇），FAT[4] = 0x0FFFFFFF（EOF）
        // FAT 条目偏移 = cluster*4
        // FAT[3] 在 LBA33 offset 12~15 = 04 00 00 00（小端 = 4）
        mem[33*512 + 12] = 8'h04;
        mem[33*512 + 13] = 8'h00;
        mem[33*512 + 14] = 8'h00;
        mem[33*512 + 15] = 8'h00;
        // FAT[4] 在 LBA33 offset 16~19 = FF FF FF 0F（小端 = 0x0FFFFFFF）
        mem[33*512 + 16] = 8'hFF;
        mem[33*512 + 17] = 8'hFF;
        mem[33*512 + 18] = 8'hFF;
        mem[33*512 + 19] = 8'h0F;

        // 文件数据：簇3 = LBA50（data_start 49 + (3-2)*1），512 字节填充 0x10~0x11...
        // 簇4 = LBA51，88 字节填充 0x20~0x21...
        for (k = 0; k < 512; k = k + 1)
            mem[50*512 + k] = 8'h10 + (k[7:0]);
        for (k = 0; k < 88; k = k + 1)
            mem[51*512 + k] = 8'h20 + (k[7:0]);
    end

    // ---- 时钟 ----
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // ---- 测试主流程 ----
    reg done_flag;
    reg [31:0] received_cnt;
    reg        first_byte_ok;
    reg        fat_chain_ok;

    initial begin
        rst_n = 1;
        file_start = 0;
        first_cluster = 32'd3;
        file_size = 32'd600;
        done_flag = 0;
        received_cnt = 0;
        first_byte_ok = 0;
        fat_chain_ok = 0;
        #10;
        rst_n = 0;
        #30;
        rst_n = 1;
        #20;
        file_start = 1;
        #10;
        file_start = 0;

        wait (file_done == 1'b1);
        #10;
        done_flag = 1;

        if (fs_error == 8'd0)
            $display("PASS: fs_error = 0");
        else
            $display("FAIL: fs_error = 0x%0h", fs_error);

        if (received_cnt == 32'd600)
            $display("PASS: 收到 %0d 字节（期望 600）", received_cnt);
        else
            $display("FAIL: 收到 %0d 字节（期望 600）", received_cnt);

        if (first_byte_ok)
            $display("PASS: 首字节 = 0x10（簇3 数据正确）");
        else
            $display("FAIL: 首字节错误（期望 0x10）");

        if (fat_chain_ok)
            $display("PASS: 检测到 FAT 链跳簇（读到簇4 的 0x20 数据）");
        else
            $display("FAIL: 未检测到 FAT 链跳簇");

        #100;
        $display("TEST DONE");
        $finish;
    end

    // 统计收到的字节 + 校验首字节和 FAT 链
    always @(posedge clk) begin
        if (file_valid) begin
            received_cnt <= received_cnt + 32'd1;
            // 首字节（offset 0）= 0x10
            if (file_byte_offset == 32'd0 && file_byte == 8'h10)
                first_byte_ok <= 1'b1;
            // 第 513 字节（offset 512，簇4 首字节）= 0x20，证明 FAT 链跳簇成功
            if (file_byte_offset == 32'd512 && file_byte == 8'h20)
                fat_chain_ok <= 1'b1;
        end
    end

    // 超时保护
    initial begin
        #200000;
        if (!done_flag) begin
            $display("TIMEOUT: file_done 未在 200us 内拉高，fs_error=0x%0h", fs_error);
            $finish;
        end
    end

endmodule
