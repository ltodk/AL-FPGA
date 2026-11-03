`timescale 1ns/1ps
// Step1 稳定性增强验证 TB：bmp_read + 模拟 SD 卡扇区读取
// 用 8x8 小图（file_len=54+64*3=246B，1个扇区）加速仿真
module tb_bmp_read_stability;
    reg clk = 0, rst = 1;
    always #5 clk = ~clk;   // 100MHz

    reg         scan_start = 0;
    reg  [31:0] scan_start_sector = 0, scan_max_sector = 1000;
    reg  [2:0]  scan_target_count = 3'd4;
    wire        scan_done, scan_found_valid;
    wire [31:0] scan_found_sector;
    wire [2:0]  scan_found_total;

    reg         load_start = 0, load_abort = 0;
    reg  [31:0] load_sector = 0;
    reg         sd_init_done;
    wire [3:0]  state_code;
    // 第13讲：端口语义由「必须等于」改成「不超过上限」
    reg  [15:0] max_width = 16'd2048, max_height = 16'd1536;
    wire        write_req; reg write_req_ack = 0;
    wire        sd_sec_read; wire [31:0] sd_sec_read_addr;
    reg  [7:0]  sd_mem [0:563199];      // 按扇区组织的模拟卡：覆盖扇区 0~1099
    reg  [7:0]  sd_sec_read_data;
    reg         sd_sec_read_data_valid = 0, sd_sec_read_end = 0;
    wire        bmp_data_wr_en; wire [23:0] bmp_data;
    wire [7:0]  bmp_error;
    wire        load_error;

    // 参数缩短超时：1ms@100MHz=100000 周期（一个扇区读约1030拍，不会误超时）
    bmp_read #(.CLK_FREQ_HZ(100_000_000), .SD_READ_TIMEOUT_MS(1)) u_dut (
        .clk(clk), .rst(rst), .ready(),
        .scan_start(scan_start), .scan_start_sector(scan_start_sector),
        .scan_max_sector(scan_max_sector), .scan_target_count(scan_target_count),
        .scan_done(scan_done), .scan_found_valid(scan_found_valid),
        .scan_found_sector(scan_found_sector), .scan_found_total(scan_found_total),
        .load_start(load_start), .load_abort(load_abort), .load_sector(load_sector),
        .sd_init_done(sd_init_done), .state_code(state_code),
        .max_width(max_width), .max_height(max_height),
        .write_req(write_req), .write_req_ack(write_req_ack),
        .sd_sec_read(sd_sec_read), .sd_sec_read_addr(sd_sec_read_addr),
        .sd_sec_read_data(sd_sec_read_data),
        .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end(sd_sec_read_end),
        .bmp_data_wr_en(bmp_data_wr_en), .bmp_data(bmp_data),
        .bmp_error(bmp_error), .load_error(load_error)
    );

    // 注意：超时路径不在此 TB 验证（需要 SD 模型支持停顿），Step1 以板级验证为主

    // ---- SD 扇区读取模型：读 sd_mem[addr*512 + i]，每拍吐1字节 ----
    integer i;
    initial sd_init_done = 0;

    initial begin : sd_reader
        forever begin
            @(posedge clk);
            if (sd_sec_read) begin
                repeat (3) @(posedge clk);
                for (i = 0; i < 512; i = i + 1) begin
                    sd_sec_read_data       <= sd_mem[sd_sec_read_addr*512 + i];
                    sd_sec_read_data_valid <= 1'b1;
                    @(posedge clk);
                    sd_sec_read_data_valid <= 1'b0;
                end
                sd_sec_read_end <= 1'b1;
                @(posedge clk);
                sd_sec_read_end <= 1'b0;
            end
        end
    end

    // ---- 辅助：写 BMP 头到模拟卡 ----
    task put_header(input [31:0] sector, input [7:0] m0, input [7:0] m1,
                    input [31:0] flen, input [31:0] off,
                    input [31:0] w, input signed [31:0] h,
                    input [15:0] planes_v, input [15:0] bpp, input [31:0] comp,
                    input [31:0] dib);
        begin
            sd_mem[sector*512+ 0] = m0;
            sd_mem[sector*512+ 1] = m1;
            sd_mem[sector*512+ 2] = flen[7:0];   sd_mem[sector*512+ 3] = flen[15:8];
            sd_mem[sector*512+ 4] = flen[23:16]; sd_mem[sector*512+ 5] = flen[31:24];
            sd_mem[sector*512+10] = off[7:0];    sd_mem[sector*512+11] = off[15:8];
            sd_mem[sector*512+12] = off[23:16];  sd_mem[sector*512+13] = off[31:24];
            sd_mem[sector*512+14] = dib[7:0];    sd_mem[sector*512+15] = dib[15:8];
            sd_mem[sector*512+16] = dib[23:16];  sd_mem[sector*512+17] = dib[31:24];
            sd_mem[sector*512+18] = w[7:0];      sd_mem[sector*512+19] = w[15:8];
            sd_mem[sector*512+20] = w[23:16];    sd_mem[sector*512+21] = w[31:24];
            sd_mem[sector*512+22] = h[7:0];      sd_mem[sector*512+23] = h[15:8];
            sd_mem[sector*512+24] = h[23:16];    sd_mem[sector*512+25] = h[31:24];
            sd_mem[sector*512+26] = planes_v[7:0]; sd_mem[sector*512+27] = planes_v[15:8];
            sd_mem[sector*512+28] = bpp[7:0];    sd_mem[sector*512+29] = bpp[15:8];
            sd_mem[sector*512+30] = comp[7:0];   sd_mem[sector*512+31] = comp[15:8];
            sd_mem[sector*512+32] = comp[23:16]; sd_mem[sector*512+33] = comp[31:24];
        end
    endtask

    integer errors = 0;
    // 观测
    reg [7:0] err_q = 0; reg err_pulse = 0;
    always @(posedge clk) begin
        err_pulse <= load_error;
        if (load_error) err_q <= bmp_error;
    end

    // 可靠的加载任务：等 IDLE -> 发 load_start -> 确认被接收 -> 等完成或出错
    task do_load(input [31:0] sector);
        begin
            wait (u_dut.state == 3'd0);          // 等 bmp_read 空闲
            @(posedge clk);
            load_sector <= sector;
            load_start  <= 1'b1;
            @(posedge clk);
            load_start  <= 1'b0;
            repeat (2) @(posedge clk);           // 等状态离开 IDLE
            $display("[do_load] sector=%0d 启动后 state=%0d latched=%0d addr=%0d",
                     sector, u_dut.state, u_dut.load_sector_latched, u_dut.sd_sec_read_addr);
            fork
                wait (load_error);               // 等错误脉冲
                begin
                    wait (u_dut.state == 3'd0);  // 等回到 IDLE（完成）
                    repeat (5) @(posedge clk);   // 等 finish_pending 校验完成
                end
            join_any
            repeat (2) @(posedge clk);           // 等 err_q 的 NBA 更新完成
            $display("[do_load] sector=%0d 结束 state=%0d bmp_error=%0d planes=%0d file_len=%0d",
                     sector, u_dut.state, u_dut.bmp_error, u_dut.u_bmp24_decoder.planes, u_dut.u_bmp24_decoder.file_len);
        end
    endtask

    initial begin
        // 初始化卡为 0xFF（非 BMP 内容）
        for (i = 0; i < 563200; i = i + 1) sd_mem[i] = 8'hff;

        // 场景布局：
        //  sector 100: 好 BMP（8x8, 24bpp, 全字段合法, file_len=246, off=54）
        //  sector 110: 坏 BMP（签名 BM 但 planes=2）flen=1024 -> 应整文件跳2扇区到112
        //  sector 105: 非BMP（签名 XX）
        put_header(100, "B","M", 246, 54, 8, 8, 1, 24, 0, 40);
        put_header(110, "B","M", 1024, 54, 8, 8, 2, 24, 0, 40);  // planes=2 非法
        put_header(105, "X","X", 246, 54, 8, 8, 1, 24, 0, 40);

        // 好 BMP 像素数据：64 像素 * 3 字节 = 192 字节，从 offset 54 开始
        for (i = 0; i < 192; i = i + 1)
            sd_mem[100*512 + 54 + i] = i[7:0];

        // 复位
        rst = 1; repeat (5) @(posedge clk); rst = 0;
        sd_init_done = 1;

        // ===== 测试1：扫描（应找到 sector 100，跳过 105/110 的假图）=====
        @(posedge clk); scan_start <= 1;
        @(posedge clk); scan_start <= 0;
        wait (scan_done);
        @(posedge clk);
        $display("T1 扫描: found_total=%0d first_sector=%0d (期望1张@100)",
                 scan_found_total, scan_found_sector);
        if (scan_found_total == 1 && scan_found_sector == 100)
            $display("T1 PASS：严格校验把 planes=2 的假图拦下（快速跳过生效）");
        else begin errors = errors+1; $display("T1 FAIL"); end

        // ===== 测试2：加载好图（无错误、无 ERR_TRUNCATED 误报）=====
        do_load(100);
        $display("T2 加载好图: err_q=%0d (期望0), pixel_count=%0d (期望64)",
                 err_q, u_dut.pixel_count);
        if (err_q == 0 && u_dut.pixel_count == 64)
            $display("T2 PASS：正常图无误报，像素计数准确");
        else begin errors = errors+1; $display("T2 FAIL"); end

        // ===== 测试3：加载坏头图（planes=2 -> ERR_PLANES 0x33）=====
        do_load(110);
        $display("T3 坏头: err_q=0x%0h (期望0x33=ERR_PLANES)", err_q);
        if (err_q == 8'h33) $display("T3 PASS：坏头报 ERR_PLANES(0x33)");
        else begin errors = errors+1; $display("T3 FAIL"); end

        // ===== 测试4：加载非BMP图（签名XX -> ERR_SIGNATURE 0x31）=====
        do_load(105);
        $display("T4 非BMP: err_q=0x%0h (期望0x31=ERR_SIGNATURE)", err_q);
        if (err_q == 8'h31) $display("T4 PASS");
        else begin errors = errors+1; $display("T4 FAIL"); end

        $display("=== 结果: %s (errors=%0d) ===", errors==0?"ALL PASS":"HAS FAIL", errors);
        $finish;
    end

    // ---- SDRAM 写应答模型：write_req 出现后 1 拍给 ack ----
    reg wr_ack_q = 0;
    always @(posedge clk) begin
        wr_ack_q     <= write_req;
        write_req_ack <= wr_ack_q;
    end

    initial begin #50_000_000; $display("TIMEOUT"); $finish; end

    // 调试：周期性打印状态与关键信号
    integer dbg_cnt = 0;
    always @(posedge clk) begin
        dbg_cnt = dbg_cnt + 1;
        if (dbg_cnt % 20000 == 0)
            $display("[dbg] t=%0t state=%0d sd_sec_read=%b addr=%0d rd_cnt=%0d scan_done=%b found=%0d",
                     $time, state_code, sd_sec_read, sd_sec_read_addr,
                     u_dut.rd_cnt, scan_done, scan_found_total);
    end
    initial $display("[dbg] TB start, clk=%b rst=%b", clk, rst);
endmodule
