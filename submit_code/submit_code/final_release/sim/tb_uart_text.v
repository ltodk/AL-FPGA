// =====================================================================
// tb_uart_text.v  —— uart_logger 输出文本回归
//
// 目的：2026-10-21 对 uart_logger 做了时序优化（把 hex_ascii/十进制数字
//       提前算进寄存器），必须确认 UART 输出文本与优化前逐字符一致。
//
// 做法：把 BAUD 分频缩小（CLK_FREQ_HZ=1_152_000），使每个字符只占 100 拍，
//       逐个触发 10 种消息，把 dut.tx_byte 连续打印出来人工核对。
//
// 期望输出（每段之间用 ---- phase N ---- 分隔）：
//   1  SD OK\r\n
//   2  SD OUT\r\n
//   3  IMG:2/4\r\n
//   4  ERR:0x38\r\n
//   5  WR:3S\r\n
//   6  RD:3S\r\n
//   7  DS:00004000\r\n
//   8  DS:FAIL\r\n
//   9  S1:00001234\r\n
//  10  S0:00002345\r\n
// =====================================================================
`timescale 1ns / 1ps

module tb_uart_text;

    reg clk = 1'b0;
    always #5 clk = ~clk;                 // 100MHz

    reg         rst = 1'b1;
    reg         sd_init_done   = 1'b0;
    reg  [1:0]  img_idx        = 2'd0;
    reg  [2:0]  img_total      = 3'd0;
    reg         load_error     = 1'b0;
    reg  [7:0]  bmp_err_code   = 8'h00;
    reg         ld3s_pulse     = 1'b0;
    reg         ld3s_src       = 1'b0;
    reg         fat_dbg_pulse  = 1'b0;
    reg         fat_dbg_ok     = 1'b0;
    reg  [31:0] fat_dbg_val    = 32'd0;
    reg         scan_diag_pulse= 1'b0;
    reg         scan_diag_zero = 1'b0;
    reg  [31:0] scan_diag_val  = 32'd0;

    wire uart_tx;

    // 分频 = CLK_FREQ_HZ / BAUD_RATE = 10，每字符 10bit x 10 = 100 拍
    uart_logger #(
        .CLK_FREQ_HZ (1_152_000),
        .BAUD_RATE   (115_200)
    ) dut (
        .clk              (clk),
        .rst              (rst),
        .sd_init_done     (sd_init_done),
        .img_idx          (img_idx),
        .img_total        (img_total),
        .load_error       (load_error),
        .bmp_err_code     (bmp_err_code),
        .ld3s_pulse       (ld3s_pulse),
        .ld3s_src         (ld3s_src),
        .fat_dbg_pulse    (fat_dbg_pulse),
        .fat_dbg_ok       (fat_dbg_ok),
        .fat_dbg_val      (fat_dbg_val),
        .scan_diag_pulse  (scan_diag_pulse),
        .scan_diag_zero   (scan_diag_zero),
        .scan_diag_val    (scan_diag_val),
        .uart_tx          (uart_tx)
    );

    // ---- 抓取每个被提交的字符 ----
    integer    char_cnt = 0;
    reg [7:0]  cap [0:255];
    integer    k;

    always @(posedge clk) begin
        if (dut.tx_start) begin
            cap[char_cnt] = dut.tx_byte;
            char_cnt     = char_cnt + 1;
        end
    end

    // ---- 打印自上次以来收到的字符 ----
    task show_chars;
        integer i;
        begin
            if (char_cnt == 0) begin
                $write("<none>");
            end else begin
                for (i = 0; i < char_cnt; i = i + 1) begin
                    if (cap[i] == 8'h0D)      $write("\\r");
                    else if (cap[i] == 8'h0A) $write("\\n");
                    else                      $write("%s", cap[i]);
                end
            end
            $write("\n");
            char_cnt = 0;
        end
    endtask

    task phase_wait;
        input integer n;
        begin
            repeat (n) @(posedge clk);
        end
    endtask

    integer fail_cnt = 0;

    // 期望文本比对（把捕获的字符拼成字符串后逐字节核对）
    reg [8*24-1:0] exp_str;
    task check_text;
        input [8*24-1:0] expected;
        integer i;
        integer len;
        begin
            // 计算长度
            len = 0;
            for (i = 0; i < 24; i = i + 1)
                if (expected[8*(23-i) +: 8] != 8'h00) len = 24 - i;
            // 比对
            if (len != char_cnt) begin
                $display("  [FAIL] 长度不符：期望 %0d 实际 %0d", len, char_cnt);
                fail_cnt = fail_cnt + 1;
            end else begin
                for (i = 0; i < len; i = i + 1) begin
                    if (cap[i] !== expected[8*(24-len+i) +: 8]) begin
                        $display("  [FAIL] 第 %0d 字节：期望 %02h 实际 %02h",
                                 i, expected[8*(24-len+i) +: 8], cap[i]);
                        fail_cnt = fail_cnt + 1;
                    end
                end
            end
            char_cnt = 0;
        end
    endtask

    initial begin
        // 复位：先保持，再释放
        repeat (20) @(posedge clk);
        rst = 1'b0;
        repeat (20) @(posedge clk);

        // ---------------- phase 1: SD OK ----------------
        $display("---- phase 1  expect: SD OK\\r\\n");
        sd_init_done = 1'b1;
        phase_wait(2000);
        show_chars;

        // ---------------- phase 2: SD OUT ----------------
        $display("---- phase 2  expect: SD OUT\\r\\n");
        sd_init_done = 1'b0;
        phase_wait(2000);
        show_chars;

        // ---------------- phase 3: IMG:2/4 ----------------
        // IMG 消息的触发条件是 img_changed && sd_init_done && img_total!=0，
        // 所以先把卡状态恢复成"在位"，等 SD OK 那条发完再切图。
        sd_init_done = 1'b1;
        phase_wait(2000);
        char_cnt = 0;              // 丢掉 "SD OK"，不计入本段
        $display("---- phase 3  expect: IMG:2/4\\r\\n");
        img_total = 3'd4;
        img_idx   = 2'd0;
        phase_wait(50);
        img_idx   = 2'd1;          // 0->1，显示 idx+1 = 2
        phase_wait(2000);
        show_chars;

        // ---------------- phase 4: ERR:0x38 ----------------
        $display("---- phase 4  expect: ERR:0x38\\r\\n");
        bmp_err_code = 8'h38;
        load_error   = 1'b1;
        phase_wait(10);
        load_error   = 1'b0;
        phase_wait(2500);
        show_chars;

        // ---------------- phase 5: WR:3S ----------------
        $display("---- phase 5  expect: WR:3S\\r\\n");
        ld3s_src   = 1'b1;
        ld3s_pulse = 1'b1;
        phase_wait(10);
        ld3s_pulse = 1'b0;
        phase_wait(2000);
        show_chars;

        // ---------------- phase 6: RD:3S ----------------
        $display("---- phase 6  expect: RD:3S\\r\\n");
        ld3s_src   = 1'b0;
        ld3s_pulse = 1'b1;
        phase_wait(10);
        ld3s_pulse = 1'b0;
        phase_wait(2000);
        show_chars;

        // ---------------- phase 7: DS:00004000 ----------------
        $display("---- phase 7  expect: DS:00004000\\r\\n");
        fat_dbg_ok  = 1'b1;
        fat_dbg_val = 32'h0000_4000;
        fat_dbg_pulse = 1'b1;
        phase_wait(10);
        fat_dbg_pulse = 1'b0;
        phase_wait(2500);
        show_chars;

        // ---------------- phase 8: DS:FAIL ----------------
        $display("---- phase 8  expect: DS:FAIL\\r\\n");
        fat_dbg_ok  = 1'b0;
        fat_dbg_pulse = 1'b1;
        phase_wait(10);
        fat_dbg_pulse = 1'b0;
        phase_wait(2000);
        show_chars;

        // ---------------- phase 9: S1:00001234 ----------------
        $display("---- phase 9  expect: S1:00001234\\r\\n");
        scan_diag_zero = 1'b0;
        scan_diag_val  = 32'h0000_1234;
        scan_diag_pulse = 1'b1;
        phase_wait(10);
        scan_diag_pulse = 1'b0;
        phase_wait(2500);
        show_chars;

        // ---------------- phase 10: S0:00002345 ----------------
        $display("---- phase 10 expect: S0:00002345\\r\\n");
        scan_diag_zero = 1'b1;
        scan_diag_val  = 32'h0000_2345;
        scan_diag_pulse = 1'b1;
        phase_wait(10);
        scan_diag_pulse = 1'b0;
        phase_wait(2500);
        show_chars;

        $display("");
        $display("==== tb_uart_text done (人工核对上面 10 段文本) ====");
        $finish;
    end

    // 防止仿真跑飞
    initial begin
        #20_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
