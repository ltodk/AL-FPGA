`timescale 1ns / 1ps
// =====================================================================
// tb_osd_param —— OSD 第 4 行参数回显渲染验证（第 14 讲）
//
//   直接扫第 4 行（Y4=62）的 11 个字符位，把 ascii_sel 拼成字符串打印，
//   验证十进制转换是否正确、格式是否对齐 "B:255 C:128"。
//   重点覆盖：0 / 个位数补零 / 99→100 进位 / 200+ / 255 上限。
// =====================================================================
module tb_osd_param;

    reg clk = 1'b0;
    always #20 clk = ~clk;

    reg        rst = 1'b1;
    reg        de  = 1'b0;
    reg        vs  = 1'b0;
    reg [9:0]  vx  = 10'd0;
    reg [23:0] vin = 24'h000000;
    reg        sd_ok = 1'b1;
    reg [1:0]  idx = 2'd0;
    reg [2:0]  tot = 3'd3;
    reg [7:0]  err = 8'd0;
    reg [7:0]  pbr = 8'd255;
    reg [7:0]  pct = 8'd128;
    reg        pshow = 1'b0;

    wire [23:0] vout;

    osd_overlay dut (
        .clk            (clk),
        .rst            (rst),
        .de             (de),
        .vs             (vs),
        .video_x        (vx),
        .video_in       (vin),
        .sd_init_done   (sd_ok),
        .img_idx        (idx),
        .img_total      (tot),
        .bmp_err_code   (err),
        .param_bright   (pbr),
        .param_contrast (pct),
        .param_show     (pshow),
        .video_out      (vout)
    );

    integer n_pass = 0, n_fail = 0;

    // 扫第 4 行的 11 个字符
    task scan_l4;
        output [8*11-1:0] str;
        integer i;
        begin
            force dut.line_cnt = 10'd70;     // Y4(62) + 8，字符中间行
            for (i = 0; i < 11; i = i + 1) begin
                vx = 10'd8 + i*8 + 3;        // X0 + i*CW + 字符内偏移
                #2;
                str[8*(10-i) +: 8] = dut.ascii_sel;
            end
            release dut.line_cnt;
            vx = 10'd0;
        end
    endtask

    task expect_str;
        input [8*40-1:0] name;
        input [8*11-1:0] got;
        input [8*11-1:0] exp;
        begin
            if (got === exp) begin
                n_pass = n_pass + 1;
                $display("  [ OK ] %0s : \"%0s\"", name, got);
            end else begin
                n_fail = n_fail + 1;
                $display("  [FAIL] %0s : 得到 \"%0s\"  期望 \"%0s\"", name, got, exp);
            end
        end
    endtask

    task set_val;
        input [7:0] b;
        input [7:0] c;
        begin
            pbr = b;  pct = c;
            repeat (4) @(posedge clk);       // 2FF 同步
        end
    endtask

    reg [8*11-1:0] s;

    initial begin
        $display("\n===== 第14讲 OSD 参数回显渲染验证 =====");

        rst = 1'b1;
        repeat (5) @(posedge clk);
        rst = 1'b0;
        repeat (4) @(posedge clk);

        // ---------------- P1：显示关闭时无文字 ----------------
        $display("\n[P1] param_show=0 → 第4行不应有文字");
        pshow = 1'b0;
        repeat (4) @(posedge clk);
        scan_l4(s);
        expect_str("show=0", s, "           ");

        // ---------------- P2：默认值 ----------------
        $display("\n[P2] B=255 C=128 → \"B:255 C:128\"");
        pshow = 1'b1;
        repeat (4) @(posedge clk);
        set_val(8'd255, 8'd128);
        scan_l4(s);
        expect_str("255/128", s, "B:255 C:128");

        // ---------------- P3~P7：十进制转换边界 ----------------
        $display("\n[P3] B=0 C=0 → 补零到三位");
        set_val(8'd0, 8'd0);
        scan_l4(s);
        expect_str("0/0", s, "B:000 C:000");

        $display("\n[P4] B=7 C=10 → 个位补零");
        set_val(8'd7, 8'd10);
        scan_l4(s);
        expect_str("7/10", s, "B:007 C:010");

        $display("\n[P5] B=99 C=100 → 进位边界");
        set_val(8'd99, 8'd100);
        scan_l4(s);
        expect_str("99/100", s, "B:099 C:100");

        $display("\n[P6] B=200 C=240 → 百位=2");
        set_val(8'd200, 8'd240);
        scan_l4(s);
        expect_str("200/240", s, "B:200 C:240");

        $display("\n[P7] B=16 C=144 → 实际档位值");
        set_val(8'd16, 8'd144);
        scan_l4(s);
        expect_str("16/144", s, "B:016 C:144");

        // ---------------- P8：像素输出有效性 ----------------
        $display("\n[P8] 扫 'B' 整个 8x16 字符格，确认真的有白色笔画");
        begin : p8
            integer cnt, xx, yy;
            cnt = 0;
            de = 1'b1;                       // ★ 必须拉高 de，否则叠加被门控掉
            for (yy = 62; yy < 78; yy = yy + 1) begin
                force dut.line_cnt = yy[9:0];
                for (xx = 0; xx < 8; xx = xx + 1) begin
                    vx = 10'd8 + xx;         // 第 0 个字符 'B'
                    #1;
                    if (vout === 24'hFFFFFF) cnt = cnt + 1;
                end
            end
            $display("     'B' 白像素 cnt = %0d / 128", cnt);
            if (cnt > 10) begin
                n_pass = n_pass + 1;
                $display("  [ OK ] 字符确实渲染出来了（不是只有 ascii 值）");
            end else begin
                n_fail = n_fail + 1;
                $display("  [FAIL] 'B' 几乎没有白像素，渲染异常");
            end

            // 底带内非笔画处：vin=0 时底带应为纯黑
            force dut.line_cnt = 10'd70;
            vx = 10'd8 + 10*8 + 7;  #2;      // 最后一个字符 '8' 的末列
            $display("     底带位置输出 = %06h", vout);
            if (vout === 24'h000000 || vout === 24'hFFFFFF) begin
                n_pass = n_pass + 1;
                $display("  [ OK ] 底带区域输出受控（黑底或白字）");
            end else begin
                n_fail = n_fail + 1;
                $display("  [FAIL] 底带区域出现意外颜色 %06h", vout);
            end
            release dut.line_cnt;
            de = 1'b0;
        end

        $display("\n========================================");
        $display(" 通过 %0d 项，失败 %0d 项", n_pass, n_fail);
        if (n_fail == 0) $display(" RESULT: PASS");
        else             $display(" RESULT: FAIL");
        $display("========================================\n");
        $finish;
    end

    initial begin
        #500000;
        $display(" TIMEOUT");
        $finish;
    end

endmodule
