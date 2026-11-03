`timescale 1ns/1ps
// ============================================================================
// tb_wav_progress.v —— 第11讲阶段C：字节→秒 换算器验证
//
// 单位：秒 = 字节 / 96000（48000 样本/s × 2 字节）
// 关键验证点：
//   G1 整除边界（96000×N）
//   G2 余数边界（N×96000-1 → N-1 秒，不能四舍五入进位）
//   G3 0 字节 / 0 偏移
//   G4 超过 MAX_SEC=999 要截顶，不能回绕
//   G5 size_stable=0 时 tot 不参与运算，但 pos 仍要正常更新
//   G6 一个除法器时分复用：pos 与 tot 交替更新，两者都要对
// ============================================================================
module tb_wav_progress;

    reg         clk;
    reg         rst_n;
    reg         size_stable;
    reg  [31:0] file_size;
    reg  [31:0] file_offset;
    wire [9:0]  pos_sec;
    wire [9:0]  tot_sec;

    wav_progress u_dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .size_stable (size_stable),
        .file_size   (file_size),
        .file_offset (file_offset),
        .pos_sec     (pos_sec),
        .tot_sec     (tot_sec)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;      // 100MHz
    end

    integer errors;

    task check_pair;
        input [8*12-1:0] label;
        input integer    ep;
        input integer    et;
        begin
            if ((pos_sec === ep) && (tot_sec === et)) begin
                $display("  PASS  %0s pos=%0d tot=%0d", label, pos_sec, tot_sec);
            end else begin
                $display("  FAIL  %0s pos=%0d(exp %0d) tot=%0d(exp %0d)",
                         label, pos_sec, ep, tot_sec, et);
                errors = errors + 1;
            end
        end
    endtask

    task settle;
        begin
            repeat (400) @(posedge clk);   // 一轮 66 拍，400 拍足够跑完好几轮
        end
    endtask

    integer i;

    initial begin
        errors      = 0;
        rst_n       = 1'b1;
        size_stable = 1'b1;
        file_size   = 32'd0;
        file_offset = 32'd0;

        #10;
        rst_n = 1'b0;
        #40;
        rst_n = 1'b1;
        #40;

        $display("");
        $display("===== G1/G2 整除与余数边界 =====");
        file_size = 32'd576000; file_offset = 32'd0;          settle;
        check_pair("6s file, pos 0",     0, 6);
        file_offset = 32'd288000;                              settle;
        check_pair("pos = half",         3, 6);
        file_offset = 32'd575999;                              settle;
        check_pair("pos = size-1",       5, 6);
        file_offset = 32'd576000;                              settle;
        check_pair("pos = size",         6, 6);

        file_size = 32'd96000; file_offset = 32'd95999;        settle;
        check_pair("1s-1",               0, 1);
        file_offset = 32'd96000;                               settle;
        check_pair("1s exact",           1, 1);

        file_size = 32'd95999; file_offset = 32'd95999;        settle;
        check_pair("<1s file",           0, 0);

        $display("");
        $display("===== G3 零值 =====");
        file_size = 32'd0; file_offset = 32'd0;                settle;
        check_pair("all zero",           0, 0);

        $display("");
        $display("===== G4 超过 MAX_SEC 截顶 =====");
        file_size   = 32'd300_000_000;                         settle;
        file_offset = 32'd200_000_000;                         settle;
        check_pair("clamp 3125/2083",    999, 999);
        file_size   = 32'd96_000_000;    // 1000s 刚好越过上限
        file_offset = 32'd96_000_000;                          settle;
        check_pair("1000s -> 999",       999, 999);
        file_size   = 32'd95_040_000;    // 990s
        file_offset = 32'd48_000_000;    // 500s
        settle;
        check_pair("990/500 in range",   500, 990);

        $display("");
        $display("===== G5 size_stable=0 时 pos 仍更新 =====");
        size_stable = 1'b0;
        file_size   = 32'd576000;
        file_offset = 32'd192000;                              settle;
        check_pair("unstable, pos=2",    2, 990);   // tot 保持上一次的值（不参与运算）

        $display("");
        $display("===== G6 时分复用：来回切换 offset 多次 =====");
        size_stable = 1'b1;
        file_size   = 32'd480000;                              settle;
        for (i = 0; i < 5; i = i + 1) begin
            file_offset = 32'd0;            settle;
            check_pair("loop pos 0",     0, 5);
            file_offset = 32'd240000;       settle;
            check_pair("loop pos half",  2, 5);
            file_offset = 32'd479999;       settle;
            check_pair("loop pos end",   4, 5);
        end

        $display("");
        if (errors == 0)
            $display(">>> WAV PROGRESS REGRESSION PASS");
        else
            $display(">>> WAV PROGRESS REGRESSION FAIL (%0d checks)", errors);
        $display("");
        $finish;
    end

    initial begin
        #5000000;
        $display(">>> GLOBAL TIMEOUT");
        $finish;
    end

endmodule
