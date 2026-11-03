`timescale 1ns/1ps
// ============================================================================
// tb_wav_async_fifo.v —— 第 11 讲阶段 B：音频跨时钟域 FIFO 回归
//
// 为什么这个 FIFO 必须仿真：
//   它是"SD 卡 100MHz 写 / 播放 25MHz 读"的跨域接缝。gray 指针同步写错一位，
//   症状是"偶尔有个样本重复或丢失"——在板子上表现为听不出来的一点点杂音，
//   极难定位。这里用"写序号 = 数据内容"的方式做端到端核对：
//   读出来的序列必须严格等于写入序列，一个不差、一个不乱。
//
// 覆盖点：
//   B1 灌满：wr_room 归零、rd_level 到达深度
//   B2 排空：数据顺序、总数精确
//   B3 并发（写快读慢 4:1）：不丢、不重、不乱序
//   B4 写侧遵守 wr_room：满了就停，恢复后继续，且不溢出
//   B5 复位后从空开始
// ============================================================================
module tb_wav_async_fifo;

    localparam integer AW    = 10;
    localparam integer DEPTH = 1 << AW;

    reg wr_clk = 0;
    reg rd_clk = 0;
    reg wr_rst_n = 0;
    reg rd_rst_n = 0;
    always #5  wr_clk = ~wr_clk;      // 100MHz
    always #20 rd_clk = ~rd_clk;      // 25MHz

    reg         wr_en = 0;
    reg  [15:0] wr_data = 0;
    wire [AW:0] wr_room;
    reg         rd_pop = 0;
    wire [15:0] rd_data;
    wire        rd_valid;
    wire        rd_empty;
    wire [AW:0] rd_level;

    wav_async_fifo16 #(.AW(AW)) dut (
        .wr_clk(wr_clk), .wr_rst_n(wr_rst_n), .wr_en(wr_en), .wr_data(wr_data),
        .wr_room(wr_room),
        .rd_clk(rd_clk), .rd_rst_n(rd_rst_n), .rd_pop(rd_pop),
        .rd_data(rd_data), .rd_valid(rd_valid), .rd_empty(rd_empty), .rd_level(rd_level)
    );

    // ---------------- 读侧：rd_run=1 时非空就弹，逐个核对 ----------------
    integer    expect_cnt = 0;
    integer    rd_total   = 0;
    integer    rd_err     = 0;
    reg        rd_check_en = 0;
    reg        rd_run      = 0;       // 0 = 暂停读侧（用于把 FIFO 灌满）

    always @(negedge rd_clk) begin
        if (rd_rst_n)
            rd_pop <= rd_run && !rd_empty;
        else
            rd_pop <= 1'b0;
    end

    always @(posedge rd_clk) begin
        if (rd_rst_n && rd_valid) begin
            if (rd_check_en && (rd_data !== expect_cnt[15:0])) begin
                rd_err <= rd_err + 1;
                if (rd_err < 8)
                    $display("  [FIFO] out of order: got %0d expect %0d", rd_data, expect_cnt[15:0]);
            end
            expect_cnt <= expect_cnt + 1;
            rd_total   <= rd_total + 1;
        end
    end

    // ---------------- 检查工具 ----------------
    integer pass_cnt = 0;
    integer fail_cnt = 0;

    task check;
        input            cond;
        input [8*72-1:0] name;
        begin
            if (cond) begin
                pass_cnt = pass_cnt + 1;
                $display("  [PASS] %0s", name);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("  [FAIL] %0s", name);
            end
        end
    endtask

    // ---------------- 写侧任务：写 n 个字，遵守 wr_room ----------------
    integer wr_seq = 0;
    integer room_violation = 0;

    task write_words;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) begin
                @(negedge wr_clk);
                // 等有空间（真实设计里由 wav_sector_feeder 用 file_pause 做同样的事）
                while (wr_room == 0) begin
                    wr_en <= 1'b0;
                    @(negedge wr_clk);
                end
                if (wr_room == 0 || wr_room > DEPTH)
                    room_violation = room_violation + 1;
                wr_en   <= 1'b1;
                wr_data <= wr_seq[15:0];
                wr_seq  = wr_seq + 1;
            end
            @(negedge wr_clk);
            wr_en <= 1'b0;
        end
    endtask

    integer i;
    initial begin
        $display("============================================================");
        $display(" tb_wav_async_fifo -- 100MHz write / 25MHz read");
        $display("============================================================");

        repeat (4) @(negedge wr_clk);
        wr_rst_n = 1;
        rd_rst_n = 1;
        repeat (4) @(negedge wr_clk);

        // ---------------- B1：灌满（读侧暂停）----------------
        $display("\n--- B1 fill to full (reader paused) ---");
        rd_check_en = 1'b0;
        rd_run      = 1'b0;
        write_words(DEPTH);
        repeat (6) @(posedge rd_clk);      // 等写指针跨域同步过来
        check(wr_room == 0, "B1 wr_room == 0 when full");
        $display("  rd_level = %0d (depth %0d)", rd_level, DEPTH);
        check(rd_level == DEPTH, "B1 rd_level == depth");

        // ---------------- B2：排空，校验顺序 ----------------
        $display("\n--- B2 drain and check order ---");
        rd_check_en = 1'b1;
        rd_err      = 0;
        rd_run      = 1'b1;
        wait (rd_empty === 1'b1);
        repeat (4) @(posedge rd_clk);
        check(rd_total == DEPTH, "B2 all words read back");
        check(rd_err == 0, "B2 read order strictly matches write order");
        check(wr_room == DEPTH, "B2 wr_room back to depth after drain");

        // ---------------- B3：并发（写快读慢）----------------
        $display("\n--- B3 concurrent write(100M)/read(25M), 3000 words ---");
        rd_check_en = 1'b1;
        rd_err = 0;
        rd_total = 0;
        expect_cnt = wr_seq;              // 读侧期望值接着写序号走
        write_words(3000);
        // 等读侧追平
        while (!rd_empty) @(posedge rd_clk);
        repeat (10) @(posedge rd_clk);
        check(rd_total == 3000, "B3 3000 words all read back");
        check(rd_err == 0, "B3 no loss / no duplicate / no reorder");
        check(room_violation == 0, "B3 writer never exceeded wr_room");

        // ---------------- B4：读侧停下时写侧必须被挡住 ----------------
        $display("\n--- B4 writer stalls when reader pauses ---");
        rd_check_en = 1'b0;
        rd_run      = 1'b0;               // 暂停读侧
        write_words(DEPTH);               // 正好写满，不溢出
        repeat (6) @(posedge rd_clk);
        check(wr_room == 0, "B4 fifo fills up and writer is blocked (no overflow)");
        rd_run = 1'b1;                    // 放开读侧，剩余数据流走
        wait (rd_empty === 1'b1);
        repeat (6) @(posedge rd_clk);
        check(wr_room == DEPTH, "B4 recovers after reader resumes");

        // ---------------- B5：复位 ----------------
        $display("\n--- B5 reset ---");
        wr_rst_n = 1'b0;
        rd_rst_n = 1'b0;
        repeat (3) @(negedge wr_clk);
        wr_rst_n = 1'b1;
        rd_rst_n = 1'b1;
        repeat (6) @(negedge wr_clk);
        check(rd_empty === 1'b1 && wr_room == DEPTH, "B5 empty after reset");

        $display("\n============================================================");
        $display(" RESULT: %0d passed, %0d failed", pass_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display(" >>> FIFO REGRESSION PASS");
        else
            $display(" >>> FIFO REGRESSION FAIL");
        $display("============================================================");
        $finish;
    end

    initial begin
        #3000000;
        $display("[FATAL] timeout");
        $display(" RESULT: %0d passed, %0d failed", pass_cnt, fail_cnt);
        $finish;
    end

endmodule
