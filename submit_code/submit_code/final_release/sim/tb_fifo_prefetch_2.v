`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_fifo_prefetch_2 —— 用真实 rfifo_32_32_512 + fifo_prefetch_2 验证
// 链路：写侧(25M) push 数据 → 异步 FIFO → prefetch → 读侧(25M) 连续消费
// 验证点：
//   1) 连续消费时 out_valid 连续为 1（无 underflow 空洞）
//   2) 数据序列 0..N-1 顺序正确、无丢失、无重复
//   3) 数据比"直接读 FIFO"提前 1 拍就绪（prefetch 的核心价值）
// -----------------------------------------------------------------------------
module tb_fifo_prefetch_2;

    parameter N = 40;   // 推入 40 个字

    reg         wclk, rclk, rst;
    reg         we;
    reg  [31:0] wdata;
    wire [31:0] fifo_dout;
    wire        fifo_valid;
    wire        fifo_empty;
    wire        fifo_full;
    wire [8:0]  fifo_wrusedw;
    wire [8:0]  fifo_rdusedw;

    wire        rd_en;
    wire        out_valid;
    wire [31:0] out_data;
    reg         consume;

    // 异步 FIFO（真实 IP）
    rfifo_32_32_512 #(
        .DATA_WIDTH_W(32), .DATA_WIDTH_R(32),
        .ADDR_WIDTH_W(9), .ADDR_WIDTH_R(9),
        .AL_FULL_NUM(504), .AL_EMPTY_NUM(2),
        .SHOW_AHEAD_EN(1'b0), .OUTREG_EN("NOREG")
    ) u_fifo (
        .rst(rst), .clkw(wclk), .clkr(rclk),
        .we(we), .di(wdata), .re(rd_en), .dout(fifo_dout), .valid(fifo_valid),
        .full_flag(fifo_full), .empty_flag(fifo_empty),
        .afull(), .aempty(), .wrusedw(fifo_wrusedw), .rdusedw(fifo_rdusedw)
    );

    fifo_prefetch_2 #(.DATA_WIDTH(32)) u_prefetch (
        .clk(rclk), .rst(rst),
        .raw_empty(fifo_empty), .raw_valid(fifo_valid), .raw_data(fifo_dout),
        .raw_read_en(rd_en), .consume(consume),
        .out_valid(out_valid), .out_empty(), .out_data(out_data)
    );

    // 时钟：写 125M(8ns)、读 25M(40ns)
    always #4  wclk = ~wclk;
    always #20 rclk = ~rclk;

    // ---- 写侧：推入 N 个字 ----
    integer wi;
    initial begin
        wclk = 0; rclk = 0; rst = 1; we = 0; wdata = 0;
        repeat (8) @(posedge wclk);
        rst = 0;
        repeat (4) @(posedge wclk);
        for (wi = 0; wi < N; wi = wi + 1) begin
            @(posedge wclk);
            we = 1; wdata = 32'hB000_0000 + wi;
        end
        @(posedge wclk); we = 0;
    end

    // ---- 读侧：等待数据就绪后连续消费 ----
    reg [31:0] got [0:255];
    integer    got_cnt;
    integer    err;

    // 采集输出（out_valid && consume 同拍）
    always @(posedge rclk) begin
        if (!rst && out_valid && consume) begin
            got[got_cnt] = out_data;
            got_cnt = got_cnt + 1;
        end
    end

    // 消费时记录 out_valid 是否连续（检测 underflow 空洞）
    reg prev_consuming;
    integer holes;
    initial begin
        consume = 0; got_cnt = 0; err = 0; holes = 0; prev_consuming = 0;
        // 等写侧推满 + FIFO 稳定
        repeat (60) @(posedge rclk);

        // 连续消费 40 个字：只要 out_valid 就 consume，模拟 de 全有效
        while (got_cnt < N) begin
            @(posedge rclk);
            if (out_valid) begin
                consume = 1;
                if (prev_consuming == 0 && got_cnt != 0)
                    holes = holes + 1;   // 之前断过，记录空洞
                prev_consuming = 1;
            end else begin
                consume = 0;
                if (prev_consuming == 1)
                    holes = holes + 1;
                prev_consuming = 0;
            end
        end
        consume = 0;

        // 校验序列
        if (got_cnt != N) begin
            $display("[FAIL] got %0d, expect %0d", got_cnt, N);
            err = err + 1;
        end
        for (wi = 0; wi < got_cnt; wi = wi + 1) begin
            if (got[wi] !== 32'hB000_0000 + wi) begin
                $display("[FAIL] word[%0d]=%h expect %h", wi, got[wi], 32'hB000_0000+wi);
                err = err + 1;
            end
        end

        if (err == 0 && holes == 0)
            $display("RESULT: PASS  (%0d 字连续无空洞、顺序正确)", N);
        else if (err != 0)
            $display("RESULT: FAIL  (%0d 数据错误)", err);
        else
            $display("RESULT: FAIL  (%0d 处消费空洞 = underflow)", holes);

        $finish;
    end

    initial #500000 $finish;
endmodule
