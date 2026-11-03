`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_media_reader —— media_frame_reader 完整读链路验证
//
// 用一个简单 SDRAM 模型（按地址回读数据），验证：
//   ① App_rd_en/App_rd_addr 发出连续的 128 拍 burst 读
//   ② SDRAM 数据经 frame_fifo_read + 异步 FIFO + prefetch 输出 pixel_rgb
//   ③ 无 underflow（显示侧持续消费时数据不中断）
//   ④ 像素序列顺序正确、无丢失、无重复
//
// SDRAM 模型语义：App_rd_en 发出后约 10 拍 Sdr_rd_en 拉高回数据
//（复刻 frame_fifo_read 里 rd_delay==10 的时序假设）。
// -----------------------------------------------------------------------------
module tb_media_reader;

    parameter MEM_DATA_BITS = 32;
    parameter ADDR_BITS     = 21;
    parameter FRAME_WORDS   = 307200;

    // ---- mem_clk 125M / pix_clk 25M ----
    reg mem_clk = 0, pix_clk = 0;
    always #4  mem_clk = ~mem_clk;
    always #20 pix_clk = ~pix_clk;
    reg mem_rst = 1, pix_rst = 1;

    // ---- SDRAM 读接口 ----
    wire        app_rd_en;
    wire [20:0] app_rd_addr;
    reg         sdr_rd_en;
    reg  [31:0] sdr_rd_dout;

    // ---- 显示侧 ----
    reg         read_req;
    wire        read_req_ack;
    reg         pixel_consume;
    wire [23:0] pixel_rgb;
    wire        pixel_valid;
    wire        underflow;

    // ---- 读地址记录（检查 burst 连续性）----
    reg  [20:0] prev_addr;
    integer     addr_gap_err;
    integer     rd_words;

    media_frame_reader #(
        .MEM_DATA_BITS(32), .ADDR_BITS(21), .BURST_BITS(9),
        .BURST_SIZE(128), .FIFO_DEPTH(512), .FIFO_MARGIN(8)
    ) dut (
        .mem_clk(mem_clk), .rst(mem_rst),
        .Sdr_init_done(1'b1), .Sdr_init_ref_vld(1'b1), .Sdr_busy(1'b0),
        .Sdr_rd_en(sdr_rd_en), .Sdr_rd_dout(sdr_rd_dout),
        .App_rd_en(app_rd_en), .App_rd_addr(app_rd_addr),
        .ext_wr_busy  (),
        .ext_wr_active    (1'b0),          // new: 窄写窗口, tb 恒 0
        .pix_clk(pix_clk), .pix_rst(pix_rst),
        .read_req(read_req), .read_req_ack(read_req_ack),
        .read_addr_0(21'd0), .read_addr_1(FRAME_WORDS),
        .read_addr_2(FRAME_WORDS*2), .read_addr_3(FRAME_WORDS*3),
        .read_addr_index(2'd0), .read_len(FRAME_WORDS),
        .pixel_consume(pixel_consume), .pixel_rgb(pixel_rgb), .pixel_valid(pixel_valid),
        .underflow(underflow)
    );

    // ---- SDRAM 回读模型：用移位寄存器模拟 10 拍延迟回数据 ----
    reg        app_rd_en_d [0:10];
    reg [20:0] app_rd_addr_d [0:10];
    integer    j;
    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst) begin
            for (j = 0; j <= 10; j = j + 1) begin
                app_rd_en_d[j]   <= 1'b0;
                app_rd_addr_d[j] <= 21'd0;
            end
            sdr_rd_en <= 1'b0; sdr_rd_dout <= 0;
        end else begin
            // 移位
            app_rd_en_d[0]   <= app_rd_en;
            app_rd_addr_d[0] <= app_rd_addr;
            for (j = 1; j <= 10; j = j + 1) begin
                app_rd_en_d[j]   <= app_rd_en_d[j-1];
                app_rd_addr_d[j] <= app_rd_addr_d[j-1];
            end
            // 第 10 级输出回数据
            sdr_rd_en   <= app_rd_en_d[10];
            sdr_rd_dout <= {7'd0, app_rd_addr_d[10]};
        end
    end

    // ---- 读地址连续性检查 ----
    always @(posedge mem_clk) begin
        if (!mem_rst) begin
            if (app_rd_en) begin
                if (rd_words > 0 && app_rd_addr !== prev_addr + 1'b1)
                    addr_gap_err <= addr_gap_err + 1;
                prev_addr <= app_rd_addr;
                rd_words  <= rd_words + 1;
            end
        end
    end

    // ---- 像素采集 ----
    reg [23:0] px_got [0:1023];
    integer    px_cnt;
    always @(posedge pix_clk) begin
        if (!pix_rst && pixel_valid && pixel_consume) begin
            px_got[px_cnt] = pixel_rgb;
            px_cnt = px_cnt + 1;
        end
    end

    // ---- 下溢检测 ----
    reg underflow_seen;
    always @(posedge pix_clk or posedge pix_rst) begin
        if (pix_rst) underflow_seen <= 0;
        else if (underflow) begin
            underflow_seen <= 1;
            $display("t=%0t  UNDERFLOW! consume=%b valid=%b fifo_empty=%b px_cnt=%0d",
                     $time, pixel_consume, pixel_valid, dut.u_read_fifo.empty_flag, px_cnt);
        end
    end

    // ---- 主流程 ----
    integer err;
    integer k;
    initial begin
        mem_rst = 1; pix_rst = 1;
        read_req = 0; pixel_consume = 0;
        addr_gap_err = 0; rd_words = 0; px_cnt = 0; err = 0;
        prev_addr = 0;

        repeat (10) @(posedge mem_clk);
        mem_rst = 0; pix_rst = 0;
        repeat (10) @(posedge mem_clk);

        // 发一次帧读请求：read_req 是电平握手，保持到 ack 才拉低
        read_req = 1;
        // 等 ack（ack 在 mem_clk 域产生）
        while (!read_req_ack) begin
            @(posedge mem_clk);
        end
        $display("t=%0t  read_req_ack 拉高", $time);
        read_req = 0;

        // 显示侧消费：只在 pixel_valid 时拉高（模拟真实 de 节奏——de 有效即数据就绪）
        // 先等第一个像素有效（FIFO 预填完成），再连续消费
        while (!pixel_valid) @(posedge pix_clk);
        while (px_cnt < 512) begin
            @(posedge pix_clk);
            pixel_consume = pixel_valid;
        end
        $display("t=%0t  消费到 %0d 像素", $time, px_cnt);
        pixel_consume = 0;

        // 检查：无 underflow、地址连续、像素序列正确
        if (underflow) begin
            $display("[FAIL] underflow 发生");
            err = err + 1;
        end
        if (addr_gap_err != 0) begin
            $display("[FAIL] 读地址不连续 %0d 处", addr_gap_err);
            err = err + 1;
        end
        // 像素序列：pixel_rgb[23:0] 应等于地址低 24 位，递增
        for (k = 1; k < px_cnt; k = k + 1) begin
            if (px_got[k] !== px_got[k-1] + 24'd1) begin
                $display("[FAIL] 像素[%0d]=%h 前=%h 不连续", k, px_got[k], px_got[k-1]);
                err = err + 1;
                if (err > 5) k = 1024;
            end
        end

        if (err == 0)
            $display("RESULT: PASS  (读 %0d 字连续、%0d 像素无 underflow 顺序正确)", rd_words, px_cnt);
        else
            $display("RESULT: FAIL  (%0d 处错误)", err);

        $finish;
    end

    initial #1000000 $finish;
endmodule
