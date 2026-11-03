// =============================================================================
// tb_thumb_gen.v —— 第15讲：2×2 缩略图页生成引擎（真实 2:1 抽取模式）
//
// 目的：
//   在 ModelSim 里用"理想 SDRAM 模型"验证 media_frame_reader 内部的生成引擎：
//     ① 每组写必须**恰好 4 拍连续**（APUG011：ERAM 最小写单位 = 4×32bit = 128bit）
//     ② 每组写首地址必须 4 字对齐
//     ③ 组内地址严格 +1
//     ④ 每笔数据的值 == 源图对应像素（2:1 抽取，取偶数列 / 偶数行）
//     ⑤ 写完 307200 字后逐像素比对整页，缺失/错值必须为 0
//
// SDRAM 模型：
//   · 读延迟固定 11 拍（真实控制器 8~11 拍）
//   · 源图内容 = 地址本身（src_mem[i] = i），所以"期望值"就是源地址
//   · 写：tb 直接把 tb_wr_addr/tb_wr_data 落进 page 数组（不再模拟忙）
//   · 不模拟擦洗（SCRUB_EN=0），只测生成引擎本身
//
// 映射（必须与 RTL 的 tb_page_off / tb_rd_addr 完全一致）：
//   页内偏移 j → row=j/640, col=j%640
//   qx = col>=320, qy = row>=240, img = qy*2+qx
//   d = row - qy*240, c = col - qx*320
//   源地址 = img*307200 + d*1280 + c*2
// =============================================================================
`timescale 1ns/1ps

module tb_thumb_gen;

    localparam integer SRC_W     = 640;
    localparam integer FRAME_N   = 307200;
    localparam integer TB_ADDR   = 1228800;
    localparam integer TB_W      = 320;
    localparam integer TB_H      = 240;
    localparam integer TB_PASS   = 2;          // 与正式版一致：整页跑两遍
    localparam integer SRC_WORDS = 4 * FRAME_N;

    reg mem_clk = 1'b0;
    reg pix_clk = 1'b0;
    always #4  mem_clk = ~mem_clk;     // 125MHz
    always #20 pix_clk = ~pix_clk;     // 25MHz

    reg rst = 1'b1;

    // ---------------- 源图模型 ----------------
    reg [31:0] src_mem [0:SRC_WORDS-1];
    integer i;
    initial begin
        for (i = 0; i < SRC_WORDS; i = i + 1)
            src_mem[i] = i[31:0];
    end

    // ---------------- DUT ----------------
    wire        App_rd_en;
    wire [20:0] App_rd_addr;
    wire        Sdr_rd_en;
    wire [31:0] Sdr_rd_dout;
    wire        tb_wr_en;
    wire [20:0] tb_wr_addr;
    wire [31:0] tb_wr_data;
    wire        tb_busy, tb_done, app_rd_busy, undeflow_x;
    wire [23:0] pixel_rgb;
    wire        pixel_valid;
    wire        read_req_ack;

    // 读延迟流水（11 拍）
    reg [20:0] rd_a [0:10];
    reg        rd_v [0:10];
    integer k;
    always @(posedge mem_clk) begin
        rd_a[0] <= App_rd_addr;
        rd_v[0] <= App_rd_en;
        for (k = 1; k <= 10; k = k + 1) begin
            rd_a[k] <= rd_a[k-1];
            rd_v[k] <= rd_v[k-1];
        end
    end
    assign Sdr_rd_en   = rd_v[10];
    assign Sdr_rd_dout = src_mem[rd_a[10]];

    media_frame_reader #(
        .MEM_DATA_BITS     (32),
        .ADDR_BITS         (21),
        .SCRUB_EN          (0),          // 本 tb 只测生成引擎
        .SCRUB_WORDS       (1536000),
        .TB_EN             (1),
        .TB_TEST_EN        (0),
        .TB_PASSES         (TB_PASS),
        .TB_ADDR           (21'd1228800),
        .TB_SRC_W          (SRC_W),
        .TB_W              (TB_W),
        .TB_H              (TB_H)
    ) dut (
        .mem_clk          (mem_clk),
        .rst              (rst),
        .Sdr_init_done    (1'b1),
        .Sdr_init_ref_vld (1'b0),
        .Sdr_busy         (1'b0),
        .Sdr_rd_en        (Sdr_rd_en),
        .Sdr_rd_dout      (Sdr_rd_dout),
        .App_rd_en        (App_rd_en),
        .App_rd_addr      (App_rd_addr),
        .ext_wr_busy  (1'b0),
        .ext_wr_active    (1'b0),          // new: 窄写窗口, tb 恒 0
        .app_rd_busy      (app_rd_busy),
        .pix_clk          (pix_clk),
        .pix_rst          (rst),
        .read_req         (1'b0),
        .read_req_ack     (read_req_ack),
        .read_addr_0      (21'd0),
        .read_addr_1      (21'd307200),
        .read_addr_2      (21'd614400),
        .read_addr_3      (21'd921600),
        .read_addr_index  (2'd0),
        .read_len         (21'd307200),
        .pixel_consume    (1'b0),        // 全程消隐
        .pixel_rgb        (pixel_rgb),
        .pixel_valid      (pixel_valid),
        .underflow        (undeflow_x),
        .tb_image_complete(4'b1111),
        .src_addr_0       (21'd0),
        .src_addr_1       (21'd307200),
        .src_addr_2       (21'd614400),
        .src_addr_3       (21'd921600),
        .tb_wr_en         (tb_wr_en),
        .tb_wr_addr       (tb_wr_addr),
        .tb_wr_data       (tb_wr_data),
        .tb_busy          (tb_busy),
        .tb_done          (tb_done)
    );

    // ---------------- 页模型 + 检查 ----------------
    reg [31:0] page    [0:FRAME_N-1];
    reg        written [0:FRAME_N-1];

    integer n_words    = 0;
    integer n_groups   = 0;
    integer err_len    = 0;
    integer err_align  = 0;
    integer err_seq    = 0;
    integer err_oob    = 0;
    integer grp_len    = 0;
    reg [20:0] grp_addr = 21'd0;
    integer off;

    // 读 burst 监视
    integer rd_len   = 0;
    integer err_rlen = 0;
    integer err_raln = 0;

    always @(posedge mem_clk) begin
        // ---- 读 side：每段 burst 必须恰好 8 拍且 8 字对齐 ----
        if (App_rd_en) begin
            if (rd_len == 0)
                if (App_rd_addr[2:0] != 3'd0) err_raln = err_raln + 1;
            rd_len = rd_len + 1;
        end else if (rd_len != 0) begin
            if (rd_len != 8) err_rlen = err_rlen + 1;
            rd_len = 0;
        end

        // ---- 写 side：每段必须恰好 4 拍、4 字对齐、地址 +1 ----
        if (tb_wr_en) begin
            if (grp_len == 0) begin
                grp_addr = tb_wr_addr;
                if (tb_wr_addr[1:0] != 2'd0) err_align = err_align + 1;
            end else begin
                if (tb_wr_addr != (grp_addr + grp_len)) err_seq = err_seq + 1;
            end
            grp_len = grp_len + 1;
            n_words = n_words + 1;
            off = tb_wr_addr - TB_ADDR;
            if ((off < 0) || (off >= FRAME_N)) begin
                err_oob = err_oob + 1;
            end else begin
                page[off]    = tb_wr_data;
                written[off] = 1'b1;
            end
        end else if (grp_len != 0) begin
            if (grp_len != 4) err_len = err_len + 1;
            n_groups = n_groups + 1;
            grp_len  = 0;
        end
    end

    // ---------------- 主流程 ----------------
    integer t;
    integer j, row, col, qy, qx, img, d, c;
    integer exp_addr;
    integer mism = 0;
    integer missing = 0;

    initial begin
        #200 rst = 1'b0;

        t = 0;
        while (!tb_done && (t < 40000000)) begin
            @(posedge mem_clk);
            t = t + 1;
        end

        repeat (200) @(posedge mem_clk);

        if (!tb_done) begin
            $display("RESULT: FAIL (timeout, n_words=%0d n_groups=%0d)", n_words, n_groups);
            $finish;
        end

        // 逐像素比对整页
        mism    = 0;
        missing = 0;
        for (j = 0; j < FRAME_N; j = j + 1) begin
            row = j / SRC_W;
            col = j % SRC_W;
            qx  = (col >= TB_W) ? 1 : 0;
            qy  = (row >= TB_H) ? 1 : 0;
            img = qy * 2 + qx;
            d   = row - qy * TB_H;
            c   = col - qx * TB_W;
            exp_addr = img * FRAME_N + d * (2 * SRC_W) + c * 2;
            if (!written[j]) begin
                missing = missing + 1;
                if (missing <= 4)
                    $display("  MISSING  offset=%0d (want %0d)", j, exp_addr);
            end else if (page[j] !== exp_addr[31:0]) begin
                mism = mism + 1;
                if (mism <= 4)
                    $display("  MISMATCH offset=%0d got=%0d want=%0d", j, page[j], exp_addr);
            end
        end

        $display("--------------------------------------------------");
        $display("n_words      = %0d (expect %0d)", n_words,  FRAME_N * TB_PASS);
        $display("n_groups     = %0d (expect %0d)", n_groups, (FRAME_N/4) * TB_PASS);
        $display("err_group_len= %0d", err_len);
        $display("err_wr_align = %0d", err_align);
        $display("err_wr_seq   = %0d", err_seq);
        $display("err_oob      = %0d", err_oob);
        $display("err_rd_len   = %0d", err_rlen);
        $display("err_rd_align = %0d", err_raln);
        $display("missing      = %0d", missing);
        $display("mismatch     = %0d", mism);
        if ((err_len == 0) && (err_align == 0) && (err_seq == 0) && (err_oob == 0) &&
            (err_rlen == 0) && (err_raln == 0) && (missing == 0) && (mism == 0) &&
            (n_words == FRAME_N * TB_PASS))
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");
        $finish;
    end

endmodule
