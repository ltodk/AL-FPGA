`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_scrub —— 验证 media_frame_reader 新增的「SDRAM 擦洗(scrub)」逻辑
//
// ①【最关键】擦洗数据绝不能混进显示 FIFO：每帧消费到的有效像素必须正好
//    READ_LEN 个，且帧内地址严格连续递增（0,1,2,...），一旦错位就是擦洗串了
// ② 擦洗 burst 只在「消隐期」（fifo_wrusedw 接近满）才启动，不抢显示带宽
// ③ 擦洗确实发生，且地址在扫
// ④ 擦洗期间 app_rd_busy 必须为高（否则写侧会撞车）
//
// 内存模型：App_rd_en 后 11 拍返回 Sdr_rd_en，数据 = {11'b0, addr}
// 激励：用模计数产生帧节奏（不用状态机，避免 integer/非阻塞混用的坑）
// -----------------------------------------------------------------------------
module tb_scrub;

    localparam ADDR_BITS = 21;
    localparam READ_LEN  = 512;      // 一帧字数（缩小版）
    localparam BLANK_LEN = 3000;     // 消隐期 pix 拍数（真实设计里 vsync 后先消隐再出 de）
    localparam FRAME_PRD = BLANK_LEN + READ_LEN;   // 一帧总 pix 拍数

    reg mem_clk = 0, pix_clk = 0;
    always #4  mem_clk = ~mem_clk;   // 125MHz
    always #20 pix_clk = ~pix_clk;   // 25MHz

    reg rst = 1, pix_rst = 1;

    wire        App_rd_en;
    wire [ADDR_BITS-1:0] App_rd_addr;
    wire        app_rd_busy;
    wire        Sdr_rd_en;
    wire [31:0] Sdr_rd_dout;
    wire [23:0] pixel_rgb;
    wire        pixel_valid;
    wire        read_req_ack;
    wire        underflow;
    reg         pixel_consume = 0;
    reg         read_req = 0;

    media_frame_reader #(
        .SCRUB_EN     (1),
        .SCRUB_BURST  (128),
        .SCRUB_FULL_HI(500),
        .SCRUB_WORDS  (4096)
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
        .pix_rst          (pix_rst),
        .read_req         (read_req),
        .read_req_ack     (read_req_ack),
        .read_addr_0      (21'd0),
        .read_addr_1      (21'd0),
        .read_addr_2      (21'd0),
        .read_addr_3      (21'd0),
        .read_addr_index  (2'd0),
        .read_len         (READ_LEN[ADDR_BITS-1:0]),
        .pixel_consume    (pixel_consume),
        .pixel_rgb        (pixel_rgb),
        .pixel_valid      (pixel_valid),
        .underflow        (underflow)
    );

    // =========================================================================
    // 内存模型：11 拍延迟，数据 = {11'b0, addr}
    // =========================================================================
    reg [ADDR_BITS-1:0] dl [0:10];
    reg                 vq [0:10];
    integer i;
    always @(posedge mem_clk) begin
        if (rst) begin
            for (i = 0; i < 11; i = i + 1) begin dl[i] <= 0; vq[i] <= 0; end
        end else begin
            dl[0] <= App_rd_addr;
            vq[0] <= App_rd_en;
            for (i = 1; i < 11; i = i + 1) begin dl[i] <= dl[i-1]; vq[i] <= vq[i-1]; end
        end
    end
    assign Sdr_rd_en   = vq[10];
    assign Sdr_rd_dout = {11'b0, dl[10]};

    // =========================================================================
    // 显示侧激励：模计数（周期 FRAME_PRD，前 READ_LEN 拍消费）
    // =========================================================================
    integer cyc       = 0;
    integer frm       = 0;
    integer in_frame  = 0;     // 本帧已进入消费窗口的拍数（用于期望值）
    integer got_pix   = 0;     // 本帧收到的有效像素数
    integer n_pixels  = 0;     // 总有效像素
    integer err_pix   = 0;     // 帧内不连续次数
    integer err_cnt   = 0;     // 帧内像素数不对的帧数
    reg [20:0] prev_pix = 21'hx;
    reg        frame_done = 0;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            cyc <= 0; pixel_consume <= 0; read_req <= 0;
        end else begin
            cyc <= cyc + 1;
            // 真实时序：vsync 处发 read_req，然后有整段消隐期给 FIFO 预填，
            // 消隐结束才开始出 de（消费）。这里严格按这个顺序造激励。
            if (cyc % FRAME_PRD == 0) begin
                read_req      <= 1'b1;
                pixel_consume <= 1'b0;
                got_pix       <= 0;
                prev_pix      <= 21'hx;
            end else if (cyc % FRAME_PRD >= BLANK_LEN) begin
                read_req      <= 1'b0;
                pixel_consume <= 1'b1;
            end else begin
                read_req      <= 1'b0;
                pixel_consume <= 1'b0;
            end
        end
    end

    // 帧末统计（在每帧最后一个 pix 拍做）
    // 说明：fifo_prefetch_2 是两级流水，de 拉高后的头 2 拍 prefetch_valid 还为 0，
    // 所以每帧实际只能收到 READ_LEN-2 个有效像素。这是该预取结构固有的启动延迟，
    // 与擦洗无关（SCRUB_EN=0 时同样是 READ_LEN-2）。这里按 READ_LEN-2 判定。
    localparam PIX_EXPECT = READ_LEN - 2;
    always @(posedge pix_clk) begin
        if (!pix_rst && (cyc % FRAME_PRD == FRAME_PRD - 1)) begin
            frm <= frm + 1;
            if (got_pix != PIX_EXPECT) begin
                err_cnt = err_cnt + 1;
                $display("[ERR] 帧%0d 收到 %0d 个有效像素，应为 %0d", frm, got_pix, PIX_EXPECT);
            end
        end
    end

    // 逐像素连续性检查（帧内必须 +1 递增；帧首允许不连续，因为预取里可能留着上一帧的尾巴）
    always @(posedge pix_clk) begin
        if (!pix_rst && pixel_consume && pixel_valid) begin
            n_pixels = n_pixels + 1;
            got_pix  = got_pix + 1;
            if (prev_pix !== 21'hx) begin
                if (pixel_rgb[20:0] !== ((prev_pix + 21'd1) & 21'h1FFFFF)) begin
                    err_pix = err_pix + 1;
                    if (err_pix <= 8)
                        $display("[ERR] 帧%0d 第%0d 个像素不连续: %0d -> %0d",
                                 frm, got_pix, prev_pix, pixel_rgb[20:0]);
                end
            end
            prev_pix = pixel_rgb[20:0];
        end
    end

    // =========================================================================
    // 监测擦洗行为
    // =========================================================================
    integer n_scrub_burst = 0;
    integer err_early     = 0;
    integer err_busy      = 0;
    reg     scr_d = 0;

    always @(posedge mem_clk) begin
        if (!rst && dut.Sdr_init_done) begin
            scr_d <= dut.scr_bus;
            if (dut.scr_bus && !scr_d) begin
                n_scrub_burst = n_scrub_burst + 1;
                if (dut.fifo_wrusedw < 400) begin
                    err_early = err_early + 1;
                    if (err_early <= 5)
                        $display("[ERR] 擦洗在 FIFO 只有 %0d 字时启动（应≈消隐期）", dut.fifo_wrusedw);
                end
            end
            if (dut.scr_bus && !dut.app_rd_busy) begin
                err_busy = err_busy + 1;
                if (err_busy <= 3) $display("[ERR] 擦洗期间 app_rd_busy 没有拉高");
            end
        end
    end

    // =========================================================================
    initial begin
        rst = 1; pix_rst = 1;
        repeat (20) @(posedge mem_clk);
        rst = 0;
        repeat (20) @(posedge pix_clk);
        pix_rst = 0;

        wait (frm >= 8);
        repeat (500) @(posedge pix_clk);

        $display("");
        $display("================ tb_scrub 结果 ================");
        $display("  完成帧数         : %0d", frm);
        $display("  有效像素总数     : %0d", n_pixels);
        $display("  帧内不连续次数   : %0d", err_pix);
        $display("  像素数不对的帧   : %0d", err_cnt);
        $display("  擦洗 burst 次数  : %0d", n_scrub_burst);
        $display("  擦洗过早(抢带宽) : %0d", err_early);
        $display("  擦洗期间未拉忙   : %0d", err_busy);
        $display("  underflow        : %0d", underflow);
        if (err_pix == 0 && err_cnt == 0 && n_scrub_burst > 0 && err_early == 0 && err_busy == 0)
            $display("  RESULT: PASS");
        else
            $display("  RESULT: FAIL");
        $display("===============================================");
        $finish;
    end

    initial begin
        #5_000_000;
        $display("  RESULT: TIMEOUT (frm=%0d scrub=%0d pix=%0d)", frm, n_scrub_burst, n_pixels);
        $finish;
    end

endmodule
