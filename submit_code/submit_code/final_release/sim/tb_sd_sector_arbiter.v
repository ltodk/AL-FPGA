`timescale 1ns/1ps
// ============================================================================
// tb_sd_sector_arbiter.v —— 第 11 讲阶段 B：SD 三路逐扇区仲裁器回归
//
// 覆盖点（对应 sd_sector_arbiter.v 的设计要点）：
//   A1 单客户端：只有音频请求时能正常完成一个扇区
//   A2 优先级：三路同时请求 → 归属顺序必须是 音频 → 图片 → 扫描器
//   A3 逐扇区重判：连续多扇区时每个扇区结束后重新判一次归属
//   A4 请求不丢：在别人正在传输时发出的“单拍请求”必须被 pending 接住
//   A5 数据不串：每个客户端收到的 512 字节必须严格来自它自己的 LBA
//   A6 abort：传输中途 abort，客户端拿不到 done；abort 后还能正常重新开工
//   A7 不饿死：音频持续请求时，图片仍能推进
//
// 说明：本 tb 用一个“扇区适配器行为模型”产生 512 字节，内容 = LBA 低 8 位 +
//       字节序号，这样 A5 能精确断言“数据没串户”。
// ★ 激励全部在 negedge 上驱动：posedge 上驱动会与 DUT 的采样同拍竞争，
//   导致“这一拍到底看不看得见新值”随机化（第一版就是这么假失败的）。
// ============================================================================
module tb_sd_sector_arbiter;

    reg clk = 0;
    reg rst_n = 0;
    reg abort = 0;
    always #5 clk = ~clk;          // 100MHz

    // ---------------- 客户端请求（由 stub 驱动）----------------
    wire        c0_req, c1_req, c2_req;
    wire [31:0] c0_lba, c1_lba, c2_lba;
    reg  [31:0] c0_sel, c1_sel, c2_sel;      // tb 指定的目标 LBA
    reg         c0_go, c1_go, c2_go;
    wire        c0_busy, c0_valid, c0_done;
    wire [7:0]  c0_byte;
    wire [8:0]  c0_idx;
    wire [7:0]  c0_err;
    wire        c1_busy, c1_valid, c1_done;
    wire [7:0]  c1_byte;
    wire [8:0]  c1_idx;
    wire [7:0]  c1_err;
    wire        c2_busy, c2_valid, c2_done;
    wire [7:0]  c2_byte;
    wire [8:0]  c2_idx;
    wire [7:0]  c2_err;
    wire        m_req;
    wire [31:0] m_lba;
    reg         m_busy, m_valid, m_done;
    reg  [7:0]  m_byte;
    reg  [8:0]  m_idx;
    reg  [7:0]  m_err;
    wire [2:0]  grant_dbg;

    sd_sector_arbiter dut (
        .clk(clk), .rst_n(rst_n), .abort(abort),
        .c0_req(c0_req), .c0_lba(c0_lba), .c0_busy(c0_busy), .c0_valid(c0_valid),
        .c0_byte(c0_byte), .c0_byte_index(c0_idx), .c0_done(c0_done), .c0_error(c0_err),
        .c1_req(c1_req), .c1_lba(c1_lba), .c1_busy(c1_busy), .c1_valid(c1_valid),
        .c1_byte(c1_byte), .c1_byte_index(c1_idx), .c1_done(c1_done), .c1_error(c1_err),
        .c2_req(c2_req), .c2_lba(c2_lba), .c2_busy(c2_busy), .c2_valid(c2_valid),
        .c2_byte(c2_byte), .c2_byte_index(c2_idx), .c2_done(c2_done), .c2_error(c2_err),
        .m_req(m_req), .m_lba(m_lba), .m_busy(m_busy), .m_valid(m_valid),
        .m_byte(m_byte), .m_byte_index(m_idx), .m_done(m_done), .m_error(m_err),
        .grant_dbg(grant_dbg)
    );

    // ========================================================================
    // 扇区适配器行为模型：m_req 脉冲 → 512 字节 → m_done
    //   abort 必须能掐断在途事务（真实 sd_sector_adapter 就是这样的），
    //   否则 abort 之后旧事务的 done 会被路由给新客户端 → 数据串户。
    // ========================================================================
    reg [1:0]  mst;
    reg [9:0]  mcnt;
    reg [31:0] mlba_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mst     <= 2'd0;
            m_busy  <= 1'b0;
            m_valid <= 1'b0;
            m_done  <= 1'b0;
            m_byte  <= 8'd0;
            m_idx   <= 9'd0;
            m_err   <= 8'd0;
            mcnt    <= 10'd0;
            mlba_q  <= 32'd0;
        end else if (abort) begin
            mst     <= 2'd0;
            m_busy  <= 1'b0;
            m_valid <= 1'b0;
            m_done  <= 1'b0;
        end else begin
            case (mst)
                2'd0: begin
                    m_valid <= 1'b0;
                    m_done  <= 1'b0;
                    if (m_req) begin
                        mlba_q <= m_lba;
                        mcnt   <= 10'd0;
                        m_busy <= 1'b1;
                        mst    <= 2'd1;
                    end
                end
                2'd1: begin
                    m_valid <= 1'b1;
                    m_byte  <= mlba_q[7:0] + mcnt[7:0];
                    m_idx   <= mcnt[8:0];
                    mcnt    <= mcnt + 10'd1;
                    if (mcnt == 10'd511)
                        mst <= 2'd2;
                end
                2'd2: begin
                    m_valid <= 1'b0;
                    m_busy  <= 1'b0;
                    m_done  <= 1'b1;
                    mst     <= 2'd0;
                end
                default: mst <= 2'd0;
            endcase
        end
    end

    // ========================================================================
    // 三份客户端行为模型
    // ========================================================================
    wire c0_fin, c1_fin, c2_fin;
    wire [31:0] c0_ok, c1_ok, c2_ok, c0_bad, c1_bad, c2_bad;

    client_stub u_c0 (
        .clk(clk), .rst_n(rst_n), .abort(abort), .go(c0_go), .lba_in(c0_sel),
        .req(c0_req), .lba(c0_lba),
        .busy(c0_busy), .valid(c0_valid), .byte_in(c0_byte), .idx(c0_idx),
        .done(c0_done), .err(c0_err),
        .ok_cnt(c0_ok), .bad_cnt(c0_bad), .finished(c0_fin), .id(2'd0)
    );
    client_stub u_c1 (
        .clk(clk), .rst_n(rst_n), .abort(abort), .go(c1_go), .lba_in(c1_sel),
        .req(c1_req), .lba(c1_lba),
        .busy(c1_busy), .valid(c1_valid), .byte_in(c1_byte), .idx(c1_idx),
        .done(c1_done), .err(c1_err),
        .ok_cnt(c1_ok), .bad_cnt(c1_bad), .finished(c1_fin), .id(2'd1)
    );
    client_stub u_c2 (
        .clk(clk), .rst_n(rst_n), .abort(abort), .go(c2_go), .lba_in(c2_sel),
        .req(c2_req), .lba(c2_lba),
        .busy(c2_busy), .valid(c2_valid), .byte_in(c2_byte), .idx(c2_idx),
        .done(c2_done), .err(c2_err),
        .ok_cnt(c2_ok), .bad_cnt(c2_bad), .finished(c2_fin), .id(2'd2)
    );

    // ---------------- 归属顺序记录 ----------------
    reg [2:0] grant_q;
    integer   order_cnt;
    reg [2:0] order_hist [0:15];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            grant_q   <= 3'd7;
            order_cnt <= 0;
        end else begin
            grant_q <= grant_dbg;
            if (grant_dbg != 3'd7 && grant_dbg != grant_q && order_cnt < 16) begin
                order_hist[order_cnt] <= grant_dbg;
                order_cnt <= order_cnt + 1;
            end
        end
    end

    // ---------------- 检查工具 ----------------
    integer pass_cnt = 0;
    integer fail_cnt = 0;
    integer base_c0, base_c1, base_c2;

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

    // ---------------- negedge 激励任务（避免与 posedge 采样竞争）----------------
    task kick0;   // 扫描器发一个扇区请求
        begin
            @(negedge clk); c0_go = 1'b1;
            @(negedge clk); c0_go = 1'b0;
        end
    endtask
    task kick1;   // 图片流读器
        begin
            @(negedge clk); c1_go = 1'b1;
            @(negedge clk); c1_go = 1'b0;
        end
    endtask
    task kick2;   // 音频
        begin
            @(negedge clk); c2_go = 1'b1;
            @(negedge clk); c2_go = 1'b0;
        end
    endtask

    // ========================================================================
    // 主流程
    // ========================================================================
    initial begin
        c0_go = 0; c1_go = 0; c2_go = 0;
        c0_sel = 0; c1_sel = 0; c2_sel = 0;
        abort = 0;

        $display("============================================================");
        $display(" tb_sd_sector_arbiter -- per-sector 3-way arbitration");
        $display("============================================================");

        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // ---------------- A1：只有音频请求 ----------------
        $display("\n--- A1 single client (audio) ---");
        base_c2 = c2_ok;
        c2_sel = 32'd4242;
        kick2;
        wait (c2_fin === 1'b1);
        @(posedge clk);
        check((c2_ok - base_c2) == 1 && c2_bad == 0, "A1 audio sector done, 512 bytes ok");

        // ---------------- A2：三路同时请求 ----------------
        $display("\n--- A2 3-way simultaneous: audio > image > scanner ---");
        order_cnt = 0;
        base_c0 = c0_ok; base_c1 = c1_ok; base_c2 = c2_ok;
        c0_sel = 32'd100; c1_sel = 32'd200; c2_sel = 32'd300;
        @(negedge clk);
        c0_go = 1'b1; c1_go = 1'b1; c2_go = 1'b1;
        @(negedge clk);
        c0_go = 1'b0; c1_go = 1'b0; c2_go = 1'b0;
        wait (c0_fin === 1'b1 && c1_fin === 1'b1 && c2_fin === 1'b1);
        @(posedge clk);
        $display("  grant order: %0d -> %0d -> %0d", order_hist[0], order_hist[1], order_hist[2]);
        check(order_hist[0] == 3'd2 && order_hist[1] == 3'd1 && order_hist[2] == 3'd0,
              "A2 grant order = audio, image, scanner");
        check((c0_ok - base_c0) == 1 && (c1_ok - base_c1) == 1 && (c2_ok - base_c2) == 1,
              "A2 all three sectors complete");
        check(c0_bad == 0 && c1_bad == 0 && c2_bad == 0, "A2 no byte mismatch");

        // ---------------- A3/A4：逐扇区重判 + 迟到请求不丢 ----------------
        $display("\n--- A3/A4 per-sector re-arbitration / late req not lost ---");
        order_cnt = 0;
        base_c0 = c0_ok; base_c1 = c1_ok; base_c2 = c2_ok;
        c2_sel = 32'd500;
        kick2;
        repeat (20) @(posedge clk);          // 音频正在传输
        c1_sel = 32'd600;
        kick1;                               // 图片的单拍请求落在音频在途期间
        repeat (3) @(posedge clk);
        c0_sel = 32'd700;
        kick0;                               // 扫描器同样
        wait (c0_fin === 1'b1 && c1_fin === 1'b1 && c2_fin === 1'b1);
        @(posedge clk);
        check((c0_ok - base_c0) == 1 && (c1_ok - base_c1) == 1 && (c2_ok - base_c2) == 1,
              "A3/A4 late single-cycle requests not lost");
        check(order_cnt >= 3, "A3 at least 3 grant switches");

        // ---------------- A5：连续多扇区 ----------------
        $display("\n--- A5 bursts: audio x4 + image x2 ---");
        base_c1 = c1_ok; base_c2 = c2_ok;
        fork
            begin : audio_burst
                integer k;
                for (k = 0; k < 4; k = k + 1) begin
                    c2_sel = 32'd1000 + k;
                    kick2;
                    wait (c2_fin === 1'b1);
                    @(posedge clk);
                end
            end
            begin : image_burst
                integer k;
                for (k = 0; k < 2; k = k + 1) begin
                    c1_sel = 32'd2000 + k;
                    kick1;
                    wait (c1_fin === 1'b1);
                    @(posedge clk);
                end
            end
        join
        check((c2_ok - base_c2) == 4 && c2_bad == 0, "A5 audio 4 more sectors all correct");
        check((c1_ok - base_c1) == 2 && c1_bad == 0, "A5 image 2 more sectors all correct");

        // ---------------- A7：不饿死 ----------------
        $display("\n--- A7 image progresses while audio hammers ---");
        base_c1 = c1_ok; base_c2 = c2_ok;
        fork
            begin : audio_hammer
                integer k;
                for (k = 0; k < 6; k = k + 1) begin
                    c2_sel = 32'd3000 + k;
                    kick2;
                    wait (c2_fin === 1'b1);
                    @(posedge clk);
                end
            end
            begin : image_progress
                c1_sel = 32'd4000;
                kick1;
                wait (c1_fin === 1'b1);
            end
        join
        check((c1_ok - base_c1) == 1 && c1_bad == 0, "A7 image still progresses under audio load");
        check((c2_ok - base_c2) == 6 && c2_bad == 0, "A7 audio 6 sectors all correct");

        // ---------------- A6：abort ----------------
        $display("\n--- A6 abort in the middle ---");
        base_c0 = c0_ok;
        c0_sel = 32'd9999;
        kick0;
        repeat (30) @(posedge clk);          // 保证已进入传输
        abort = 1'b1;
        repeat (3) @(posedge clk);
        abort = 1'b0;
        repeat (10) @(posedge clk);
        check(c0_fin === 1'b0, "A6 aborted client receives no done");
        c0_sel = 32'd8888;
        kick0;
        wait (c0_fin === 1'b1);
        @(posedge clk);
        check((c0_ok - base_c0) == 1 && c0_bad == 0, "A6 works again after abort");

        // ---------------- 汇总 ----------------
        $display("\n============================================================");
        $display(" RESULT: %0d passed, %0d failed", pass_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display(" >>> ARBITER REGRESSION PASS");
        else
            $display(" >>> ARBITER REGRESSION FAIL");
        $display("============================================================");
        $finish;
    end

    initial begin
        #2000000;
        $display("[FATAL] timeout");
        $display(" RESULT: %0d passed, %0d failed", pass_cnt, fail_cnt);
        $finish;
    end

endmodule


// ============================================================================
// client_stub —— 仲裁器客户端行为模型
//   发一个扇区请求 → 收 512 字节逐个校验 → 计成功/失败
//   finished 是“完成锁存电平”，下一次 go 时才清零 —— 这样 wait() 不会漏脉冲
// ============================================================================
module client_stub (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        abort,
    input  wire        go,
    input  wire [31:0] lba_in,
    output reg         req,
    output reg  [31:0] lba,
    input  wire        busy,
    input  wire        valid,
    input  wire [7:0]  byte_in,
    input  wire [8:0]  idx,
    input  wire        done,
    input  wire [7:0]  err,
    output reg  [31:0] ok_cnt,
    output reg  [31:0] bad_cnt,
    output reg         finished,
    input  wire [1:0]  id
);
    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_REQ  = 2'd1;
    localparam [1:0] S_WAIT = 2'd2;

    reg [1:0]  st;
    reg [31:0] lba_q;
    reg [9:0]  nbytes;
    reg        mismatch;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            req      <= 1'b0;
            lba      <= 32'd0;
            lba_q    <= 32'd0;
            nbytes   <= 10'd0;
            mismatch <= 1'b0;
            ok_cnt   <= 32'd0;
            bad_cnt  <= 32'd0;
            finished <= 1'b0;
        end else begin
            req <= 1'b0;

            if (abort) begin
                // 与真实客户端一致：abort 一拍回 IDLE，不再等 done
                st <= S_IDLE;
            end else begin
                case (st)
                    S_IDLE: if (go) begin
                        lba_q    <= lba_in;
                        lba      <= lba_in;
                        req      <= 1'b1;
                        nbytes   <= 10'd0;
                        mismatch <= 1'b0;
                        finished <= 1'b0;      // 清完成锁存
                        st       <= S_REQ;
                    end

                    S_REQ: st <= S_WAIT;

                    S_WAIT: begin
                        if (valid) begin
                            if (byte_in !== (lba_q[7:0] + nbytes[7:0]))
                                mismatch <= 1'b1;
                            nbytes <= nbytes + 10'd1;
                        end
                        if (done) begin
                            if (nbytes == 10'd512 && !mismatch && err == 8'd0)
                                ok_cnt <= ok_cnt + 32'd1;
                            else begin
                                bad_cnt <= bad_cnt + 32'd1;
                                $display("  [client%0d] lba=%0d BAD nbytes=%0d mismatch=%0d err=%0d",
                                         id, lba_q, nbytes, mismatch, err);
                            end
                            finished <= 1'b1;
                            st <= S_IDLE;
                        end
                    end

                    default: st <= S_IDLE;
                endcase
            end
        end
    end
endmodule
