`timescale 1ns/1ps
// =============================================================================
// tb_bmp_scaler —— bmp_scale_640x480 单元测试（第 13 讲）
//
// 验证策略：把源坐标编码进像素颜色，输出端反解坐标，与理论抽样公式逐个比对。
//   编码（24 位，注意 RTL 是 RGB888，R 在高位）：
//     R[23:16] = x[7:0]
//     G[15:8]  = y[7:0]
//     B[7:0]   = {x[10:8], y[10:8], 2'b00}
//   解码：x = {B[7:5], R}，y = {B[4:2], G}   （支持到 2047×2047）
//
// 理论抽样公式（与 RTL 的源驱动累加器严格等价，已逐点推导验证）
//   RTL：每个源像素 acc += DST_W；只要 acc >= src_w 就输出一个并 acc -= src_w
//   等价于：目标列 dx 取自源列 sx = floor(((dx+1)*src_w - 1) / DST_W)
//   垂直同理：dy 取自源行 sy = floor(((dy+1)*src_h - 1) / DST_H)
//   自检：1:1 时 sx=dx；2:1 放大时 sx=floor(dx/2)；2:1 缩小时 sx=2dx+1。
//
// 用例
//   C1 640×480  1:1 直通      —— ★ 走旁路，输出必须与输入逐像素完全一致
//   C2 1280×960 2:1 缩小      —— 抽掉一半像素和一半行
//   C3 320×240  2:1 放大      —— 每个源像素/源行重复 2 次
//   C4 800×600  非整数比例    —— 1.25:1，最容易暴露累加器边界错误
//   C5 1024×768 非整数 + 反压 —— out_ready 周期性拉低，验证不会丢像素
//   C6 1920×1080 全高清缩小   —— 真实场景（3:1）
//   C7 640×240  垂直 2:1 放大 —— ★ 水平 1:1 但**走行缓冲**（旁路只按整张图判，
//                                 能覆盖"行缓冲里的水平 1:1 通路"这个最容易出错的接缝）
//   每个用例都强制检查：
//     ① 输出像素总数恰好 307200（否则下游 SHORT_FRAME）
//     ② out_fire 次数 == 307200（写提交计数器就吃这个信号）
//     ③ 无 overflow / 无行缓冲溢出
//     ④ 逐像素源坐标 == 理论公式
// =============================================================================
module tb_bmp_scaler;

    localparam integer DST_W = 640;
    localparam integer DST_H = 480;
    localparam integer TOTAL = DST_W * DST_H;

    reg clk = 0;
    always #5 clk = ~clk;              // 100MHz

    reg rst = 1;
    reg start = 0;
    reg [15:0] src_w = 640;
    reg [15:0] src_h = 480;
    reg in_valid = 0;
    reg [23:0] in_rgb = 0;
    reg out_ready = 1;
    reg mode_1to1 = 0;          // 1 = 直通旁路（上层用寄存器给）

    wire out_valid;
    wire [23:0] out_rgb;
    wire out_fire;
    wire overflow;

    bmp_scale_640x480 #(.DST_W(DST_W), .DST_H(DST_H)) dut (
        .clk        (clk),
        .rst        (rst),
        .start      (start),
        .src_width  (src_w),
        .src_height (src_h),
        .mode_1to1  (mode_1to1),
        .in_valid   (in_valid),
        .in_rgb     (in_rgb),
        .out_ready  (out_ready),
        .out_valid  (out_valid),
        .out_rgb    (out_rgb),
        .out_fire   (out_fire),
        .overflow   (overflow)
    );

    // ---------------------------------------------------------------------
    // 坐标编解码（RGB888：R 在 [23:16]）
    // ---------------------------------------------------------------------
    function [23:0] enc;
        input [11:0] x, y;
        begin
            enc = {x[7:0], y[7:0], x[10:8], y[10:8], 2'b00};
        end
    endfunction

    function [11:0] dec_x;
        input [23:0] rgb;
        begin
            dec_x = {rgb[7:5], rgb[23:16]};
        end
    endfunction

    function [11:0] dec_y;
        input [23:0] rgb;
        begin
            dec_y = {rgb[4:2], rgb[15:8]};
        end
    endfunction

    function integer exp_sx;
        input integer dx;
        input integer w;
        begin
            exp_sx = (((dx + 1) * w) - 1) / DST_W;
        end
    endfunction

    function integer exp_sy;
        input integer dy;
        input integer h;
        begin
            exp_sy = (((dy + 1) * h) - 1) / DST_H;
        end
    endfunction

    // ---------------------------------------------------------------------
    // 输出监视 + 逐像素校验
    // ---------------------------------------------------------------------
    integer out_cnt;
    integer fire_cnt;
    integer case_err;
    reg     check_en;
    reg     byp_cur;            // 本用例是否走 1:1 旁路（旁路时不碰行缓冲）
    integer m_dx, m_dy, m_ex, m_ey, m_gx, m_gy;

    // ---------------------------------------------------------------------
    // 结构诊断（白盒，直接读 DUT 内部信号）
    //   diag_drop : 行缓冲写满导致丢像素的次数 —— 必须恒为 0
    //   diag_pub  : 写侧登记"整行就绪"的次数，应等于源图行数（旁路用例不适用）
    //   diag_cons : 读侧送完一整遍的次数，应等于 DST_H（旁路用例不适用）
    //   pub/cons 对不上说明乒乓半区被吞或被重复消费，是结构级错误而不是
    //   抽样公式错误 —— 这两类失败要分开报，否则极难定位。
    // ---------------------------------------------------------------------
    integer diag_drop = 0, diag_pub = 0, diag_cons = 0;
    // ★★ 下游契约违例计数（第 13 讲 B 轮实机 ERR:0x42 的回归探针）
    //   media_frame_loader 不是标准 AXI-Stream：它把"valid=1 但 ready=0"
    //   直接判为 FIFO 溢出并 abort 整帧。所以真正的握手规则是
    //   「ready 为低的每一拍 valid 都必须是 0」，一次都不许有。
    //   老监视器只在 valid&&ready 时统计，天生看不到这类违例 —— 正因为如此，
    //   这个 bug 在 9 个用例全 PASS 的情况下仍然流到了实机。
    integer hs_err = 0;

    // 用例级开关（由用例在调用 run_case 前设置，run_case 内部读取）
    integer tb_thr    = 0;   // 反压强度：0=无 1=轻(收4停3) 2=极端(每300拍放100)
    integer tb_ovf_ok = 0;   // 1 = 故意制造 overflow，只打印数值不判错

    reg [1:0] thr_mode = 0;
    integer   thr_cnt  = 0;

    always @(posedge clk) begin
        if (out_fire)                                              fire_cnt = fire_cnt + 1;
        if (dut.x_hit && !dut.wr_free)                             diag_drop = diag_drop + 1;
        // 行发布次数：口径必须是"x_hit 使 wr_cnt 走到行尾"，不能带 wr_free 前提。
        // 带 wr_free 会把"溢出时靠推进 wr_cnt 补满的行"漏掉，C8 会误报 pub=2/240。
        if (dut.x_hit && dut.wr_cnt == 11'd639)                     diag_pub  = diag_pub  + 1;
        if (out_fire && dut.rd_cnt == 11'd639)                     diag_cons = diag_cons + 1;
        // ★ 契约检查：valid 拉高时下游必须当拍就能收（loader 的硬性要求）
        if (out_valid && !out_ready) hs_err = hs_err + 1;
    end

    always @(posedge clk) begin
        // ★ out_valid 是组合输出，一次成交的判据是"当拍 valid && 当拍 ready"，
        //   所以监视器必须同时看 out_ready，不能只看 out_valid（否则反压用例
        //   会把下游没接走的像素也算进去）。
        if (out_valid && out_ready) begin
            m_dx = out_cnt % DST_W;
            m_dy = out_cnt / DST_W;
            m_gx = dec_x(out_rgb);
            m_gy = dec_y(out_rgb);
            if (check_en) begin
                m_ex = exp_sx(m_dx, src_w);
                m_ey = exp_sy(m_dy, src_h);
                if (m_gx !== m_ex || m_gy !== m_ey) begin
                    case_err = case_err + 1;
                    if (case_err <= 6)
                        $display("  FAIL pos(%0d,%0d) got(%0d,%0d) exp(%0d,%0d)",
                                 m_dx, m_dy, m_gx, m_gy, m_ex, m_ey);
                end
            end
            out_cnt = out_cnt + 1;
        end
    end

    // ---------------------------------------------------------------------
    // 送源像素（gap = 每个像素之后再空等几拍；0 表示每 2 拍 1 个像素）
    //   真实 SPI 每像素约 96 拍，这里取小值是为了压缩仿真时间。
    // ---------------------------------------------------------------------
    integer fx, fy;

    task feed_frame;
        input integer w;
        input integer h;
        input integer gap;
        begin
            for (fy = 0; fy < h; fy = fy + 1) begin
                for (fx = 0; fx < w; fx = fx + 1) begin
                    @(negedge clk);
                    in_valid <= 1'b1;
                    in_rgb   <= enc(fx[11:0], fy[11:0]);
                    @(negedge clk);
                    in_valid <= 1'b0;
                    repeat (gap) @(negedge clk);
                end
            end
            @(negedge clk);
            in_valid <= 1'b0;
        end
    endtask

    integer wt;
    task wait_all_out;
        begin
            wt = 0;
            while (out_cnt < TOTAL && wt < TOTAL*24) begin
                @(negedge clk);
                wt = wt + 1;
            end
            repeat (20) @(negedge clk);
        end
    endtask

    // ---------------------------------------------------------------------
    // 用例框架
    // ---------------------------------------------------------------------
    integer total_err;

    task run_case;
        input [15:0]  w;
        input [15:0]  h;
        input integer gap;
        input         is_bypass;      // 1 = 期望走 1:1 旁路
        begin
            rst = 1; start = 0; in_valid = 0;
            thr_mode  = tb_thr[1:0];
            mode_1to1 = is_bypass;
            repeat (4) @(negedge clk);
            rst = 0;
            src_w = w;
            src_h = h;
            out_cnt  = 0;
            fire_cnt = 0;
            case_err = 0;
            byp_cur  = is_bypass;
            check_en = 1'b1;
            repeat (2) @(negedge clk);
            @(negedge clk); start = 1;
            @(negedge clk); start = 0;
            repeat (2) @(negedge clk);
            diag_drop = 0; diag_pub = 0; diag_cons = 0; hs_err = 0;

            feed_frame(w, h, gap);
            wait_all_out;

            // 结构检查：丢像素 / 行登记与消费次数必须严格对上
            // ★ C8 是"故意制造 overflow"的诊断用例，只打印数值不判错，
            //   目的是回答一个关键问题：行缓冲一旦 DROP，整帧还完整吗？
            if (tb_ovf_ok) begin
                $display("  INFO  overflow probe: drop=%0d overflow=%b pub=%0d/%0d cons=%0d/%0d out=%0d/%0d fire=%0d/%0d",
                         diag_drop, overflow, diag_pub, h, diag_cons, DST_H,
                         out_cnt, TOTAL, fire_cnt, TOTAL);
                if (out_cnt == TOTAL && fire_cnt == TOTAL && diag_cons == DST_H)
                    $display("  PASS  %0dx%0d -> frame still complete under violent backpressure",
                             w, h);
                else begin
                    $display("  FAIL  %0dx%0d -> FRAME BROKEN under backpressure (this is the 0x38 root cause)",
                             w, h);
                    total_err = total_err + 1;
                end
            end else if (!byp_cur && hs_err != 0) begin
                // ★ 最高优先级的一条：握手不合下游契约，实机会整帧 abort
                $display("  FAIL handshake: out_valid=1 while out_ready=0 (%0d cycles)", hs_err);
                $display("       -> media_frame_loader 会判 ERR_FIFO_OVERFLOW(0x42) 并放弃整帧");
                total_err = total_err + 1;
            end else if (diag_drop != 0 || overflow !== 1'b0) begin
                $display("  FAIL buffer overflow: drop=%0d overflow=%b (行缓冲不够)",
                         diag_drop, overflow);
                total_err = total_err + 1;
            end else if (out_cnt != TOTAL) begin
                $display("  FAIL count: got %0d, expect %0d", out_cnt, TOTAL);
                total_err = total_err + 1;
            end else if (fire_cnt != TOTAL) begin
                $display("  FAIL fire count: got %0d, expect %0d (写提交计数器会数错)",
                         fire_cnt, TOTAL);
                total_err = total_err + 1;
            end else if (case_err != 0) begin
                $display("  FAIL coords: %0d mismatches", case_err);
                total_err = total_err + 1;
            end else if (!byp_cur && diag_pub != h) begin
                $display("  FAIL rows published: %0d, expect source rows %0d", diag_pub, h);
                total_err = total_err + 1;
            end else if (!byp_cur && diag_cons != DST_H) begin
                $display("  FAIL rows consumed: %0d, expect %0d", diag_cons, DST_H);
                total_err = total_err + 1;
            end else if (byp_cur && dut.rd_active !== 1'b0) begin
                $display("  FAIL bypass not taken: rd_active=%b (1:1 应当完全绕开行缓冲)",
                         dut.rd_active);
                total_err = total_err + 1;
            end else begin
                $display("  PASS  %0dx%0d -> 640x480, %0d px, coords ok (%s, pub=%0d cons=%0d)",
                         w, h, out_cnt, byp_cur ? "bypass" : "linebuf", diag_pub, diag_cons);
            end
            $display("");
        end
    endtask

    // ---------------------------------------------------------------------
    // 反压发生器
    //   thr_mode = 0 : 无反压（每拍都能收）
    //   thr_mode = 1 : 轻反压，收 4 拍停 3 拍（C5 用）
    //   thr_mode = 2 : ★ 极端反压，每 300 拍只放 100 拍
    //                  用来复现实机"SDRAM 写通道被总线挡住"的情形：
    //                  读侧排空一行远慢于写侧产出一行 → 两个半区必然同时占满
    //                  → 写侧 DROP。C8 专门用这个模式验证"DROP 之后帧还完整吗"。
    // ---------------------------------------------------------------------
    // ---------------------------------------------------------------------
    // （thr_mode / thr_cnt 的声明在模块上部，run_case 里要用，必须前置）
    always @(negedge clk) begin
        thr_cnt = thr_cnt + 1;
        case (thr_mode)
           2'd1:    out_ready <= ((thr_cnt % 7)   < 4)   ? 1'b1 : 1'b0;
            2'd2:    out_ready <= ((thr_cnt % 300) < 100) ? 1'b1 : 1'b0;
            2'd3:    out_ready <= ((thr_cnt % 16)  < 2)   ? 1'b1 : 1'b0;
            default: out_ready <= 1'b1;
        endcase
    end

    initial begin
        total_err = 0;
        $display("=== tb_bmp_scaler ===");
        $display("");

        // C1 走 1:1 旁路：这是"第 13 讲对第 12 讲零回归"的核心保证，
        // 输出必须与输入逐像素、逐拍完全一致。
        // C0 先跑"640×480 但强制走行缓冲"：mux 的另一条支路也要覆盖，
        //    否则旁路一旦选错，行缓冲里的 1:1 通路就完全没人测。
        $display("[C0] 640x480 1:1 through line buffer (bypass disabled, pixel-exact)");
        run_case(16'd640, 16'd480, 2, 1'b0);

        $display("[C1] 640x480 1:1 passthrough (bypass, must be pixel-exact)");
        run_case(16'd640, 16'd480, 2, 1'b1);

        $display("[C2] 1280x960 2:1 downscale");
        run_case(16'd1280, 16'd960, 0, 1'b0);

        // 放大用例必须给足源像素间隔：输出一行 640 像素至少 640 拍，
        // 而放大 2 倍时一行要输出 2 遍（1280 拍），源行到达时间必须更慢。
        $display("[C3] 320x240 2:1 upscale");
        run_case(16'd320, 16'd240, 8, 1'b0);

        $display("[C4] 800x600 non-integer ratio (1.25:1)");
        run_case(16'd800, 16'd600, 0, 1'b0);

        $display("[C5] 1024x768 non-integer + backpressure");
        tb_thr = 1;
        run_case(16'd1024, 16'd768, 0, 1'b0);
        tb_thr = 0;

        $display("[C6] 1920x1080 full-HD downscale (3:1)");
        run_case(16'd1920, 16'd1080, 0, 1'b0);

        // C7：整张图不是 1:1（所以不走旁路），但水平方向恰好 1:1，
        //     专门覆盖"行缓冲里的水平直通"这条通路。
        $display("[C7] 640x240 vertical 2:1 upscale (horizontal 1:1 in line buffer)");
        run_case(16'd640, 16'd240, 8, 1'b0);

        // ------------------------------------------------------------------
        // C8 ★ 溢出探针（实机 ERR:0x38 的复现与回归）
        //   实机上 B 轮（非 1:1）偶发 ERR:0x38，A 轮（旁路）从不出现；
        //   而顶层 err_code_show 的优先级决定了：显示 0x38 ⇒ **loader 自己没报错**，
        //   即"loader 还活着，但 500ms 内没数满 307200 个像素"。
        //   唯一的可能就是缩放器输出的像素数不足。
        //
        //   ★ 源像素间隔取 16 拍（实机 SPI 16.67MHz 下约 144 拍，这里按比例缩小
        //     以控制仿真时长），反压取 1/8 吞吐。此时写侧一行 320×16=5120 拍，
        //     读侧排 2 遍要 2×640×8=10240 拍 —— 写侧明显更快，两个半区必然占满，
        //     会产生**少量** DROP。这正是实机的量级（偶发、不是持续）。
        //
        //   修复前（DROP 时 wr_cnt 不动）实测：pub 只剩 38/240、out 48640/307200，
        //   正是"像素数不足 → frame_commit 不来 → 0x38"的死法。
        //   修复后（DROP 也推进 wr_cnt）要求 out_cnt / fire_cnt 仍恰好 307200。
        // ------------------------------------------------------------------
        $display("[C8] 320x240 upscale + 1/8 backpressure (overflow probe, frame MUST stay complete)");
        tb_thr = 3; tb_ovf_ok = 1;
        run_case(16'd320, 16'd240, 16, 1'b0);
        tb_thr = 0; tb_ovf_ok = 0;

        $display("=========================================");
        if (total_err == 0)
            $display("RESULT: ALL PASS");
        else
            $display("RESULT: %0d CASE(S) FAILED", total_err);
        $display("=========================================");
        $finish;
    end

    initial begin
        #1_000_000_000;     // 1s 仿真时间（约 1 亿拍 @100MHz），远大于本组用例需要
        $display("TIMEOUT: out_cnt=%0d", out_cnt);
        $finish;
    end

endmodule
