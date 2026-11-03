`timescale 1ns/1ps
// =============================================================================
// tb_load_handshake —— "第 2 帧开始 loader 收不到 load_start" 的复现与验证
//
// 背景（实机现象）：
//   上电首图正常；按切换 → 先闪 ERR:0x41，再变 ERR:0x38，串口反复 WR:3S，
//   图永远切不动。
//
// 分析：
//   sd_card_bmp 的 load_start_pulse 是"电平保持"（等 streamer 真正开始读、
//   file_busy 上升沿才撤），而 write_req 原来写成
//       if (load_start_pulse) write_req<=1; else if (ack) write_req<=0;
//   顶层 ack = write_req 上升沿的单拍脉冲，恰好落在 load_start_pulse=1 的周期，
//   被 if 分支抢先 → write_req 置 1 后再也清不掉 → 第二帧没有上升沿 →
//   loader 不启动（像素来时 !active → 0x41）→ 写完信号不来 → 0x38。
//
// 本 tb 用"真实 RTL"（loader + CDC + commit）+ 复刻的 write_req 逻辑：
//   OLD_MODE  : 复刻老逻辑 → 第 2 帧 load_start 次数应为 0（复现 bug）
//   默认(NEW) : 用 load_kick 置请求 → 每帧 load_start 恰好 1 次、正常提交
//
// 编译：vlog +define+OLD_MODE ... tb_load_handshake.v   （复现）
//       vlog ... tb_load_handshake.v                    （验证修复）
// =============================================================================
module tb_load_handshake;

`ifdef OLD_MODE
    localparam integer OLD_MODE_EN = 1;
`else
    localparam integer OLD_MODE_EN = 0;
`endif

    localparam integer IMAGE_WORDS   = 307200;
    localparam integer PIXEL_GAP     = 7;        // 每 8 拍一个像素（远快于真实 SPI）
    localparam integer FILE_OPEN_LAT = 900;      // 模拟读 FAT/文件头的延迟
    localparam integer SETTLE_CYCLES = 200000;   // 提交等待窗口

    reg clk = 1'b0;
    reg dst_clk = 1'b0;
    always #5  clk     = ~clk;      // 100 MHz
    always #4  dst_clk = ~dst_clk;  // 125 MHz

    reg rst_n = 1'b0;

    // 提前声明（FSM 上面要用到）
    wire [7:0] loader_memory_error;
    wire       frame_write_toggle;
    integer    dst_words;

    // -------------------------------------------------------------------------
    // 复刻 sd_card_bmp：load_start_pulse(电平保持) / load_kick(单拍) / write_req
    // -------------------------------------------------------------------------
    reg        pulse_level;    // 对应 load_start_pulse
    reg        kick;           // 对应 load_kick
    reg        wr;             // 对应 write_req
    reg        wr_d;
    wire       wr_ack = wr & ~wr_d;   // 顶层：write_req 上升沿单拍应答
    wire       load_start = wr_ack;

    reg  [3:0] state;
    localparam S_IDLE   = 4'd0,
               S_HOLD   = 4'd1,    // load_start_pulse 保持（等 file_busy 上升沿）
               S_WAIT   = 4'd2,    // 读 FAT/文件头，没有像素
               S_STREAM = 4'd3,    // 像素流
               S_SETTLE = 4'd4;    // 等写完成 toggle / 超时

    integer    cnt, sub, px_left;
    reg        px_valid;
    reg  [1:0] image_index;

    reg        go;             // 主流程发起一次加载
    reg        attempt_done;   // FSM 回报本次加载结束
    integer    ls_cnt;         // 本次加载内 load_start 脉冲数
    integer    ls_cnt_rec [0:3];
    integer    err_rec    [0:3];
    integer    tgl_rec    [0:3];
    integer    words_rec  [0:3];
    integer    attempt;

    integer    toggle_cnt;
    reg        frame_write_toggle_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pulse_level <= 1'b0;
            kick        <= 1'b0;
            wr          <= 1'b0;
            wr_d        <= 1'b0;
            state       <= S_IDLE;
            cnt         <= 0;
            sub         <= 0;
            px_left     <= 0;
            px_valid    <= 1'b0;
            image_index <= 2'd0;
            attempt_done<= 1'b0;
            ls_cnt      <= 0;
        end else begin
            wr_d <= wr;
            kick <= 1'b0;                 // load_kick 单拍默认撤销

            // ---- 复刻的 write_req 逻辑（OLD / NEW 二选一）----
            if (OLD_MODE_EN) begin
                if (pulse_level)   wr <= 1'b1;
                else if (wr_ack)   wr <= 1'b0;
            end else begin
                if (kick)          wr <= 1'b1;
                else if (wr_ack)   wr <= 1'b0;
            end

            if (load_start) ls_cnt <= ls_cnt + 1;

            // ---- 加载序列 ----
            case (state)
                S_IDLE: begin
                    px_valid <= 1'b0;
                    if (go) begin
                        pulse_level  <= 1'b1;      // streamer 要看到的启动电平
                        kick         <= 1'b1;      // 写通道启动请求（单拍）
                        image_index  <= attempt[1:0];
                        ls_cnt       <= 0;
                        attempt_done <= 1'b0;
                        cnt          <= 0;
                        state        <= S_HOLD;
                    end
                end
                S_HOLD: begin
                    // 模拟 streamer 离开 IDLE：file_busy 上升 -> load_start_pulse 撤销
                    if (cnt == 40) begin
                        pulse_level <= 1'b0;
                        cnt         <= 0;
                        state       <= S_WAIT;
                    end else cnt <= cnt + 1;
                end
                S_WAIT: begin
                    if (cnt == FILE_OPEN_LAT) begin
                        cnt     <= 0;
                        sub     <= 0;
                        px_left <= IMAGE_WORDS;
                        state   <= S_STREAM;
                    end else cnt <= cnt + 1;
                end
                S_STREAM: begin
                    if (sub == PIXEL_GAP) begin
                        sub      <= 0;
                        px_valid <= 1'b1;
                    end else begin
                        sub      <= sub + 1;
                        px_valid <= 1'b0;
                    end
                    if (px_valid) begin                // 用上一拍的 px_valid
                        px_left <= px_left - 1;
                        if (px_left == 1) begin
                            cnt   <= 0;
                            state <= S_SETTLE;
                        end
                    end
                end
                S_SETTLE: begin
                    px_valid <= 1'b0;
                    cnt      <= cnt + 1;
                    if (cnt >= SETTLE_CYCLES) begin
                        ls_cnt_rec[attempt]  <= ls_cnt;
                        err_rec[attempt]     <= {24'd0, loader_memory_error};
                        tgl_rec[attempt]     <= toggle_cnt;
                        words_rec[attempt]   <= dst_words;
                        attempt_done         <= 1'b1;
                        state                <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // 真实 RTL：loader + CDC + 写提交
    // -------------------------------------------------------------------------
    wire        loader_load_busy, loader_load_done, loader_frame_commit;
    wire [3:0]  loader_image_complete;
    wire        loader_app_wr_en;
    wire [20:0] loader_app_wr_addr;
    wire [3:0]  loader_app_wr_mask;
    wire [31:0] loader_app_wr_data;

    wire        cdc_src_full, cdc_src_overflow, cdc_dst_idle;
    wire        cdc_app_wr_en;
    wire [20:0] cdc_app_wr_addr;
    wire [3:0]  cdc_app_wr_mask;
    wire [31:0] cdc_app_wr_data;
    wire        cdc_dst_wr_busy;

    reg cdc_idle_s1, cdc_idle_s2;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) {cdc_idle_s2, cdc_idle_s1} <= 2'b00;
        else        {cdc_idle_s2, cdc_idle_s1} <= {cdc_idle_s1, cdc_dst_idle};

    media_frame_loader #(
        .ADDR_WIDTH (21), .IMAGE_WORDS (IMAGE_WORDS), .FIFO_DEPTH (512),
        .BURST_SIZE (128), .STALL_TIMEOUT (65535),
        .WRITE_V_FLIP (1), .FRAME_WIDTH (640), .FRAME_HEIGHT (480)
    ) u_loader (
        .clk (clk), .rst_n (rst_n),
        .load_start (load_start), .image_index (image_index),
        .pixel_valid (px_valid),
        .pixel_rgb ({px_left[7:0], px_left[15:8], px_left[23:16]}),
        .frame_commit (loader_frame_commit),
        .decode_error (8'd0),
        .load_busy (loader_load_busy), .load_done (loader_load_done),
        .image_complete (loader_image_complete), .memory_error (loader_memory_error),
        .controller_init_done (1'b1), .controller_busy (1'b0), .refresh_active (1'b0),
        .app_wr_en (loader_app_wr_en), .app_wr_addr (loader_app_wr_addr),
        .app_wr_mask (loader_app_wr_mask), .app_wr_data (loader_app_wr_data),
        .out_full (1'b0)
    );

    media_write_commit #(
        .FRAME_PIXELS (IMAGE_WORDS), .DRAIN_GUARD_CYCLES (8'd255)
    ) u_commit (
        .clk (clk), .rst_n (rst_n),
        .load_start (load_start), .pixel_valid (px_valid),
        .load_done (loader_load_done), .memory_error (loader_memory_error),
        .cdc_idle (cdc_idle_s2),
        .frame_commit (loader_frame_commit), .frame_write_toggle (frame_write_toggle)
    );

    media_sdram_write_cdc #(
        .ADDR_WIDTH (21), .FIFO_ADDR_WIDTH (9), .BURST_SIZE (128)
    ) u_cdc (
        .src_clk (clk), .src_rst_n (rst_n),
        .src_wr_en (loader_app_wr_en), .src_wr_addr (loader_app_wr_addr),
        .src_wr_mask (loader_app_wr_mask), .src_wr_data (loader_app_wr_data),
        .src_full (cdc_src_full), .src_overflow (cdc_src_overflow),
        .src_frame_start (1'b0),
        .dst_clk (dst_clk), .dst_rst_n (rst_n),
        .dst_app_wr_en (cdc_app_wr_en), .dst_app_wr_addr (cdc_app_wr_addr),
        .dst_app_wr_mask (cdc_app_wr_mask), .dst_app_wr_data (cdc_app_wr_data),
        .dst_idle (cdc_dst_idle),
        .dst_rd_busy (1'b0), .dst_wr_busy (cdc_dst_wr_busy), .guard_dis (1'b0)
        .dst_ref_vld  (1'b0),
        .dst_blanking     (1'b1),          // new: tb 里恒为消隐期(老行为)
        .dst_wr_hold      (1'b0),          // 2026-10-29 端口（老 tb 里没接，必须补）
        .dst_wr_active    (),
    );

    // toggle 翻转计数（写完成握手次数）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            toggle_cnt            <= 0;
            frame_write_toggle_d  <= 1'b0;
        end else begin
            frame_write_toggle_d <= frame_write_toggle;
            if (frame_write_toggle !== frame_write_toggle_d)
                toggle_cnt <= toggle_cnt + 1;
        end
    end

    // SDRAM 侧的极简 sink：每拍接受一次写，统计字数
    integer dst_bank_words [0:3];
    always @(posedge dst_clk or negedge rst_n) begin
        if (!rst_n) begin
            dst_words <= 0;
            dst_bank_words[0] <= 0; dst_bank_words[1] <= 0;
            dst_bank_words[2] <= 0; dst_bank_words[3] <= 0;
        end else if (cdc_app_wr_en) begin
            dst_words <= dst_words + 1;
            case (cdc_app_wr_addr[20:19])
                2'd0: dst_bank_words[0] <= dst_bank_words[0] + 1;
                2'd1: dst_bank_words[1] <= dst_bank_words[1] + 1;
                2'd2: dst_bank_words[2] <= dst_bank_words[2] + 1;
                default: dst_bank_words[3] <= dst_bank_words[3] + 1;
            endcase
        end
    end


    // -------------------------------------------------------------------------
    // 主流程
    // -------------------------------------------------------------------------
    integer fail;
    task run_attempt;
        input integer idx;
        begin
            attempt = idx;
            @(negedge clk);
            go = 1'b1;
            wait (state == S_HOLD);          // FSM 已采样到 go（同时清零 attempt_done）
            @(negedge clk);
            go = 1'b0;
            wait (attempt_done == 1'b1);
            $display("[attempt %0d] load_start=%0d  error=0x%02h  toggle=%0d  words=%0d",
                     idx, ls_cnt_rec[idx], err_rec[idx][7:0], tgl_rec[idx], words_rec[idx]);
            if (ls_cnt_rec[idx] != 1) begin
                $display("  [FAIL] load_start should be 1, got %0d -> loader never started",
                         ls_cnt_rec[idx]);
                fail = fail + 1;
            end else if (words_rec[idx] - ((idx == 0) ? 0 : words_rec[idx-1]) != IMAGE_WORDS) begin
                $display("  [FAIL] words written this frame should be %0d", IMAGE_WORDS);
                fail = fail + 1;
            end else if (err_rec[idx] != 0) begin
                $display("  [FAIL] loader error 0x%02h", err_rec[idx][7:0]);
                fail = fail + 1;
            end else if (tgl_rec[idx] - ((idx == 0) ? 0 : tgl_rec[idx-1]) != 1) begin
                $display("  [FAIL] write toggle should flip exactly once");
                fail = fail + 1;
            end
            repeat (20) @(posedge clk);
        end
    endtask

    initial begin
        fail    = 0;
        attempt = 0;
        go      = 1'b0;
        dst_words = 0;
        #200 rst_n = 1'b1;
        #1000;

        $display("=========================================================");
        $display(" write_req logic : %s", (OLD_MODE_EN != 0) ? "OLD (level set + ack clear)" : "NEW (load_kick one-shot)");
        $display("=========================================================");

        run_attempt(0);     // 上电首图
        run_attempt(1);     // 按一次切换

        $display("---------------------------------------------------------");
        if (OLD_MODE_EN != 0) begin
            // 预期：第 2 帧 load_start=0、报 0x41、无写入 -> 复现实机现象
            if (ls_cnt_rec[1] == 0 && err_rec[1] == 8'h41 && (words_rec[1] - words_rec[0]) == 0)
                $display("RESULT: BUG REPRODUCED (frame2 load_start=0, ERR=0x41, zero words)");
            else
                $display("RESULT: UNEXPECTED (cannot reproduce)");
        end else begin
            if (fail == 0) $display("RESULT: PASS (each frame: 1 load_start, 307200 words, 1 commit)");
            else           $display("RESULT: FAIL (%0d checks)", fail);
        end
        $display("---------------------------------------------------------");
        $finish;
    end

endmodule
