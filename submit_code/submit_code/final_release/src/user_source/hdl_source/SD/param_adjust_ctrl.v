`timescale 1ns / 1ps
// =====================================================================
// param_adjust_ctrl —— 亮度 / 对比度按键调节控制器（第 14 讲）
//
//   4 个按键（低有效），各管一个方向：
//     key_b_inc_n : 亮度 +
//     key_b_dec_n : 亮度 -
//     key_c_inc_n : 对比度 +
//     key_c_dec_n : 对比度 -
//
//   短按 = 步进一次；按住超过 REPEAT_DELAY_MS 后每 REPEAT_RATE_MS 连发一次。
//   步长 STEP=16：亮度 0~255 共 16 档，对比度 0~255 共 16 档，
//   两端饱和（到顶/到底就停，不回绕 —— 回绕会让亮度突然跳到全黑，吓人）。
//
//   默认值：brightness=255（原始亮度）、contrast=128（原始对比度），
//   即上电画面与第 9 讲完全一致，可作严格回归基线。
//
//   调节后 show_en 拉高 SHOW_MS（3 秒）供 OSD 回显参数，之后自动隐藏。
//
//   时钟域：clk（50MHz 控制域）。输出的 brightness/contrast 由像域模块
//           自行 2FF + vsync 帧边界锁存，本模块不关心像素时序。
// =====================================================================
module param_adjust_ctrl #(
    parameter integer CLKS_PER_MS    = 50_000,   // 50MHz
    parameter integer DEBOUNCE_MS    = 20,       // 20ms 消抖
    parameter integer REPEAT_DELAY_MS= 400,      // 按住 400ms 后开始连发
    parameter integer REPEAT_RATE_MS = 80,       // 连发间隔 80ms
    parameter integer SHOW_MS        = 3000,     // 参数显示保留 3 秒
    parameter integer STEP           = 16        // 每步 16
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       key_b_inc_n,
    input  wire       key_b_dec_n,
    input  wire       key_c_inc_n,
    input  wire       key_c_dec_n,
    output reg  [7:0] brightness,
    output reg  [7:0] contrast,
    output reg        show_en,      // 1 = 正在显示参数
    output reg        any_pulse     // 任一按键有效步进（调试/串口用）
);
    localparam [31:0] DB_MAX    = DEBOUNCE_MS     * CLKS_PER_MS;
    localparam [31:0] REP_DELAY = REPEAT_DELAY_MS * CLKS_PER_MS;
    localparam [31:0] REP_RATE  = REPEAT_RATE_MS  * CLKS_PER_MS;
    localparam [31:0] SHOW_MAX  = SHOW_MS         * CLKS_PER_MS;

    wire [3:0] key_n = {key_c_dec_n, key_c_inc_n, key_b_dec_n, key_b_inc_n};
    wire [3:0] key_pulse;

    // =================================================================
    // 四路按键：2FF 同步 → 20ms 消抖 → 按下沿立即触发一次
    //           → 按住超过 REP_DELAY 后每 REP_RATE 连发
    // =================================================================
    genvar gi;
    generate
        for (gi = 0; gi < 4; gi = gi + 1) begin : g_key
            reg [1:0]  sync;
            reg        raw;          // 消抖中的当前参考电平
            reg        st;           // 消抖后的稳定电平（0=按下）
            reg        st_d;
            reg [31:0] db_cnt;
            reg [31:0] hold_cnt;
            reg        pulse;

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    sync     <= 2'b11;
                    raw      <= 1'b1;
                    st       <= 1'b1;
                    st_d     <= 1'b1;
                    db_cnt   <= 32'd0;
                    hold_cnt <= 32'd0;
                    pulse    <= 1'b0;
                end else begin
                    pulse <= 1'b0;

                    // ---- 两级同步 + 消抖 ----
                    sync <= {sync[0], key_n[gi]};
                    if (sync[1] != raw) begin
                        if (db_cnt >= DB_MAX - 1'b1) begin
                            raw    <= sync[1];
                            st     <= sync[1];
                            db_cnt <= 32'd0;
                        end else begin
                            db_cnt <= db_cnt + 1'b1;
                        end
                    end else begin
                        db_cnt <= 32'd0;
                    end

                    // ---- 步进触发 ----
                    st_d <= st;
                    if (st_d && !st) begin
                        // 按下瞬间立刻响应一次（短按体验）
                        pulse    <= 1'b1;
                        hold_cnt <= 32'd0;
                    end else if (!st) begin
                        // 持续按住：到 REP_DELAY 触发，之后每 REP_RATE 一次
                        if (hold_cnt >= REP_DELAY - 1'b1) begin
                            pulse    <= 1'b1;
                            hold_cnt <= REP_DELAY - REP_RATE;
                        end else begin
                            hold_cnt <= hold_cnt + 1'b1;
                        end
                    end else begin
                        hold_cnt <= 32'd0;
                    end
                end
            end

            assign key_pulse[gi] = pulse;
        end
    endgenerate

    wire p_b_inc = key_pulse[0];
    wire p_b_dec = key_pulse[1];
    wire p_c_inc = key_pulse[2];
    wire p_c_dec = key_pulse[3];
    wire p_any   = |key_pulse;

    // =================================================================
    // 参数累加（饱和到 0 / 255）
    // =================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            brightness <= 8'd255;      // 原始亮度
            contrast   <= 8'd128;      // 原始对比度
            any_pulse  <= 1'b0;
        end else begin
            any_pulse <= p_any;
            if (p_b_inc) begin
                brightness <= (brightness > (8'd255 - STEP)) ? 8'd255
                                                             : brightness + STEP[7:0];
            end else if (p_b_dec) begin
                brightness <= (brightness < STEP[7:0]) ? 8'd0
                                                       : brightness - STEP[7:0];
            end else if (p_c_inc) begin
                contrast   <= (contrast > (8'd255 - STEP)) ? 8'd255
                                                           : contrast + STEP[7:0];
            end else if (p_c_dec) begin
                contrast   <= (contrast < STEP[7:0]) ? 8'd0
                                                     : contrast - STEP[7:0];
            end
        end
    end

    // =================================================================
    // 显示窗口：任一调节动作把计时器重装为 3 秒，倒计时期间 show_en=1
    // =================================================================
    reg [31:0] show_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            show_cnt <= 32'd0;
            show_en  <= 1'b0;
        end else begin
            if (p_any) begin
                show_cnt <= SHOW_MAX;
                show_en  <= 1'b1;
            end else if (show_cnt != 32'd0) begin
                show_cnt <= show_cnt - 1'b1;
                show_en  <= 1'b1;
            end else begin
                show_en  <= 1'b0;
            end
        end
    end

endmodule
