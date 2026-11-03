`timescale 1ns/1ps
// ============================================================================
// tb_audio_mux.v —— audio_source_mux 单元测试（第 11 讲 阶段 A）
//
// 被 TMUX 的核心不是数据选择（三目运算符很难写错），而是 **ready 路由**：
// 未选中的源必须收到 ready=0，否则它会「边产生边丢弃」，切回来时音频跳段。
// 所以本 TB 把 ready 的所有组合都扫一遍。
//
// 纯组合模块，不需要时钟，直接给输入看输出。
// ============================================================================
module tb_audio_mux;

    reg               wav_valid;
    wire              wav_ready;
    reg  signed [15:0] wav_left, wav_right;
    reg               wav_active;

    reg               arp_valid;
    wire              arp_ready;
    reg  signed [15:0] arp_left, arp_right;

    reg               force_arpeggio;
    wire              out_valid;
    reg               out_ready;
    wire signed [15:0] out_left, out_right;

    integer pass_cnt = 0;
    integer fail_cnt = 0;

    audio_source_mux dut (
        .wav_valid      (wav_valid),
        .wav_ready      (wav_ready),
        .wav_left       (wav_left),
        .wav_right      (wav_right),
        .wav_active     (wav_active),
        .arp_valid      (arp_valid),
        .arp_ready      (arp_ready),
        .arp_left       (arp_left),
        .arp_right      (arp_right),
        .force_arpeggio (force_arpeggio),
        .out_valid      (out_valid),
        .out_ready      (out_ready),
        .out_left       (out_left),
        .out_right      (out_right)
    );

    task check;
        input            cond;
        // 128 字节：ModelSim 的 %0s 对位向量有硬上限，原 256 位(32字节)会把长
        // 描述的前半截吃掉，看起来像"报错内容错位"。这里放大到 128 字节。
        input [8*128-1:0] name;
        begin
            if (cond) begin
                pass_cnt = pass_cnt + 1;
                $display("[ OK ] %0s", name);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] %0s", name);
            end
        end
    endtask

    initial begin
        // 初值：两路数据故意取不同值，任何串路都会立刻暴露
        wav_valid = 1'b1;  wav_left = 16'sd1000;  wav_right = -16'sd2000;
        arp_valid = 1'b1;  arp_left = 16'sd30000; arp_right = 16'sd12345;
        wav_active = 1'b0; force_arpeggio = 1'b0; out_ready = 1'b1;

        $display("\n=== T1 默认态（wav_active=0）应走琶音 ===");
        #1;
        check(out_valid === arp_valid,       "T1.1 out_valid from arp");
        check(out_left  === 16'sd30000,      "T1.2 out_left from arp");
        check(out_right === 16'sd12345,      "T1.3 out_right from arp");
        check(arp_ready === 1'b1,            "T1.4 arp_ready = out_ready");
        check(wav_ready === 1'b0,            "T1.5 wav_ready = 0 (unselected src not drained)");

        $display("\n=== T2 WAV 播放中（wav_active=1）应走 WAV ===");
        wav_active = 1'b1;
        #1;
        check(out_valid === wav_valid,       "T2.1 out_valid from WAV");
        check(out_left  === 16'sd1000,       "T2.2 out_left from WAV");
        check(out_right === -16'sd2000,      "T2.3 out_right from WAV (negative passthru)");
        check(wav_ready === 1'b1,            "T2.4 wav_ready = out_ready");
        check(arp_ready === 1'b0,            "T2.5 arp_ready = 0 (arp held, no data loss)");

        $display("\n=== T3 强制琶音优先于 wav_active ===");
        force_arpeggio = 1'b1;
        #1;
        check(out_left === 16'sd30000,       "T3.1 force_arp: out_left from arp");
        check(arp_ready === 1'b1,            "T3.2 force_arp: arp_ready = out_ready");
        check(wav_ready === 1'b0,            "T3.3 force_arp: wav_ready = 0");

        $display("\n=== T4 下游不 ready（背压）===");
        force_arpeggio = 1'b0; out_ready = 1'b0;
        #1;
        check(arp_ready === 1'b0,            "T4.1 wav mode: arp_ready always 0");
        check(wav_ready === 1'b0,            "T4.2 downstream not ready -> wav_ready=0");
        wav_active = 1'b0;
        #1;
        check(arp_ready === 1'b0,            "T4.3 arp mode: downstream not ready -> arp_ready=0");

        $display("\n=== T5 valid 透传不被 ready 影响 ===");
        out_ready = 1'b1; wav_active = 1'b1; wav_valid = 1'b0;
        #1;
        check(out_valid === 1'b0,            "T5.1 WAV valid=0 -> out_valid=0");
        wav_valid = 1'b1;
        #1;
        check(out_valid === 1'b1,            "T5.2 WAV valid=1 -> out_valid=1");

        $display("\n=== T6 极端有符号值 ===");
        // 用位模式而不是 -16'sd32768：后者在 16 位下负溢出，写法容易误导
        wav_left = 16'sh8000;    // = -32768（负满幅）
        wav_right = 16'sh7FFF;   // = +32767（正满幅）
        #1;
        check(out_left === 16'sh8000,        "T6.1 full-scale -32768 passthru");
        check(out_right === 16'sh7FFF,       "T6.2 full-scale +32767 passthru");

        // ---- 汇总 ----
        $display("\n========================================");
        $display("RESULT: %0d passed, %0d failed", pass_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");
        $display("========================================");
        $finish;
    end

endmodule
