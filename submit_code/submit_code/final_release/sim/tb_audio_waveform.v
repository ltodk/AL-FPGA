// ============================================================================
// tb_audio_waveform: audio_waveform_overlay unit regression (Lecture 12)
//
//   W1  decimation: store 1 of every 8 samples, all 64 points checked
//   W2  peak includes NON-decimated samples  <-- fix vs reference impl
//   W3  double buffer swap at frame end
//   W4  not enough samples -> no swap, keep previous frame
//   W5+ display side: dim bg / axis / wave pos+neg / thickness / peak bar / crop / pass-through
//
//   Display side uses force on internal line_y to avoid running a full
//   480-line scan; combinational logic re-evaluates #1 after de/x/enable change.
//
//   NOTE: all messages are ASCII on purpose - non-ASCII in $display makes
//   grep treat the log as binary and truncate the result summary.
// ============================================================================
`timescale 1ns/1ps

module tb_audio_waveform;

localparam integer WAVE_X = 64;
localparam integer WAVE_Y = 320;
localparam integer WAVE_W = 512;
localparam integer WAVE_H = 64;
localparam integer N       = 256;
localparam integer DEC     = 2;
localparam integer CY      = WAVE_Y + WAVE_H/2 - 1;   // 351
localparam integer PX0     = WAVE_X + WAVE_W + 8;     // 584
localparam integer WY1     = WAVE_Y + WAVE_H;         // 384

localparam [23:0] C_WAVE = 24'h20FF90;
localparam [23:0] C_PEAK = 24'hFFB020;
localparam [23:0] C_AXIS = 24'h406060;
localparam [23:0] C_DIM  = 24'h7F7F7F;   // 24'hFFFFFF with brightness halved
localparam [23:0] C_PASS = 24'hFFFFFF;   // pass-through

reg                clk = 1'b0;
reg                rst = 1'b1;
reg                de  = 1'b0;
reg                vs  = 1'b1;
reg  [9:0]         video_x   = 10'd0;
reg  [23:0]        video_in  = 24'hFFFFFF;
reg                enable    = 1'b0;
reg                pcm_valid = 1'b0;
reg  signed [15:0] pcm_sample = 16'sd0;
wire [23:0]        video_out;
wire               ovl;

integer errors = 0;
integer i;
reg [23:0] got;

always #20 clk = ~clk;   // 25MHz = 40ns

audio_waveform_overlay #(
    .WAVE_X       (WAVE_X),
    .WAVE_Y       (WAVE_Y),
    .WAVE_WIDTH   (WAVE_W),
    .WAVE_HEIGHT  (WAVE_H),
    .DECIMATE     (DEC),
    .SAMPLE_COUNT (N)
) dut (
    .clk            (clk),
    .rst            (rst),
    .de             (de),
    .vs             (vs),
    .video_x        (video_x),
    .video_in       (video_in),
    .enable         (enable),
    .pcm_valid      (pcm_valid),
    .pcm_sample     (pcm_sample),
    .video_out      (video_out),
    .overlay_active (ovl)
);

// ---------------------------------------------------------------------------
task check_int(input [199:0] name, input integer got_v, input integer exp_v);
    begin
        if (got_v === exp_v)
            $display("  PASS  %0s = %0d", name, got_v);
        else begin
            $display("  FAIL  %0s = %0d (expected %0d)", name, got_v, exp_v);
            errors = errors + 1;
        end
    end
endtask

task check_hex(input [199:0] name, input [23:0] got_v, input [23:0] exp_v);
    begin
        if (got_v === exp_v)
            $display("  PASS  %0s = %06h", name, got_v);
        else begin
            $display("  FAIL  %0s = %06h (expected %06h)", name, got_v, exp_v);
            errors = errors + 1;
        end
    end
endtask

// vs falling edge = one frame finished
task frame_end;
    begin
        @(negedge clk); vs = 1'b0;
        @(negedge clk); vs = 1'b1;
        @(negedge clk);
    end
endtask

// Feed cnt pcm samples.
//   normal sample = (k % 100) << 8   -> upper 8 bits = k%100 (always positive)
//   k == 131      = 16'h8100         -> upper 8 bits 0x81, abs = 127 (frame max)
//   131 is ODD and DECIMATE=2, so it is NOT a decimation point. A peak
//   detector that only looks at decimated samples will miss it.
task feed(input integer cnt);
    integer k;
    begin
        for (k = 0; k < cnt; k = k + 1) begin
            @(negedge clk);
            pcm_valid  = 1'b1;
            pcm_sample = (k == 131) ? 16'h8100 : ((k % 100) << 8);
            @(negedge clk);
            pcm_valid  = 1'b0;
        end
        @(negedge clk);
    end
endtask

// Probe one pixel: force internal line_y, set coords, read combinational output
task probe(input integer y, input integer x, input integer de_v, input integer en);
    begin
        @(negedge clk);
        force dut.line_y = y[9:0];
        de       = de_v[0];
        video_x  = x[9:0];
        enable   = en[0];
        #1;
        got = video_out;
    end
endtask

// ---------------------------------------------------------------------------
initial begin
    $display("");
    $display("================ tb_audio_waveform ================");

    rst = 1'b1; vs = 1'b1; de = 1'b0;
    repeat (4) @(negedge clk);
    rst = 1'b0;
    repeat (2) @(negedge clk);

    // ===================== W1 / W2: capture frame 0 =====================
    $display("-- W1 decimation: 1 of every 8 samples (all 64 points) --");
    frame_end();
    feed(N * DEC);               // 512 samples
    frame_end();                 // frame ends -> swap

    begin : W1_check
        integer bad;
        bad = 0;
        for (i = 0; i < N; i = i + 1) begin
            // bank0[i] must hold upper 8 bits of sample #(i*2) = (i*2) % 100
            if ((dut.sample_bank0[i] & 8'hFF) !== ((i*2) % 100))
                bad = bad + 1;
        end
        check_int("bank0_mismatch_count(exp 0)", bad, 0);
        check_int("bank0[0]",  (dut.sample_bank0[0]  & 8'hFF), 0);
        check_int("bank0[1]",  (dut.sample_bank0[1]  & 8'hFF), 2);
        check_int("bank0[2]",  (dut.sample_bank0[2]  & 8'hFF), 4);
        check_int("bank0[12]", (dut.sample_bank0[12] & 8'hFF), 24);
        check_int("bank0[13]", (dut.sample_bank0[13] & 8'hFF), 26);
        check_int("bank0[63]", (dut.sample_bank0[63] & 8'hFF), (63*2) % 100);
    end

    $display("-- W2 peak must include non-decimated samples --");
    check_int("peak_bank0(exp 127 from k=131)", dut.peak_bank0, 127);

    $display("-- W3 double buffer swap at frame end --");
    check_int("display_valid",        dut.display_valid, 1);
    check_int("display_bank(exp 0)",  dut.display_bank,  0);
    check_int("write_bank(exp 1)",    dut.write_bank,    1);
    check_int("display_peak(exp 127)",dut.display_peak,  127);

    // ===================== W4: not enough samples =====================
    $display("-- W4 only 100 samples: no swap, keep previous frame --");
    frame_end();
    feed(100);                   // 100 samples -> 50 decimation points (k=0,2,...,98)
    check_int("capture_count(exp 50)", dut.capture_count, 50);
    frame_end();
    check_int("display_bank unchanged(0)", dut.display_bank, 0);
    check_int("display_valid kept(1)",     dut.display_valid, 1);
    check_int("write_bank unchanged(1)",   dut.write_bank,    1);

    // ===================== W5+: display side =====================
    $display("-- W5 outside panel: pass through --");
    probe(300, 72, 1, 1);
    check_hex("y=300 above panel", got, C_PASS);
    probe(400, 72, 1, 1);
    check_hex("y=400 below panel", got, C_PASS);
    probe(330, 40, 1, 1);
    check_hex("x=40 left of panel", got, C_PASS);
    probe(330, 578, 1, 1);       // 576..583 gap between wave area and peak bar
    check_hex("x=578 gap", got, C_PASS);

    $display("-- W6 enable / de off: pass through --");
    probe(330, 72, 1, 0);
    check_hex("enable=0", got, C_PASS);
    probe(330, 72, 0, 1);
    check_hex("de=0", got, C_PASS);

    $display("-- W7 panel dim background --");
    probe(330, 72, 1, 1);
    check_hex("y=330 x=72 dim", got, C_DIM);
    probe(320, 64, 1, 1);
    check_hex("top-left corner y=320", got, C_DIM);

    $display("-- W8 center axis --");
    // x=72 -> index 4 (bank0[4]=8, ya=349) and index 5 (bank0[5]=10, yb=349)
    // vertical link covers 348..350, so y=351 shows only the axis
    probe(CY, 72, 1, 1);
    check_hex("y=351 axis", got, C_AXIS);

    $display("-- W9 wave pixel (positive sample) --");
    probe(349, 72, 1, 1);
    check_hex("y=349 wave center", got, C_WAVE);
    probe(348, 72, 1, 1);
    check_hex("y=348 upper edge", got, C_WAVE);
    probe(350, 72, 1, 1);
    check_hex("y=350 lower edge", got, C_WAVE);
    probe(347, 72, 1, 1);
    check_hex("y=347 outside thickness", got, C_DIM);

    $display("-- W10 wave pixel (negative sample, forced) --");
    force dut.sample_bank0[0] = 8'h80;      // -128 -> sample_y = 351+32 = 383
    probe(WY1-1, 64, 1, 1);
    check_hex("y=383 full negative (no clip)", got, C_WAVE);
    force dut.sample_bank0[0] = 8'h40;      // +64 -> sample_y = 351-16 = 335
    probe(335, 64, 1, 1);
    check_hex("y=335 positive +64", got, C_WAVE);
    release dut.sample_bank0[0];

    $display("-- W11 peak bar --");
    // display_peak=127 -> peak_height=31 -> orange from 384-31=353 to 383
    probe(383, PX0+2, 1, 1);
    check_hex("peak bar top y=383", got, C_PEAK);
    probe(353, PX0+2, 1, 1);
    check_hex("peak bar bottom y=353", got, C_PEAK);
    probe(352, PX0+2, 1, 1);
    check_hex("y=352 above peak bar", got, C_DIM);
    probe(330, PX0+2, 1, 1);
    check_hex("y=330 peak bar plate", got, C_DIM);
    probe(383, PX0-1, 1, 1);
    check_hex("x=583 left of peak bar", got, C_PASS);

    $display("-- W12 wave area right boundary --");
    // index = (575-64)/2 = 255 (last point) -> bank0[255]=10 -> y span 348..350
    probe(340, 575, 1, 1);
    check_hex("x=575 last point", got, C_DIM);
    probe(330, 576, 1, 1);
    check_hex("x=576 right of wave area", got, C_PASS);

    release dut.line_y;
    de = 1'b0; enable = 1'b0;

    // ===================== summary =====================
    $display("");
    if (errors == 0)
        $display(" RESULT: ALL_PASS");
    else
        $display(" RESULT: %0d FAILED", errors);
    $display("==================================================");
    $finish;
end

endmodule
