// tb_decoder_errcodes: 验证 bmp24_decoder 的 8 位细分错误码
// 逐项喂入 6 种坏头，验证 0x31~0x36 精确区分
`timescale 1ns/1ps
module tb_decoder_errcodes;

reg clk = 0;
always #5 clk = ~clk;   // 100MHz

reg rst = 1;
reg [7:0] byte_in = 0;
reg       byte_valid = 0;
reg [9:0] byte_idx = 0;

wire header_match;
wire [7:0] bmp_error;
wire [7:0] header_0_out, header_1_out;
wire [31:0] file_len_out, pixel_offset_out;

// 像素组装接口（本 TB 不测像素，固定关）
wire pixel_valid;
wire [23:0] pixel_rgb;

bmp24_decoder u_dec (
    .clk(clk), .rst(rst),
    // 第13讲：这两个端口从"期望尺寸"改成"支持上限"，校验变成 1 <= w <= max
    .max_width(16'd2048), .max_height(16'd1536),
    .hdr_parse_en(1'b1),
    .hdr_byte(byte_in), .hdr_byte_valid(byte_valid), .hdr_byte_idx(byte_idx),
    .header_match(header_match), .bmp_error(bmp_error),
    .header_0_out(header_0_out), .header_1_out(header_1_out),
    .file_len_out(file_len_out), .pixel_offset_out(pixel_offset_out),
    .img_width_out(), .img_height_out(),
    .px_assemble_en(1'b0), .px_byte(8'd0), .px_byte_idx(2'd0),
    .pixel_valid(pixel_valid), .pixel_rgb(pixel_rgb)
);

// 喂一个完整 BMP 头（54字节），字段由任务参数指定
task feed_header;
    input [7:0] m0, m1;          // 签名
    input [31:0] flen;           // file_len
    input [31:0] off;            // pixel_offset
    input [31:0] dib;            // DIB 头大小
    input [31:0] w, h;           // 宽高
    input [15:0] planes_v;       // 平面数
    input [15:0] bpp;            // 位深
    input [31:0] comp;           // 压缩
    integer k;
    begin
        // 清零字段寄存器（复位后默认已清，但先复位一次确保干净）
        @(posedge clk); rst = 1;
        @(posedge clk); rst = 0;
        @(posedge clk);

        // 喂 54 字节，逐字节带 valid
        for (k = 0; k < 54; k = k + 1) begin
            byte_idx = k;
            case (k)
                0:  byte_in = m0;
                1:  byte_in = m1;
                2:  byte_in = flen[7:0];
                3:  byte_in = flen[15:8];
                4:  byte_in = flen[23:16];
                5:  byte_in = flen[31:24];
                10: byte_in = off[7:0];
                11: byte_in = off[15:8];
                12: byte_in = off[23:16];
                13: byte_in = off[31:24];
                14: byte_in = dib[7:0];
                15: byte_in = dib[15:8];
                16: byte_in = dib[23:16];
                17: byte_in = dib[31:24];
                18: byte_in = w[7:0];
                19: byte_in = w[15:8];
                20: byte_in = w[23:16];
                21: byte_in = w[31:24];
                22: byte_in = h[7:0];
                23: byte_in = h[15:8];
                24: byte_in = h[23:16];
                25: byte_in = h[31:24];
                26: byte_in = planes_v[7:0];
                27: byte_in = planes_v[15:8];
                28: byte_in = bpp[7:0];
                29: byte_in = bpp[15:8];
                30: byte_in = comp[7:0];
                31: byte_in = comp[15:8];
                32: byte_in = comp[23:16];
                33: byte_in = comp[31:24];
                default: byte_in = 8'd0;
            endcase
            byte_valid = 1;
            @(posedge clk);
        end
        byte_valid = 0;
        @(posedge clk);
    end
endtask

integer errors = 0;
reg [7:0] err_obs;

// 观测：锁存当前错误码
always @(posedge clk) err_obs <= bmp_error;

task check_err;
    input [7:0] want;
    input [8*8-1:0] name;
    begin
        @(posedge clk);
        if (err_obs == want) begin
            $display("PASS: %0s -> 0x%0h", name, err_obs);
        end else begin
            errors = errors + 1;
            $display("FAIL: %0s 期望 0x%0h 实际 0x%0h", name, want, err_obs);
        end
    end
endtask

// 合法头基线（640x480 24bpp，file_len=54+640*480*3=921654，off=54）
localparam integer GOOD_FLEN = 54 + 640*480*3;
// 第13讲：任意分辨率的 file_len 必须按真实 W*H 算，否则会被判 ERR_TRUNCATED
localparam integer FLEN_800x600 = 54 + 800*600*3;
localparam integer FLEN_1280x720= 54 + 1280*720*3;

initial begin
    rst = 1; repeat (5) @(posedge clk); rst = 0;

    // 合法头 -> ERR_NONE (0x00)
    $display("=== 合法头 ===");
    feed_header("B","M", GOOD_FLEN, 54, 40, 640, 480, 1, 24, 0);
    check_err(8'h00, "640x480");

    // ★ 第13讲新增：非 640×480 的分辨率必须合法（这是本讲的核心目标）
    $display("=== 任意分辨率（应合法）===");
    feed_header("B","M", FLEN_800x600, 54, 40, 800, 600, 1, 24, 0);
    check_err(8'h00, "800x600");
    feed_header("B","M", FLEN_1280x720, 54, 40, 1280, 720, 1, 24, 0);
    check_err(8'h00, "1280x720");
    feed_header("B","M", 54+320*240*3, 54, 40, 320, 240, 1, 24, 0);
    check_err(8'h00, "320x240");

    // 0x31 签名错
    $display("=== 0x31 签名 ===");
    feed_header("X","X", GOOD_FLEN, 54, 40, 640, 480, 1, 24, 0);
    check_err(8'h31, "签名错");

    // 0x32 尺寸超上限（2048x1536 之外的图）
    $display("=== 0x32 尺寸超上限 ===");
    feed_header("B","M", GOOD_FLEN, 54, 40, 4000, 3000, 1, 24, 0);
    check_err(8'h32, "4000x3000 超上限");
    // 0x32 尺寸为 0
    feed_header("B","M", GOOD_FLEN, 54, 40, 0, 480, 1, 24, 0);
    check_err(8'h32, "宽为0");

    // 0x33 平面数错
    $display("=== 0x33 平面 ===");
    feed_header("B","M", GOOD_FLEN, 54, 40, 640, 480, 2, 24, 0);
    check_err(8'h33, "平面数错");

    // 0x34 位深错
    $display("=== 0x34 位深 ===");
    feed_header("B","M", GOOD_FLEN, 54, 40, 640, 480, 1, 8, 0);
    check_err(8'h34, "位深错");

    // 0x35 压缩错
    $display("=== 0x35 压缩 ===");
    feed_header("B","M", GOOD_FLEN, 54, 40, 640, 480, 1, 24, 1);
    check_err(8'h35, "压缩错");

    // 0x36 偏移错（off 太小）
    $display("=== 0x36 偏移 ===");
    feed_header("B","M", GOOD_FLEN, 10, 40, 640, 480, 1, 24, 0);
    check_err(8'h36, "偏移错");

    $display("=== 结果: %0s (errors=%0d) ===", errors==0?"ALL PASS":"HAS FAIL", errors);
    $finish;
end

initial begin #100_000_000; $display("TIMEOUT"); $finish; end
endmodule
