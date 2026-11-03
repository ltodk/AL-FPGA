`timescale 1ns/1ps
module tb_osd_latch;
reg clk=0; always #20 clk=~clk;
reg rst=1;
reg de=0, vs=0;
reg [9:0] x=0;
reg [11:0] hcnt=0; reg [9:0] vcnt=0;
reg [1:0] img_idx=0;
reg [2:0] img_total=4;
wire [23:0] vout;
osd_overlay u(.clk(clk),.rst(rst),.de(de),.vs(vs),.video_x(x),
    .video_in(24'd0),.sd_init_done(1'b1),.img_idx(img_idx),
    .img_total(img_total),.bmp_err_code(3'd0),.video_out(vout));
always @(posedge clk) begin
    if(rst) begin hcnt<=0; vcnt<=0; end
    else begin
        if(hcnt==799) begin hcnt<=0; if(vcnt==524) vcnt<=0; else vcnt<=vcnt+1; end
        else hcnt<=hcnt+1;
    end
end
always @(*) begin
    de = (hcnt>=160)&&(vcnt<480);
    vs = ~((vcnt>=490)&&(vcnt<=491));
    x  = (hcnt>=160)?(hcnt[9:0]-160):(hcnt[9:0]+640);
end
// 观察 tot_show
wire [2:0] ts = u.tot_show;
integer px;
reg vs_d;
always @(posedge clk) begin
    if(rst) px<=0;
    else begin
        if(u.osd_active) px<=px+1;
        if(vs && !vs_d) begin
            $display("FRAME: img_total=%0d tot_show=%0d osd_px=%0d", img_total, ts, px);
            px<=0;
        end
        vs_d<=vs;
    end
end
initial begin
    rst=1; repeat(10) @(posedge clk); rst=0;
    // 阶段1: total=4，跑几帧
    img_total=4; repeat(420000*4) @(posedge clk);
    // 阶段2: total=0（模拟重扫），IMG行应靠锁存保持
    img_total=0; repeat(420000*4) @(posedge clk);
    // 阶段3: total恢复4
    img_total=4; repeat(420000*4) @(posedge clk);
    $finish;
end
endmodule
