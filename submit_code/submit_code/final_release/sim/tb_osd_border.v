`timescale 1ns/1ps
module tb_osd_border;
reg clk=0; always #20 clk=~clk;
reg rst=1;
reg de=0, vs=0;
reg [9:0] x=0;
reg [11:0] hcnt=0; reg [9:0] vcnt=0;
wire [23:0] vout;
osd_overlay u(.clk(clk),.rst(rst),.de(de),.vs(vs),.video_x(x),
    .video_in(24'd0),.sd_init_done(1'b1),.img_idx(2'd0),
    .img_total(3'd4),.bmp_err_code(3'd0),.video_out(vout));
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
reg vs_d;
integer white_px, black_px;
always @(posedge clk) begin
    if(rst) begin white_px<=0; black_px<=0; end
    else begin
        if(u.osd_active) white_px<=white_px+1;
        if(u.osd_border) black_px<=black_px+1;
        if(vs && !vs_d) begin
            $display("FRAME: white=%0d black_border=%0d", white_px, black_px);
            white_px<=0; black_px<=0;
        end
        vs_d<=vs;
    end
end
initial begin
    rst=1; repeat(10) @(posedge clk); rst=0;
    repeat(420000*3) @(posedge clk);
    $finish;
end
endmodule
