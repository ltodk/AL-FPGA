
module seg_decoder
(
	input[3:0]      bin_data,     // bin data input
	output reg[6:0] seg_data      // seven segments LED output
);

always@(*)
begin
	// 共阴数码管：段码 bit 为 1 表示该段点亮（与原共阳段码逐位取反）
	case(bin_data)
		4'd0:seg_data <= 7'b011_1111;
		4'd1:seg_data <= 7'b000_0110;
		4'd2:seg_data <= 7'b101_1011;
		4'd3:seg_data <= 7'b100_1111;
		4'd4:seg_data <= 7'b110_0110;
		4'd5:seg_data <= 7'b110_1101;
		4'd6:seg_data <= 7'b111_1101;
		4'd7:seg_data <= 7'b000_0111;
		4'd8:seg_data <= 7'b111_1111;
		4'd9:seg_data <= 7'b110_1111;
		4'ha:seg_data <= 7'b111_0111;
		4'hb:seg_data <= 7'b111_1100;
		4'hc:seg_data <= 7'b011_1001;
		4'hd:seg_data <= 7'b101_1110;
		4'he:seg_data <= 7'b111_1001;
		4'hf:seg_data <= 7'b111_0001;
		default:seg_data <= 7'b000_0000;
	endcase
end
endmodule