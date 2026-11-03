
module sd_card_sec_read_write
#(
	parameter  SPI_LOW_SPEED_DIV = 248,         // spi clk speed = clk speed /((SPI_LOW_SPEED_DIV + 2) * 2 )
	parameter  SPI_HIGH_SPEED_DIV = 0,          // spi clk speed = clk speed /((SPI_HIGH_SPEED_DIV + 2) * 2 )
	// 【拔插卡死锁修复】空闲保活探测：SPI 模式没有热插拔检测引脚，卡被拔掉/插回
	// 后若一直没人读，本层永远不知道卡已经换了（重新上电的卡处于原生 SD 模式，
	// 不响应 SPI 命令），于是一直显示旧图、下一次读才失败。这里在空闲时周期性
	// 发 CMD13(SEND_STATUS) 探活：有应答=卡在位；无应答=卡被拔/被换 → 回 IDLE
	// 完整重新初始化（CMD0/CMD8/CMD55/CMD41/CMD16），插回后自动恢复读卡。
	parameter  integer PROBE_EN = 1,                  // 1=使能空闲探活
	parameter  integer PROBE_GAP_CYCLES = 50_000_000, // 探活间隔（默认 500ms @100MHz）
	parameter  integer PWR_WAIT_CYCLES  = 5_000_000   // 上电/重新初始化前的稳定延时（默认 50ms）
)
(
	input            clk,
	input            rst,
	output reg       sd_init_done,
	input            sd_sec_read,
	input[31:0]      sd_sec_read_addr,
	output[7:0]      sd_sec_read_data,
	output           sd_sec_read_data_valid,
	output           sd_sec_read_end,
	input            sd_sec_write,
	input[31:0]      sd_sec_write_addr,
	input[7:0]       sd_sec_write_data,
	output           sd_sec_write_data_req,
	output           sd_sec_write_end,

	output reg[15:0] spi_clk_div,
	output reg       cmd_req,
	input            cmd_req_ack,
	input            cmd_req_error,
	output reg[47:0] cmd,
	output reg[7:0]  cmd_r1,
	output reg[15:0] cmd_data_len,
	output reg       block_read_req,
	input            block_read_valid,
	input[7:0]       block_read_data,
	input            block_read_req_ack,
	output reg       block_write_req,
	output[7:0]      block_write_data,
	input            block_write_data_rd,
	input            block_write_req_ack
);
reg[7:0] read_data;
reg[31:0] timer;

localparam S_IDLE               = 0;
localparam S_PWR_WAIT           = 19;   // 上电稳定延时（复位释放后先等 SD 卡供电稳定再发 CMD0）
localparam S_CMD0               = 1;
localparam S_CMD8               = 2;
localparam S_CMD55              = 3;
localparam S_CMD41              = 4;
localparam S_CMD17              = 5;
localparam S_READ               = 6;
localparam S_CMD24              = 7;
localparam S_WRITE              = 8;
localparam S_ERR                = 14;
localparam S_WRITE_END          = 15;
localparam S_READ_END           = 16;
localparam S_WAIT_READ_WRITE    = 17;
localparam S_CMD16              = 18;
localparam S_PROBE              = 20;   // 空闲探活（CMD13 SEND_STATUS）

reg[4:0]                       state;
reg[31:0]                      sec_addr;
// 空闲计时：S_WAIT_READ_WRITE 里累计，用于周期性探活
reg[31:0]                      idle_cnt;
// 读错误完成脉冲：读命令/读数据失败时打一拍，让 sd_sector_adapter 解除 busy
reg                            sec_read_err_pulse;
assign sd_sec_read_data_valid = (state == S_READ) && block_read_valid;
assign sd_sec_read_data = block_read_data;
assign sd_sec_read_end = (state == S_READ_END) | sec_read_err_pulse;
assign sd_sec_write_data_req = (state == S_WRITE) && block_write_data_rd;
assign block_write_data = sd_sec_write_data;
assign sd_sec_write_end = (state == S_WRITE_END);

always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		cmd_req <= 1'b0;
		cmd_data_len <= 16'd0;
		cmd_r1 <= 8'd0;
		cmd <= 48'd0;
		spi_clk_div <= SPI_LOW_SPEED_DIV[15:0];
		block_write_req <= 1'b0;
		block_read_req <= 1'b0;
		sec_addr <= 32'd0;
		sd_init_done <= 1'b0;
		timer <= 32'd0;
		sec_read_err_pulse <= 1'b0;
		idle_cnt <= 32'd0;
	end
	else
		case(state)
			S_IDLE:
			begin
				// 第6讲：复位释放后先进入上电稳定延时，避免 PLL 刚锁定/SD 卡
				// 供电未稳定时就发 CMD0 导致首次初始化失败（表现为上电 SD:NO CARD，
				// 按复位键后才恢复）。延时约 50ms @100MHz。
				state <= S_PWR_WAIT;
				timer <= 32'd0;
				sd_init_done <= 1'b0;
				sec_read_err_pulse <= 1'b0;
				spi_clk_div <= SPI_LOW_SPEED_DIV[15:0];
				idle_cnt <= 32'd0;
			end
			S_PWR_WAIT:
			begin
				if (timer >= PWR_WAIT_CYCLES)
				begin
					state <= S_CMD0;
					timer <= 32'd0;
				end
				else
					timer <= timer + 32'd1;
				sd_init_done <= 1'b0;
			end
			S_CMD0:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD8;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h01;
					cmd <= {8'd0,8'h00,8'h00,8'h00,8'h00,8'h95};
				end
			end
			S_CMD8:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD55;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd4;
					cmd_r1 <= 8'h01;
					cmd <= {8'd8,8'h00,8'h00,8'h01,8'haa,8'h87};
				end
			end
			S_CMD55:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD41;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h01;
					cmd <= {8'd55,8'h00,8'h00,8'h00,8'h00,8'hff};
				end
			end
			S_CMD41:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD16;
					cmd_req <= 1'b0;
					sd_init_done <= 1'b1;
					spi_clk_div <= SPI_HIGH_SPEED_DIV[15:0];
				end
				else if(cmd_req_ack)
				begin
					state <= S_CMD55;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd41,8'h40,8'h00,8'h00,8'h00,8'hff};
				end
			end
			S_CMD16:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_WAIT_READ_WRITE;
					cmd_req <= 1'b0;
					sd_init_done <= 1'b1;
					spi_clk_div <= SPI_HIGH_SPEED_DIV[15:0];
				end
				else if(cmd_req_ack)
				begin
					state <= S_CMD55;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd16,32'd512,8'hff};
				end
			end			
			S_WAIT_READ_WRITE:
			begin
				if(sd_sec_write ==  1'b1)
				begin
					state <= S_CMD24;
					sec_addr <= sd_sec_write_addr;
					idle_cnt <= 32'd0;
				end
				else if(sd_sec_read == 1'b1)
				begin
					state <= S_CMD17;
					sec_addr <= sd_sec_read_addr;
					idle_cnt <= 32'd0;
				end
				else if((PROBE_EN != 0) && (idle_cnt >= PROBE_GAP_CYCLES[31:0]))
				begin
					// 空闲够久：发一次 CMD13 探活（不打扰数据流，只在完全空闲时进行）
					state    <= S_PROBE;
					idle_cnt <= 32'd0;
				end
				else
				begin
					idle_cnt <= idle_cnt + 32'd1;
				end

				spi_clk_div <= 16'd0;
			end
			S_PROBE:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					// 卡有正常应答（R1=0x00）：卡在位，回空闲待命
					state    <= S_WAIT_READ_WRITE;
					cmd_req  <= 1'b0;
					idle_cnt <= 32'd0;
				end
				else if(cmd_req_ack & cmd_req_error)
				begin
					// 卡无应答：被拔掉 / 插回后处于原生 SD 模式 → 整卡重新初始化。
					// 这一步是"拔卡后再插卡读不到卡"的关键修复：不必等上层读失败，
					// 本层自己就会重新走 CMD0..CMD16，插回后自动恢复读卡。
					state        <= S_IDLE;
					cmd_req      <= 1'b0;
					sd_init_done <= 1'b0;
					idle_cnt     <= 32'd0;
				end
				else
				begin
					cmd_req      <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1       <= 8'h00;                       // CMD13 正常 R1=0x00
					cmd          <= {8'd13,8'h00,8'h00,8'h00,8'h00,8'hff};
				end
			end
			S_CMD24:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_WRITE;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd24,sec_addr,8'hff};

				end
			end
			S_WRITE:
			begin
				if(block_write_req_ack == 1'b1)
				begin
					block_write_req <= 1'b0;
					state <= S_WRITE_END;
				end
				else
					block_write_req <= 1'b1;
			end
			S_CMD17:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_READ;
					cmd_req <= 1'b0;
				end
				else if(cmd_req_ack & cmd_req_error)
				begin
					// 读命令失败（卡拔出/接触不良）：回 S_IDLE 重新初始化 SD 卡。
					// 同时打一拍 sec_read_err_pulse（并入 sd_sec_read_end）让
					// sd_sector_adapter 解除 busy，否则 adapter 卡死、重扫读不到卡。
					state <= S_IDLE;
					cmd_req <= 1'b0;
					sd_init_done <= 1'b0;
					sec_read_err_pulse <= 1'b1;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd17,sec_addr,8'hff};
				end
			end
			S_READ:
			begin
				if(block_read_req_ack)
				begin
					state <= S_READ_END;
					block_read_req <= 1'b0;
				end
				else if(cmd_req_error)
				begin
					// 读数据阶段出错（等 0xFE 令牌超时/卡拔除）：回 S_IDLE 重新初始化。
					// 打一拍 sec_read_err_pulse（并入 sd_sec_read_end）让 adapter 解除 busy。
					state <= S_IDLE;
					block_read_req <= 1'b0;
					sd_init_done <= 1'b0;
					sec_read_err_pulse <= 1'b1;
				end
				else
				begin
					block_read_req <= 1'b1;
				end
			end
			S_WRITE_END:
			begin
				state <= S_WAIT_READ_WRITE;
			end
			S_READ_END:
			begin
				state <= S_WAIT_READ_WRITE;
			end
			default:
				state <= S_IDLE;
		endcase
end
endmodule