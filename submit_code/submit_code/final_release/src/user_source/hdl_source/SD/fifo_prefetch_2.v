`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// fifo_prefetch_2 —— 两级预取槽（第8讲 阶段B）
// 作用：把异步 FIFO 的输出提前 1 拍顶出来，替代原 video_delay 的"提前 1 拍"机制。
//
// 为什么需要它：
//   原读侧链路里 video_delay 用 21 拍移位，让 read_en 提前 2 拍从 FIFO 取数、
//   数据相对 de 提前 1 拍就绪。阶段 B 删掉 video_delay 后，这个"提前量"
//   改由本模块提供：内部两个槽位 slot0/slot1 缓存，out_valid/out_data 在
//   FIFO 数据真正弹出前 1 拍就有效，等价于原来的 1 拍提前。
//
// 端口语义：
//   raw_empty/raw_valid/raw_data  —— 上游异步 FIFO 的读口
//   raw_read_en                    —— 本模块向上游要数的使能（含 in-flight 追踪）
//   consume                        —— 下游真的要取走一个像素
//   out_valid/out_empty/out_data   —— 下游可见的像素流
//
// 关键：outstanding 追踪"已发出 raw_read_en 但数据还没回到 raw_valid"的在途次数，
//       避免提前把 FIFO 读空（underflow 的根源之一）。
// -----------------------------------------------------------------------------
module fifo_prefetch_2 #(
    parameter DATA_WIDTH = 32
)(
    input  wire                  clk,
    input  wire                  rst,
    input  wire                  raw_empty,
    input  wire                  raw_valid,
    input  wire [DATA_WIDTH-1:0] raw_data,
    output wire                  raw_read_en,
    input  wire                  consume,
    output wire                  out_valid,
    output wire                  out_empty,
    output wire [DATA_WIDTH-1:0] out_data
);

    reg [DATA_WIDTH-1:0] slot0;
    reg [DATA_WIDTH-1:0] slot1;
    reg [1:0]            count;
    reg [1:0]            outstanding;

    wire pop = consume && (count != 0);
    wire [2:0] reserved = {1'b0, count} + {1'b0, outstanding};

    assign raw_read_en = !raw_empty &&
                         ((reserved < 3'd2) || (pop && (reserved == 3'd2)));
    assign out_valid = (count != 0);
    assign out_empty = (count == 0);
    assign out_data  = slot0;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            slot0       <= {DATA_WIDTH{1'b0}};
            slot1       <= {DATA_WIDTH{1'b0}};
            count       <= 2'd0;
            outstanding <= 2'd0;
        end else begin
            case ({raw_read_en, raw_valid})
                2'b10: outstanding <= outstanding + 2'd1;
                2'b01: outstanding <= outstanding - 2'd1;
                default: outstanding <= outstanding;
            endcase

            case ({pop, raw_valid})
                2'b00: count <= count;
                2'b01: begin
                    if (count == 2'd0)
                        slot0 <= raw_data;
                    else
                        slot1 <= raw_data;
                    count <= count + 2'd1;
                end
                2'b10: begin
                    if (count == 2'd2)
                        slot0 <= slot1;
                    count <= count - 2'd1;
                end
                2'b11: begin
                    if (count == 2'd1)
                        slot0 <= raw_data;
                    else begin
                        slot0 <= slot1;
                        slot1 <= raw_data;
                    end
                    count <= count;
                end
            endcase
        end
    end

endmodule
