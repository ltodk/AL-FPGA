`timescale 1ns/1ps
// ============================================================================
// sd_sector_adapter.v —— SD 卡扇区读接口适配层（第七讲 新增）
//
// 职责：把完赛工程 FAT32 模块的 sector_* 接口，转换成官方 sd_card_top 的
//       sd_sec_* 接口。解决三个官方接口缺失的问题：
//         1. sector_busy：官方没有 busy 信号，这里自己生成（请求→完成之间为 1）
//         2. sector_byte_index：官方不输出字节索引，这里自己计数（valid 时 +1）
//         3. sector_error：官方没有错误码，这里改为"读超时"标志（见下）
//
// 握手协议：
//   - sector_req 为单拍脉冲（FAT32 模块在 REQ 状态拉高一拍）；
//   - 本层锁存 sector_lba，拉高 sd_sec_read（电平）直到 sd_sec_read_end；
//   - 官方 sd_card_top 的 sd_sec_read 是电平触发（拉高读一个扇区，读完
//     sd_sec_read_end 单拍），每次只读一个扇区后撤销，适配 FAT32 逐扇区读。
//
// 【拔插卡死锁修复】新增两件事：
//   1. 读超时看门狗 TIMEOUT_CYCLES：请求发出后若底层一直不返回 end（SD 卡被
//      拔掉、SPI 卡死、end 脉冲被别处吃掉），到点强制结束本次读并置
//      sector_error != 0，保证 FAT32 客户端（scanner/streamer）永远不会吊死。
//   2. abort 输入：上层发现卡被拔掉 / 需要整栈软复位时，一拍清掉在途请求，
//      避免"上一个客户端遗留的请求"被下一个客户端误当成自己的数据。
//
// 复位：低有效（rst_n），与 FAT32 两个模块一致，实例化时用 ~rst 连接。
// ============================================================================
module sd_sector_adapter #(
    parameter integer TIMEOUT_CYCLES = 10_000_000    // 读超时（默认 100ms @100MHz）
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        abort,                // 软复位：一拍清掉在途请求
    // === 新模块侧（sector 接口，FAT32 模块用）===
    input  wire        sector_req,           // 读扇区请求（单拍脉冲）
    input  wire [31:0] sector_lba,           // 扇区号
    output wire        sector_busy,          // 忙（正在读）
    output wire        sector_valid,         // 字节有效脉冲
    output wire [7:0]  sector_byte,          // 当前字节
    output wire [8:0]  sector_byte_index,    // 字节索引（0~511）
    output reg         sector_done,          // 读完成脉冲
    output reg  [7:0]  sector_error,         // 错误：与 sector_done 同拍，非0=本次读超时
    // === 官方 sd_card_top 侧（sd_sec 接口）===
    output reg         sd_sec_read,          // 读扇区请求（电平）
    output reg  [31:0] sd_sec_read_addr,     // 扇区号
    input  wire        sd_sec_read_data_valid,// 字节有效
    input  wire [7:0]  sd_sec_read_data,     // 当前字节
    input  wire        sd_sec_read_end       // 读完成
);

    reg [8:0]  byte_cnt;    // 字节索引计数器（0~511）
    reg        busy_reg;    // 忙寄存器
    reg [31:0] wd_cnt;      // 读超时计数器

    assign sector_busy       = busy_reg;
    assign sector_valid      = sd_sec_read_data_valid;
    assign sector_byte       = sd_sec_read_data;
    assign sector_byte_index = byte_cnt;

    // 超时判定（组合）
    wire wd_expire = busy_reg && (wd_cnt >= TIMEOUT_CYCLES);

    // 读超时计数器：busy 期间累加，空闲 / abort 清零
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wd_cnt <= 32'd0;
        else if (abort || !busy_reg)
            wd_cnt <= 32'd0;
        else
            wd_cnt <= wd_cnt + 32'd1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sd_sec_read      <= 1'b0;
            sd_sec_read_addr <= 32'd0;
            byte_cnt         <= 9'd0;
            busy_reg         <= 1'b0;
            sector_done      <= 1'b0;
            sector_error     <= 8'd0;
        end else begin
            sector_done  <= 1'b0;
            sector_error <= 8'd0;

            if (abort) begin
                // 上层要求整栈软复位：撤销在途请求，不再产生 done（客户端同时被复位）
                sd_sec_read <= 1'b0;
                busy_reg    <= 1'b0;
                byte_cnt    <= 9'd0;
            end
            // 请求到来且不忙时：锁存扇区号，发出读请求
            else if (sector_req && !busy_reg) begin
                sd_sec_read      <= 1'b1;
                sd_sec_read_addr <= sector_lba;
                busy_reg         <= 1'b1;
                byte_cnt         <= 9'd0;
            end
            // 读完 / 读超时：撤销请求，产生完成脉冲（超时时带错误码）
            else if (busy_reg && (sd_sec_read_end || wd_expire)) begin
                sd_sec_read  <= 1'b0;
                busy_reg     <= 1'b0;
                byte_cnt     <= 9'd0;
                sector_done  <= 1'b1;
                sector_error <= wd_expire ? 8'h10 : 8'h00;
            end
            // 读进行中：字节有效时计数器递增
            else if (sd_sec_read_data_valid) begin
                byte_cnt <= byte_cnt + 9'd1;
            end
        end
    end

endmodule
