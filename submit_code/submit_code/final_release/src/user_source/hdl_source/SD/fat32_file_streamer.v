`timescale 1ns/1ps
// ============================================================================
// fat32_file_streamer.v —— FAT32 文件流读取器（第七讲 新增）
//
// 职责：根据起始簇号 + 文件大小，通过 FAT 链遍历，读取文件的所有簇，
//       输出文件字节流（file_valid + file_byte + file_byte_offset）。
//
// 状态机（7 态）：
//   IDLE → DATA_REQ → DATA_WAIT →（簇内继续 / 查 FAT 表）
//   → FAT_REQ → FAT_WAIT → FAT_PARSE →（下一簇 / EOF）→ FINISH → IDLE
//
// 关键设计：
//   - 读数据簇：cluster_lba = data_start_lba + (cluster-2) * sectors_per_cluster
//   - 查 FAT 表：fat_sector = fat_start_lba + (cluster >> 7)
//                 fat_offset = (cluster & 0x7F) << 2（每扇区 128 条 4 字节条目）
//   - FAT 条目 28 位（高 4 位保留），0xFFFFFF8~F = EOF
//   - 错误处理：FAT 环 / 簇越界 / 提前 EOF / 遍历超限，防止死循环
//
// 【拔插卡死锁修复】新增两条"绝不死等"保障：
//   1. abort 输入：上层（sd_card_bmp）在拔卡 / 重扫 / 超时兜底时拉高一拍，
//      本模块立刻回 IDLE。原实现一旦卡在 DATA_WAIT 等 sector_done（该完成
//      脉冲可能被别的客户端吃掉、或 SD 底层卡死），就永远停在非 IDLE 态，
//      之后 file_start 全部被忽略 —— 表现为"插回卡后每张图都加载超时、
//      黑屏 + ERR:0x38"，只能复位。abort 彻底消除这个死态。
//   2. 扇区读看门狗 SECTOR_TIMEOUT_CYCLES：非 IDLE 且长时间没有任何扇区
//      进度（无 sector_valid / sector_done），按 ERR_SECTOR 收场，
//      让上层走标准重扫恢复通路。
//
// 复位：低有效（rst_n），实例化时用 ~rst 连接。
// ============================================================================
module fat32_file_streamer #(
    parameter integer MAX_VISITED_CLUSTERS  = 8192,
    parameter integer SECTOR_TIMEOUT_CYCLES = 20_000_000   // 200ms @100MHz
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        abort,               // 软复位：一拍回 IDLE（拔卡/重扫兜底）
    input  wire        file_start,
    // ★ 第 11 讲：扇区级流控（默认接 0，图片加载路径行为与原来逐位一致）
    //   拉高时只在「发下一个数据扇区之前」停下，不会打断正在传输的扇区
    //   （扇区内的字节是 SD 侧连续吐出来的，打断就等于丢数据）。
    //   音频用它控制写入速率：缓冲不够就停止发请求，等播放侧腾出空间。
    //   注意 ST_FAT_REQ 不受它影响 —— FAT 表项必须读出来才能知道下一簇在哪，
    //   卡在这里只会让状态机悬空，反而更糟。
    input  wire        file_pause,
    input  wire [31:0] first_cluster,
    input  wire [31:0] file_size,
    input  wire [7:0]  sectors_per_cluster,
    input  wire [2:0]  sectors_per_cluster_shift,   // log2(每簇扇区数)，用移位代替 32 位乘法
    input  wire [31:0] fat_start_lba,
    input  wire [31:0] data_start_lba,
    input  wire [31:0] max_cluster,
    output wire        file_busy,
    output reg         file_done,
    output reg         file_valid,
    output reg  [7:0]  file_byte,
    output reg  [31:0] file_byte_offset,
    output reg  [7:0]  fs_error,
    // 扇区读接口（对接 sd_sector_adapter）
    output reg         sector_req,
    output reg  [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);
    localparam ERR_NONE              = 8'h00;
    localparam ERR_SECTOR            = 8'h10;
    localparam ERR_FAT_LOOP          = 8'h23;
    localparam ERR_CLUSTER_RANGE     = 8'h24;
    localparam ERR_EARLY_EOF         = 8'h25;
    localparam ERR_VISITED_EXHAUSTED = 8'h26;

    localparam ST_IDLE      = 4'd0;
    localparam ST_DATA_REQ  = 4'd1;
    localparam ST_DATA_WAIT = 4'd2;
    localparam ST_FAT_REQ   = 4'd3;
    localparam ST_FAT_WAIT  = 4'd4;
    localparam ST_FAT_PARSE = 4'd5;
    localparam ST_FINISH    = 4'd7;

    localparam integer CHAIN_COUNT_WIDTH = $clog2(MAX_VISITED_CLUSTERS + 1);

    reg [3:0] state;
    reg [31:0] current_cluster;
    reg [31:0] remaining_bytes;
    reg [7:0]  sector_in_cluster;
    reg [8:0]  fat_entry_offset;
    reg [7:0]  fat_byte0, fat_byte1, fat_byte2, fat_byte3;
    reg [CHAIN_COUNT_WIDTH-1:0] chain_count;
    reg        final_chain_check;
    reg [31:0] wd_cnt;                      // 扇区读看门狗

    // ---------- 2026-10-21 时序优化：簇→扇区地址计算拆成两拍 ----------
    // 原写法在一拍内完成「(cluster-2) 减法 + 比较链 → 左移 → 加 data_start_lba」，
    // 综合出 8 级逻辑（ADDER×4 + LUT4×2 + LUT5 + LUT3）、9.712ns，
    // 是 100MHz 下 sd_card_clk 组的 setup 违例（-0.064ns）。
    // 拆法：新簇到来时先把 (cluster-2)<<spc_shift 算进 cluster_base（6 级，单独一拍）；
    //       真正发请求前在 ST_DATA_REQ 里再补一次加法（4 级）。
    //       ST_DATA_REQ 本来就要等 !sector_busy，多用的这一拍不增加任何总线开销。
    // lba_reload：标记「本簇的第一个扇区，需要由 cluster_base 重算而非 +1」。
    reg [31:0] cluster_base;
    reg        lba_reload;

    // FAT 条目 28 位组合（供多处复用）
    wire [27:0] fat_entry;
    assign fat_entry = {fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0};

    assign file_busy = (state != ST_IDLE);

    // 扇区读看门狗：任何扇区进度都会清零
    // 2026-10-21：超时判定改为寄存输出。
    //   原写法是 32 位比较（2 级 ADDER）+ 状态判断，组合直连状态转移最外层 if，
    //   再串进 fat_byte0~3 的写使能，综合出 7 级逻辑 9.824ns
    //   （起点 wd_cnt_reg[11] → 终点 fat_byte0_reg[4].ce），
    //   100MHz 下 setup 违例 -0.104ns，是 sd_card_clk 组最后两条违例之一。
    //   寄存后 wd_cnt 到比较器为止（单拍），超时信号再驱动状态机（另一拍）。
    //   代价：超时晚 1 拍（10ns），对 200ms 量级的看门狗可忽略。
    //   附带改进：增加 !sector_valid/!sector_done 保护，有进度的这一拍不判超时，
    //   避免数据恰好在超时边界到达时被误判（原组合写法同样存在此隐患）。
    reg wd_expire;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wd_expire <= 1'b0;
        else
            wd_expire <= (state != ST_IDLE) && (state != ST_FINISH) &&
                         (wd_cnt >= SECTOR_TIMEOUT_CYCLES) &&
                         !sector_valid && !sector_done && !file_pause;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wd_cnt <= 32'd0;
        else if (abort || (state == ST_IDLE) || (state == ST_FINISH) ||
                 sector_done || sector_valid)
            wd_cnt <= 32'd0;
        else
            wd_cnt <= wd_cnt + 32'd1;
    end

    task finish_with_error;
        input [7:0] code;
        begin
            fs_error <= code;
            state <= ST_FINISH;
        end
    endtask

    task advance_to_next_cluster;
        input [31:0] cluster_value;
        begin
            chain_count <= chain_count + 1'b1;
            current_cluster <= cluster_value;
            // 簇号 → 扇区号：每簇扇区数在 FAT32 中必为 2 的幂（scanner 已用
            // valid_spc() 校验），因此用移位代替乘法。
            // 只算到「簇内偏移」为止，加 data_start_lba 放到 ST_DATA_REQ 里做，
            // 避免"减法+移位+加法"串成一拍（详见 cluster_base 声明处的说明）。
            cluster_base <= (cluster_value - 32'd2) << sectors_per_cluster_shift;
            lba_reload   <= 1'b1;
            state <= ST_DATA_REQ;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            file_done <= 1'b0;
            file_valid <= 1'b0;
            file_byte <= 8'd0;
            file_byte_offset <= 32'd0;
            fs_error <= ERR_NONE;
            sector_req <= 1'b0;
            sector_lba <= 32'd0;
            current_cluster <= 32'd0;
            remaining_bytes <= 32'd0;
            sector_in_cluster <= 8'd0;
            fat_entry_offset <= 9'd0;
            fat_byte0 <= 8'd0; fat_byte1 <= 8'd0;
            fat_byte2 <= 8'd0; fat_byte3 <= 8'd0;
            chain_count <= 0;
            final_chain_check <= 1'b0;
            cluster_base <= 32'd0;
            lba_reload   <= 1'b0;
        end else begin
            file_done <= 1'b0;
            file_valid <= 1'b0;
            sector_req <= 1'b0;

            if (abort) begin
                // 上层整栈软复位（拔卡 / 重扫）：立刻回 IDLE，不再等任何完成脉冲
                state             <= ST_IDLE;
                fs_error          <= ERR_NONE;
                file_byte_offset  <= 32'd0;
                remaining_bytes   <= 32'd0;
                current_cluster   <= 32'd0;
                sector_in_cluster <= 8'd0;
                chain_count       <= 0;
                final_chain_check <= 1'b0;
            end else if (wd_expire) begin
                // 看门狗：长时间没有扇区进度 → 按扇区错误收场，交给上层重扫
                fs_error <= ERR_SECTOR;
                state    <= ST_FINISH;
            end else begin

            // 数据等待态：输出文件字节流
            if (state == ST_DATA_WAIT && sector_valid && remaining_bytes != 0) begin
                file_valid <= 1'b1;
                file_byte <= sector_byte;
                file_byte_offset <= file_size - remaining_bytes;
                remaining_bytes <= remaining_bytes - 1'b1;
            end

            // FAT 等待态：采样 FAT 条目 4 字节
            if (state == ST_FAT_WAIT && sector_valid) begin
                if (sector_byte_index == fat_entry_offset)
                    fat_byte0 <= sector_byte;
                else if (sector_byte_index == fat_entry_offset + 1'b1)
                    fat_byte1 <= sector_byte;
                else if (sector_byte_index == fat_entry_offset + 2'd2)
                    fat_byte2 <= sector_byte;
                else if (sector_byte_index == fat_entry_offset + 2'd3)
                    fat_byte3 <= sector_byte;
            end

            case (state)
                ST_IDLE: if (file_start) begin
                    fs_error <= ERR_NONE;
                    file_byte_offset <= 32'd0;
                    remaining_bytes <= file_size;
                    sector_in_cluster <= 8'd0;
                    chain_count <= 1;
                    final_chain_check <= 1'b0;
                    if (file_size == 0) begin
                        state <= ST_FINISH;
                    end else if (first_cluster < 2 || first_cluster > max_cluster ||
                                 sectors_per_cluster == 0) begin
                        finish_with_error(ERR_CLUSTER_RANGE);
                    end else begin
                        current_cluster <= first_cluster;
                        // 同上：只算到簇内偏移，加法留到 ST_DATA_REQ（移位代替乘法）
                        cluster_base <= (first_cluster - 32'd2) << sectors_per_cluster_shift;
                        lba_reload   <= 1'b1;
                        state <= ST_DATA_REQ;
                    end
                end

                ST_DATA_REQ: if (!sector_busy && !file_pause) begin
                    // 本簇第一个扇区：用预算好的簇内偏移补一次加法（4 级）
                    if (lba_reload) begin
                        sector_lba <= data_start_lba + cluster_base;
                        lba_reload <= 1'b0;
                    end
                    sector_req <= 1'b1;
                    state <= ST_DATA_WAIT;
                end

                ST_DATA_WAIT: if (sector_done) begin
                    if (sector_error != 0) begin
                        finish_with_error(ERR_SECTOR);
                    end else if (remaining_bytes == 0) begin
                        // 文件读完：最后一次 FAT 校验，期望 EOF
                        final_chain_check <= 1'b1;
                        sector_in_cluster <= 8'd0;
                        fat_entry_offset <= {current_cluster[6:0], 2'b00};
                        sector_lba <= fat_start_lba + (current_cluster >> 7);
                        state <= ST_FAT_REQ;
                    end else if (sector_in_cluster + 1'b1 < sectors_per_cluster) begin
                        // 簇内还有扇区，继续读
                        sector_in_cluster <= sector_in_cluster + 1'b1;
                        sector_lba <= sector_lba + 1'b1;
                        state <= ST_DATA_REQ;
                    end else begin
                        // 当前簇读完，查 FAT 表找下一簇
                        sector_in_cluster <= 8'd0;
                        fat_entry_offset <= {current_cluster[6:0], 2'b00};
                        sector_lba <= fat_start_lba + (current_cluster >> 7);
                        state <= ST_FAT_REQ;
                    end
                end

                ST_FAT_REQ: if (!sector_busy) begin
                    sector_req <= 1'b1;
                    state <= ST_FAT_WAIT;
                end

                ST_FAT_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    else
                        state <= ST_FAT_PARSE;
                end

                ST_FAT_PARSE: begin
                    if (final_chain_check) begin
                        final_chain_check <= 1'b0;
                        // 文件读完后的最后一次 FAT 校验：必须遇到 EOF
                        if (fat_entry >= 28'hffffff8)
                            state <= ST_FINISH;
                        else if ({4'd0, fat_entry} < 32'd2 || {4'd0, fat_entry} > max_cluster)
                            finish_with_error(ERR_CLUSTER_RANGE);
                        else
                            finish_with_error(ERR_FAT_LOOP);
                    end else if (fat_entry >= 28'hffffff8) begin
                        // 数据还没读完就遇到 EOF：提前截断
                        finish_with_error(ERR_EARLY_EOF);
                    end else if ({4'd0, fat_entry} < 32'd2 || {4'd0, fat_entry} > max_cluster) begin
                        finish_with_error(ERR_CLUSTER_RANGE);
                    end else if (chain_count >= MAX_VISITED_CLUSTERS) begin
                        finish_with_error(ERR_VISITED_EXHAUSTED);
                    end else if ({4'd0, fat_entry} == current_cluster) begin
                        finish_with_error(ERR_FAT_LOOP);
                    end else begin
                        advance_to_next_cluster({4'd0, fat_entry});
                    end
                end

                ST_FINISH: begin
                    file_done <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase

            end
        end
    end
endmodule
