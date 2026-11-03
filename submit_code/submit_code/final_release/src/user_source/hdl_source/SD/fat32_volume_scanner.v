`timescale 1ns/1ps
// ============================================================================
// fat32_volume_scanner.v —— FAT32 卷扫描器（第七讲 新增）
//
// 职责：识别 FAT32 文件系统，解析 BPB，扫描根目录，按文件名匹配
//       PIC1.BMP ~ PIC4.BMP，输出 4 个文件的起始簇号和文件大小。
//       第 11 讲追加：顺带识别 MUSIC.PCM（音频），输出起始簇号和大小。
//
// 状态机（12 态）：
//   IDLE → READ0_REQ → READ0_WAIT →（判断 MBR / 直接引导）
//   → BOOT_REQ → BOOT_WAIT → BOOT_PARSE → BOOT_LAYOUT → BOOT_ROOT
//   → ROOT_REQ → ROOT_WAIT → DIR_SCAN → FINISH
//
// 关键设计：
//   - 读 LBA0 判断：偏移0=0xEB/0xE9 → 直接引导扇区(partition_lba=0)；
//     偏移450=0x0B/0x0C → MBR，从偏移454读分区起始 LBA。
//   - 解析 BPB：校验 512字节/扇区、每簇扇区数为2的幂、保留扇区非零、
//     FAT份数1或2、根目录簇号>=2，输出 sectors_per_cluster/fat_start_lba/
//     data_start_lba/max_cluster。
//   - 扫根目录：读满根目录**第一个簇**的所有扇区（簇内扇区数由 BPB 的
//     sectors_per_cluster 决定，典型 8/16 → 128/256 个目录项），逐个匹配
//     8.3 短文件名 "PIC1    BMP" ~ "PIC4    BMP"（8字节文件名 + 3字节扩展名），
//     以及第 11 讲新增的 "MUSIC   PCM"。
//     遇到目录项首字节 0x00（本项及后续全为空）立即提前收尾，不必读完整个簇。
//
// 局限（第 11 讲阶段 C 修改后）：
//   - 覆盖**整个根目录第一簇**，但**不跟随 FAT 链**读第二簇及以后。
//     FAT32 的根目录是普通簇链，理论上可以跨簇；实际上根目录几乎不会占满
//     一整个簇（典型簇 4KB~32KB = 128~1024 个目录项），因此单簇足够。
//   - 不支持长文件名(LFN)/子目录。LFN 目录项（属性 0x0F）会被当作普通项跳过。
//
// ★ 2026-10-25（第 11 讲阶段 C）改动记录：
//   原实现只读根目录**第一个扇区**（16 个目录项）。一旦卡上文件变多，
//   MUSIC.PCM 掉出前 16 项就再也找不到，表现为"昨天能放、今天没声音"。
//   改为读满第一个簇，并在 0x00 结束标记处提前退出。
//   注意：resource_valid / audio_valid 的**清零必须放在 ST_BOOT_ROOT**，
//   不能留在 ST_ROOT_REQ —— 后者现在是每读一个扇区就进一次，
//   若在那里清零会把前面扇区已经找到的结果全部抹掉。
//
// 复位：低有效（rst_n），实例化时用 ~rst 连接（本工程子模块高有效复位）。
// ============================================================================
module fat32_volume_scanner #(
    parameter integer SECTOR_TIMEOUT_CYCLES = 20_000_000    // 扫描看门狗（默认 200ms @100MHz）
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        abort,                // 软复位：一拍回 IDLE（拔卡/重扫兜底）
    input  wire        scan_start,
    output wire        scan_busy,
    output reg         scan_done,
    output reg  [7:0]  fs_error,
    output reg  [31:0] partition_lba,
    output reg  [7:0]  sectors_per_cluster,
    output reg  [2:0]  sectors_per_cluster_shift,   // log2(每簇扇区数)，供 streamer 用移位代替乘法
    output reg  [31:0] fat_start_lba,
    output reg  [31:0] data_start_lba,
    output reg  [31:0] max_cluster,
    output reg  [3:0]  resource_valid,
    output reg  [31:0] resource0_cluster,
    output reg  [31:0] resource1_cluster,
    output reg  [31:0] resource2_cluster,
    output reg  [31:0] resource3_cluster,
    output reg  [31:0] resource0_size,
    output reg  [31:0] resource1_size,
    output reg  [31:0] resource2_size,
    output reg  [31:0] resource3_size,
    // ★ 第 11 讲阶段 B：根目录下的音频文件 MUSIC.PCM（8.3 短名）
    //   与 4 张图**互不影响**：找到了就置 audio_valid，没找到也只是 audio_valid=0，
    //   不参与下面 resource_valid==4'hf 的完整性判定，所以"卡里没有音频文件"
    //   不会让整个扫描失败（图片功能照旧）。
    //   扫描范围同图片：根目录第一个簇的**全部扇区**（第 11 讲阶段 C 已从单扇区扩展）。
    output reg         audio_valid,
    output reg  [31:0] audio_cluster,
    output reg  [31:0] audio_size,
    // 扇区读接口（对接 sd_sector_adapter，不是直接对接 sd_card_top）
    output reg         sector_req,
    output reg  [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);

    localparam ERR_NONE             = 8'h00;
    localparam ERR_SECTOR           = 8'h10;
    localparam ERR_BOOT_SIGNATURE   = 8'h20;
    localparam ERR_UNSUPPORTED_BPB  = 8'h21;
    localparam ERR_RESOURCE_MISSING = 8'h22;

    localparam ST_IDLE        = 4'd0;
    localparam ST_READ0_REQ   = 4'd1;
    localparam ST_READ0_WAIT  = 4'd2;
    localparam ST_BOOT_PARSE  = 4'd3;
    localparam ST_BOOT_REQ    = 4'd4;
    localparam ST_BOOT_WAIT   = 4'd5;
    localparam ST_ROOT_REQ    = 4'd6;
    localparam ST_ROOT_WAIT   = 4'd7;
    localparam ST_DIR_SCAN    = 4'd8;
    localparam ST_FINISH      = 4'd9;
    localparam ST_BOOT_LAYOUT = 4'd10;
    localparam ST_BOOT_ROOT   = 4'd11;

    reg [3:0] state;
    reg [7:0] sector_buffer [0:511];   // 整扇区缓存（512 字节寄存器）
    reg [31:0] root_cluster;
    reg [31:0] total_sectors;
    reg [31:0] fat_sectors;
    reg [7:0]  fat_count;
    reg [15:0] reserved_sectors;
    reg [31:0] root_lba;
    reg [31:0] data_sector_count;
    reg [31:0] fat_span;
    reg [7:0]  dir_name0, dir_name1, dir_name2, dir_name3;
    reg [7:0]  dir_name4, dir_name5, dir_name6, dir_name7;
    reg [7:0]  dir_name8, dir_name9, dir_name10;
    reg        dir_entry_match;
    reg        dir_audio_match;          // 目录项是 MUSIC.PCM（第 11 讲）
    reg [1:0]  dir_resource_index;
    reg [15:0] dir_cluster_high;
    reg [15:0] dir_cluster_low;
    reg [31:0] dir_file_size;
    // ★ 第 11 讲阶段 C：根目录多扇区扫描
    reg [7:0]  root_sec_cnt;             // 当前簇内已扫描/正在扫描的扇区索引
    reg        dir_end_seen;             // 见到目录项首字节 0x00 → 后面全空，可收尾

    assign scan_busy = (state != ST_IDLE);

    // ===== 拔插卡死锁修复：扫描看门狗 =====
    // 非 IDLE 且长时间没有任何扇区进度（sector_valid / sector_done），说明底层
    // 读已经吊死（拔卡、SPI 卡死、完成脉冲被别的客户端吃掉）。到点直接按
    // ERR_SECTOR 收场 → scan_done，让 sd_card_bmp 的空扫描重试 / 重扫逻辑接管，
    // 绝不允许 scan_busy 永久为 1。
    reg [31:0] wd_cnt;
    // 2026-10-21：超时判定改为寄存输出（与 fat32_file_streamer 同一处理）。
    //   原写法 wd_expire 是 32 位比较 + 状态判断的组合输出，直接参与状态转移最外层
    //   else-if，再串进 resource0~3_cluster 的写使能，综合出 6 级逻辑 10.312ns，
    //   其中 net 8.259ns 占 81%（fanout 42）——布线延迟占绝对主导，
    //   甚至跨到了 u_media_hdmi_audio_bridge/u_symbol_core 的调度器。
    //   100MHz 下 setup 违例 -0.664ns，是 20:40 那次综合 21 个违例端点的共同起点。
    //   寄存后 wd_cnt 只到比较器为止（单拍），超时信号再驱动状态机（另一拍）。
    //   代价：超时晚 1 拍（10ns），对 200ms 量级的扫描看门狗可忽略。
    //   附带改进：加 !sector_valid/!sector_done 保护，有进度的这一拍不判超时，
    //   避免数据恰好在超时边界到达时被误判。
    reg wd_expire;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wd_expire <= 1'b0;
        else
            wd_expire <= (state != ST_IDLE) && (state != ST_FINISH) &&
                         (wd_cnt >= SECTOR_TIMEOUT_CYCLES) &&
                         !sector_valid && !sector_done;
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

    // 小端 16 位读取
    function [15:0] le16;
        input integer offset;
        begin
            le16 = {sector_buffer[offset+1], sector_buffer[offset]};
        end
    endfunction

    // 小端 32 位读取
    function [31:0] le32;
        input integer offset;
        begin
            le32 = {sector_buffer[offset+3], sector_buffer[offset+2],
                    sector_buffer[offset+1], sector_buffer[offset]};
        end
    endfunction

    // 每簇扇区数是否为 2 的幂
    function valid_spc;
        input [7:0] value;
        begin
            valid_spc = (value != 0) && ((value & (value - 1'b1)) == 0);
        end
    endfunction

    // 每簇扇区数 → 移位量（2^n → n）
    function [2:0] spc_shift;
        input [7:0] value;
        begin
            case (value)
                8'd2:   spc_shift = 3'd1;
                8'd4:   spc_shift = 3'd2;
                8'd8:   spc_shift = 3'd3;
                8'd16:  spc_shift = 3'd4;
                8'd32:  spc_shift = 3'd5;
                8'd64:  spc_shift = 3'd6;
                8'd128: spc_shift = 3'd7;
                default: spc_shift = 3'd0;
            endcase
        end
    endfunction

    task finish_with_error;
        input [7:0] code;
        begin
            fs_error <= code;
            state <= ST_FINISH;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            scan_done <= 1'b0;
            fs_error <= ERR_NONE;
            partition_lba <= 32'd0;
            sectors_per_cluster <= 8'd0;
            fat_start_lba <= 32'd0;
            data_start_lba <= 32'd0;
            max_cluster <= 32'd0;
            resource_valid <= 4'd0;
            resource0_cluster <= 32'd0;
            resource1_cluster <= 32'd0;
            resource2_cluster <= 32'd0;
            resource3_cluster <= 32'd0;
            resource0_size <= 32'd0;
            resource1_size <= 32'd0;
            resource2_size <= 32'd0;
            resource3_size <= 32'd0;
            audio_valid <= 1'b0;
            audio_cluster <= 32'd0;
            audio_size <= 32'd0;
            dir_audio_match <= 1'b0;
            sector_req <= 1'b0;
            sector_lba <= 32'd0;
            root_cluster <= 32'd0;
            total_sectors <= 32'd0;
            fat_sectors <= 32'd0;
            fat_count <= 8'd0;
            reserved_sectors <= 16'd0;
            root_lba <= 32'd0;
            data_sector_count <= 32'd0;
            fat_span <= 32'd0;
            sectors_per_cluster_shift <= 3'd0;
            dir_name0 <= 8'd0; dir_name1 <= 8'd0; dir_name2 <= 8'd0; dir_name3 <= 8'd0;
            dir_name4 <= 8'd0; dir_name5 <= 8'd0; dir_name6 <= 8'd0; dir_name7 <= 8'd0;
            dir_name8 <= 8'd0; dir_name9 <= 8'd0; dir_name10 <= 8'd0;
            dir_entry_match <= 1'b0;
            dir_resource_index <= 2'd3;
            dir_cluster_high <= 16'd0;
            dir_cluster_low <= 16'd0;
            dir_file_size <= 32'd0;
            root_sec_cnt <= 8'd0;
            dir_end_seen <= 1'b0;
        end else begin
            scan_done <= 1'b0;
            sector_req <= 1'b0;

            if (abort) begin
                // 上层整栈软复位（拔卡 / 重扫）：立刻回 IDLE，不再等任何完成脉冲
                state <= ST_IDLE;
            end else if (wd_expire) begin
                // 扫描看门狗超时：按扇区错误收场，交给上层重扫
                fs_error <= ERR_SECTOR;
                state    <= ST_FINISH;
            end else begin

            // 非根目录等待态时，扇区字节流缓存进 sector_buffer（用于 LBA0/引导扇区解析）
            if (sector_valid && state != ST_ROOT_WAIT)
                sector_buffer[sector_byte_index] <= sector_byte;

            // 根目录等待态：流式采样目录项（不缓存整扇区，省寄存器）
            if (sector_valid && state == ST_ROOT_WAIT) begin
                case (sector_byte_index[4:0])
                    5'd0: begin
                        dir_name0 <= sector_byte;
                        dir_entry_match <= 1'b0;
                        dir_audio_match <= 1'b0;
                        // ★ 目录项首字节 0x00 = 本项及后续所有项都空闲 → 根目录到此为止
                        //   （0xE5 是"已删除"，不能停，还得接着扫）
                        if (sector_byte == 8'h00)
                            dir_end_seen <= 1'b1;
                    end
                    5'd1:  dir_name1 <= sector_byte;
                    5'd2:  dir_name2 <= sector_byte;
                    5'd3:  dir_name3 <= sector_byte;
                    5'd4:  dir_name4 <= sector_byte;
                    5'd5:  dir_name5 <= sector_byte;
                    5'd6:  dir_name6 <= sector_byte;
                    5'd7:  dir_name7 <= sector_byte;
                    5'd8:  dir_name8 <= sector_byte;
                    5'd9:  dir_name9 <= sector_byte;
                    5'd10: dir_name10 <= sector_byte;
                    5'd11: begin
                        // 属性字节为 0x20（归档文件）+ 文件名 "PIC"+数字1~4+空格+扩展名 BMP
                        if (sector_byte == 8'h20 &&
                            dir_name0 == "P" && dir_name1 == "I" &&
                            dir_name2 == "C" && dir_name3 >= "1" &&
                            dir_name3 <= "4" && dir_name4 == " " &&
                            dir_name5 == " " && dir_name6 == " " &&
                            dir_name7 == " " && dir_name8 == "B" &&
                            dir_name9 == "M" && dir_name10 == "P") begin
                            dir_entry_match <= 1'b1;
                            dir_resource_index <= dir_name3 - "1";
                        end
                        // ★ 第 11 讲阶段 B：顺带识别 "MUSIC" + 三个空格 + "PCM"
                        //   8.3 短名的 8 字节文件名里 "MUSIC" 只占 5 字节，后面补空格。
                        else if (sector_byte == 8'h20 &&
                            dir_name0 == "M" && dir_name1 == "U" &&
                            dir_name2 == "S" && dir_name3 == "I" &&
                            dir_name4 == "C" && dir_name5 == " " &&
                            dir_name6 == " " && dir_name7 == " " &&
                            dir_name8 == "P" && dir_name9 == "C" &&
                            dir_name10 == "M") begin
                            dir_audio_match <= 1'b1;
                        end
                    end
                    5'd20: dir_cluster_high[7:0]  <= sector_byte;
                    5'd21: dir_cluster_high[15:8] <= sector_byte;
                    5'd26: dir_cluster_low[7:0]   <= sector_byte;
                    5'd27: dir_cluster_low[15:8]  <= sector_byte;
                    5'd28: dir_file_size[7:0]     <= sector_byte;
                    5'd29: dir_file_size[15:8]    <= sector_byte;
                    5'd30: dir_file_size[23:16]   <= sector_byte;
                    5'd31: begin
                        dir_file_size[31:24] <= sector_byte;
                        if (dir_entry_match) begin
                            case (dir_resource_index)
                                2'd0: begin
                                    resource0_cluster <= {dir_cluster_high, dir_cluster_low};
                                    resource0_size <= {sector_byte, dir_file_size[23:0]};
                                    resource_valid[0] <= 1'b1;
                                end
                                2'd1: begin
                                    resource1_cluster <= {dir_cluster_high, dir_cluster_low};
                                    resource1_size <= {sector_byte, dir_file_size[23:0]};
                                    resource_valid[1] <= 1'b1;
                                end
                                2'd2: begin
                                    resource2_cluster <= {dir_cluster_high, dir_cluster_low};
                                    resource2_size <= {sector_byte, dir_file_size[23:0]};
                                    resource_valid[2] <= 1'b1;
                                end
                                default: begin
                                    resource3_cluster <= {dir_cluster_high, dir_cluster_low};
                                    resource3_size <= {sector_byte, dir_file_size[23:0]};
                                    resource_valid[3] <= 1'b1;
                                end
                            endcase
                        end
                        // 音频文件：与上面互斥（一个目录项只可能匹配其中一种）
                        if (dir_audio_match) begin
                            audio_cluster <= {dir_cluster_high, dir_cluster_low};
                            audio_size    <= {sector_byte, dir_file_size[23:0]};
                            audio_valid   <= 1'b1;
                        end
                    end
                    default: ;
                endcase
            end

            case (state)
                ST_IDLE: if (scan_start) begin
                    fs_error <= ERR_NONE;
                    partition_lba <= 32'd0;
                    resource_valid <= 4'd0;
                    state <= ST_READ0_REQ;
                end

                ST_READ0_REQ: if (!sector_busy) begin
                    sector_lba <= 32'd0;
                    sector_req <= 1'b1;
                    state <= ST_READ0_WAIT;
                end

                ST_READ0_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    else if (sector_buffer[510] != 8'h55 || sector_buffer[511] != 8'haa)
                        finish_with_error(ERR_BOOT_SIGNATURE);
                    else if (sector_buffer[0] == 8'heb || sector_buffer[0] == 8'he9) begin
                        // 直接引导扇区（超级软盘格式），无 MBR
                        partition_lba <= 32'd0;
                        state <= ST_BOOT_PARSE;
                    end else if (sector_buffer[450] == 8'h0b || sector_buffer[450] == 8'h0c) begin
                        // MBR：从偏移 454 读分区起始 LBA
                        partition_lba <= le32(454);
                        sector_lba <= le32(454);
                        state <= ST_BOOT_REQ;
                    end else begin
                        finish_with_error(ERR_UNSUPPORTED_BPB);
                    end
                end

                ST_BOOT_REQ: if (!sector_busy) begin
                    sector_req <= 1'b1;
                    state <= ST_BOOT_WAIT;
                end

                ST_BOOT_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    else if (sector_buffer[510] != 8'h55 || sector_buffer[511] != 8'haa)
                        finish_with_error(ERR_BOOT_SIGNATURE);
                    else
                        state <= ST_BOOT_PARSE;
                end

                ST_BOOT_PARSE: begin
                    if (le16(11) != 16'd512 || !valid_spc(sector_buffer[13]) ||
                        le16(14) == 0 || (sector_buffer[16] != 1 && sector_buffer[16] != 2) ||
                        le16(17) != 0 || le16(22) != 0 || le32(32) == 0 ||
                        le32(36) == 0 || le32(44) < 2) begin
                        finish_with_error(ERR_UNSUPPORTED_BPB);
                    end else begin
                        sectors_per_cluster <= sector_buffer[13];
                        reserved_sectors <= le16(14);
                        fat_count <= sector_buffer[16];
                        total_sectors <= le32(32);
                        fat_sectors <= le32(36);
                        root_cluster <= le32(44);
                        sectors_per_cluster_shift <= spc_shift(sector_buffer[13]);
                        fat_span <= (sector_buffer[16] == 2) ? (le32(36) << 1) : le32(36);
                        fat_start_lba <= partition_lba + le16(14);
                        state <= ST_BOOT_LAYOUT;
                    end
                end

                ST_BOOT_LAYOUT: begin
                    data_start_lba <= partition_lba + reserved_sectors + fat_span;
                    data_sector_count <= total_sectors - reserved_sectors - fat_span;
                    state <= ST_BOOT_ROOT;
                end

                ST_BOOT_ROOT: begin
                    max_cluster <= (data_sector_count >> sectors_per_cluster_shift) + 1'b1;
                    root_lba <= data_start_lba + ((root_cluster - 2) << sectors_per_cluster_shift);
                    sector_lba <= data_start_lba + ((root_cluster - 2) << sectors_per_cluster_shift);
                    // ★ 多扇区扫描：整次扫描的累计结果只在这里清一次
                    root_sec_cnt <= 8'd0;
                    dir_end_seen <= 1'b0;
                    resource_valid <= 4'd0;
                    audio_valid <= 1'b0;
                    dir_entry_match <= 1'b0;
                    dir_audio_match <= 1'b0;
                    state <= ST_ROOT_REQ;
                end

                ST_ROOT_REQ: if (!sector_busy) begin
                    // ★ 只清"当前正在看的这一项"的匹配标志。
                    //   resource_valid / audio_valid 是**跨扇区累积**的结果，
                    //   绝不能在这里清 —— 本状态现在是每读一个扇区就进一次，
                    //   若在此清零会把前几个扇区已找到的图/音频全部抹掉。
                    dir_entry_match <= 1'b0;
                    dir_audio_match <= 1'b0;
                    sector_lba <= root_lba + {24'd0, root_sec_cnt};
                    sector_req <= 1'b1;
                    state <= ST_ROOT_WAIT;
                end

                ST_ROOT_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    // ① 见到 0x00 空闲项 → 后面全空，收尾
                    // ② 4 张图 + 音频都已找到 → 没必要再读
                    else if (dir_end_seen || (resource_valid == 4'hf && audio_valid))
                        state <= ST_DIR_SCAN;
                    // ③ 本簇还没读完 → 读本簇的下一个扇区（簇内扇区物理连续）
                    else if ((root_sec_cnt + 8'd1) < sectors_per_cluster) begin
                        root_sec_cnt <= root_sec_cnt + 8'd1;
                        state <= ST_ROOT_REQ;
                    end
                    // ④ 第一个簇扫完（不跟 FAT 链，见文件头"局限"）
                    else
                        state <= ST_DIR_SCAN;
                end

                ST_DIR_SCAN: begin
                    if (resource_valid == 4'hf)
                        state <= ST_FINISH;
                    else
                        finish_with_error(ERR_RESOURCE_MISSING);
                end

                ST_FINISH: begin
                    scan_done <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase

            end
        end
    end
endmodule
