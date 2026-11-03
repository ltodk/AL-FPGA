`timescale 1ns/1ps
// ============================================================================
// sd_sector_arbiter.v —— SD 扇区读三路逐扇区仲裁（第 11 讲 阶段 B 新增）
//
// 背景：第 7~10 讲只有两个消费者（FAT32 扫描器 / 文件流读器），而且它们是
//       **分时**使用的（扫描完成后 use_streamer 一置，SD 就整段归流读器），
//       所以历史实现是"按阶段选 owner"的组合三目：
//           arb_sector_req = use_streamer ? streamer_sector_req : scanner_sector_req;
//       这种写法对"一次读几 MB 的图片"完全够用，但对音频是致命的：
//       音频按 48kHz×2B = 96KB/s 需要每 5.33ms 一个扇区，如果图片加载期间
//       owner 一直挂在图片上，音频会被饿死到断流、咔咔响。
//
// 本模块把仲裁粒度从"阶段"降到"扇区"：每完成一个 CMD17，就重新判一次归属。
//
// 优先级（严格）：音频(c2) > 图片(c1) > 扫描器(c0)
//   为什么严格优先级不会饿死图片：音频哪怕在疯狂补缓冲，长期占空比也只有
//   6~8%（见工程状态文档 17.4 的带宽核算），图片拿剩下 92%+，加载时间只变长
//   约 7%，肉眼无感。反过来若给图片固定配额，音频就会出现周期性断流，
//   那是**能听出来**的缺陷。所以"宁可让图片慢 7%，也不让音频卡一下"。
//   扫描器排最后是安全的：它只在开机那一瞬间跑，此时音频才开始灌缓冲，
//   音频缓冲很快灌满就停下，扫描反而是先跑完的。
//
// ★ 设计要点：请求是"脉冲"，所以必须登记
//   三个客户端的 sector_req 都是**单拍脉冲**（见 fat32_volume_scanner /
//   fat32_file_streamer / wav_sector_feeder 的 REQ 状态）。若本拍不选中它，
//   这个脉冲就永远消失了，客户端会卡死在 WAIT 态等一个不会来的 done。
//   因此每个客户端配一个 pending 位：脉冲到来就置位，它自己那一笔完成后清零。
//   选中的依据是 pending 而不是当拍的 req —— 这样"晚到的选中"也不会丢请求。
//
// 协议沿用 sd_sector_adapter 的 sector_* 接口：
//   客户端：拉 sector_req(单拍) + sector_lba → 等 sector_busy → sector_valid×512
//           → sector_done（错误码与 done 同拍）
//   本模块：把被选中客户端的 req/lba 转给 adapter，把 adapter 的返回**只**路由
//           给被选中的客户端（其余客户端看到的 busy/valid/done 全为 0）。
//
// 复位：低有效 rst_n，abort 一拍清空全部在途状态（拔卡 / 重扫时用）。
// ============================================================================
module sd_sector_arbiter (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        abort,               // 整栈软复位：一拍清空在途请求

    // === 客户端 0：FAT32 卷扫描器（最低优先级）===
    input  wire        c0_req,
    input  wire [31:0] c0_lba,
    output wire        c0_busy,
    output wire        c0_valid,
    output wire [7:0]  c0_byte,
    output wire [8:0]  c0_byte_index,
    output wire        c0_done,
    output wire [7:0]  c0_error,

    // === 客户端 1：图片文件流读器 ===
    input  wire        c1_req,
    input  wire [31:0] c1_lba,
    output wire        c1_busy,
    output wire        c1_valid,
    output wire [7:0]  c1_byte,
    output wire [8:0]  c1_byte_index,
    output wire        c1_done,
    output wire [7:0]  c1_error,

    // === 客户端 2：WAV 音频文件流读器（最高优先级）===
    input  wire        c2_req,
    input  wire [31:0] c2_lba,
    output wire        c2_busy,
    output wire        c2_valid,
    output wire [7:0]  c2_byte,
    output wire [8:0]  c2_byte_index,
    output wire        c2_done,
    output wire [7:0]  c2_error,

    // === 主口：对接 sd_sector_adapter ===
    output reg         m_req,
    output reg  [31:0] m_lba,
    input  wire        m_busy,
    input  wire        m_valid,
    input  wire [7:0]  m_byte,
    input  wire [8:0]  m_byte_index,
    input  wire        m_done,
    input  wire [7:0]  m_error,

    // === 诊断：当前归属 0/1/2，无归属时为 7 ===
    output wire [2:0]  grant_dbg
);

    localparam [1:0] G_NONE = 2'd3;

    reg [1:0] grant;        // 当前归属（G_NONE = 空闲）
    reg [2:0] pending;      // 每位：该客户端有一个已登记、尚未完成的请求

    assign grant_dbg = (grant == G_NONE) ? 3'd7 : {1'b0, grant};

    // 本拍是否有"被选中客户端完成"（用于清 pending）
    wire sel0 = (grant == 2'd0);
    wire sel1 = (grant == 2'd1);
    wire sel2 = (grant == 2'd2);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            grant   <= G_NONE;
            pending <= 3'b000;
            m_req   <= 1'b0;
            m_lba   <= 32'd0;
        end else if (abort) begin
            // 拔卡 / 重扫：撤销在途请求。客户端同时被 abort 复位，不会再等 done。
            grant   <= G_NONE;
            pending <= 3'b000;
            m_req   <= 1'b0;
        end else begin
            // m_req 是单拍：默认撤销，选中分支里再置 1
            m_req <= 1'b0;

            // ---- 1) 登记新请求（脉冲到来即置位）----
            if (c0_req) pending[0] <= 1'b1;
            if (c1_req) pending[1] <= 1'b1;
            if (c2_req) pending[2] <= 1'b1;

            // ---- 2) 归属判定 / 释放 ----
            if (grant == G_NONE) begin
                // 严格优先级：音频 → 图片 → 扫描器
                // 条件里带上当拍 req，是为了"空闲 + 请求同拍到"能零延迟发出，
                // 不白等一个周期（pending 的置位与清零同时发生时，清零在后，胜出）
                if (pending[2] || c2_req) begin
                    grant     <= 2'd2;
                    pending[2] <= 1'b0;
                    m_req     <= 1'b1;
                    m_lba     <= c2_lba;
                end else if (pending[1] || c1_req) begin
                    grant     <= 2'd1;
                    pending[1] <= 1'b0;
                    m_req     <= 1'b1;
                    m_lba     <= c1_lba;
                end else if (pending[0] || c0_req) begin
                    grant     <= 2'd0;
                    pending[0] <= 1'b0;
                    m_req     <= 1'b1;
                    m_lba     <= c0_lba;
                end
            end else if (m_done) begin
                // 本扇区结束，释放归属；下一拍重新判，实现"逐扇区仲裁"
                grant <= G_NONE;
            end
        end
    end

    // ---- 返回信号只路由给被选中的客户端 ----
    // 未被选中者 busy/valid/done 恒 0：它们若要发请求，看到 !busy 就发，
    // 请求被 pending 接住、等自己那一轮；不会误收到别人的数据。
    assign c0_busy       = sel0 && (grant != G_NONE) && m_busy;
    assign c0_valid      = sel0 && (grant != G_NONE) && m_valid;
    assign c0_byte       = m_byte;
    assign c0_byte_index = m_byte_index;
    assign c0_done       = sel0 && (grant != G_NONE) && m_done;
    assign c0_error      = m_error;

    assign c1_busy       = sel1 && (grant != G_NONE) && m_busy;
    assign c1_valid      = sel1 && (grant != G_NONE) && m_valid;
    assign c1_byte       = m_byte;
    assign c1_byte_index = m_byte_index;
    assign c1_done       = sel1 && (grant != G_NONE) && m_done;
    assign c1_error      = m_error;

    assign c2_busy       = sel2 && (grant != G_NONE) && m_busy;
    assign c2_valid      = sel2 && (grant != G_NONE) && m_valid;
    assign c2_byte       = m_byte;
    assign c2_byte_index = m_byte_index;
    assign c2_done       = sel2 && (grant != G_NONE) && m_done;
    assign c2_error      = m_error;

endmodule
