`timescale 1ns/1ps
// ============================================================================
// tb_streamer_lostdone.v —— "扇区完成脉冲丢失"前后对比（bug 机理证明）
//
// 场景：加载途中卡被拔掉 / 适配层的 sector_done 被别的客户端吃掉，
//       fat32_file_streamer 永远等不到"读完成"。两个实例吃完全相同的激励，
//       只有恢复机制不同：
//
//   u_old ：SECTOR_TIMEOUT_CYCLES = 32'h7FFF_FFFF（等价原代码：没有看门狗），
//           abort 恒为 0（原代码没有 abort 端口）
//           -> 永久卡在 DATA_WAIT，之后所有 file_start 全部被忽略
//              （这就是"插回卡后每张图都超时、ERR:0x38、只能复位"的根因）
//   u_new ：小看门狗 + 真实 abort 通路
//           -> 超时收场（fs_error=0x10 ERR_SECTOR）回 IDLE，重新 file_start 能读完
// ============================================================================
module tb_streamer_lostdone;

    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0;

    reg         file_start    = 1'b0;
    reg  [31:0] first_cluster = 32'd3;
    reg  [31:0] file_size     = 32'd600;
    reg         drop_en       = 1'b0;   // 1 = 扇区读完但不发 done（完成脉冲丢失）

    // ---------------- 旧实例（等价原代码）----------------
    wire        old_busy, old_fdone, old_fvalid;
    wire [7:0]  old_fbyte, old_ferr;
    wire [31:0] old_foff;
    wire        old_req;  wire [31:0] old_lba;
    wire        old_sbusy, old_svalid, old_sdone;
    wire [7:0]  old_sbyte; wire [8:0] old_sidx;

    // ---------------- 新实例（修复后）----------------
    wire        new_busy, new_fdone, new_fvalid;
    wire [7:0]  new_fbyte, new_ferr;
    wire [31:0] new_foff;
    wire        new_req;  wire [31:0] new_lba;
    wire        new_sbusy, new_svalid, new_sdone;
    wire [7:0]  new_sbyte; wire [8:0] new_sidx;

    fat32_file_streamer #(
        .SECTOR_TIMEOUT_CYCLES (32'h7FFF_FFFF)     // 等价"没有看门狗"
    ) u_old (
        .clk(clk), .rst_n(rst_n), .abort(1'b0), .file_start(file_start),
        .first_cluster(first_cluster), .file_size(file_size),
        .sectors_per_cluster(8'd1), .sectors_per_cluster_shift(3'd0),
        .fat_start_lba(32'd33),
        .data_start_lba(32'd49), .max_cluster(32'd100),
        .file_busy(old_busy), .file_done(old_fdone),
        .file_valid(old_fvalid), .file_byte(old_fbyte),
        .file_byte_offset(old_foff), .fs_error(old_ferr),
        .sector_req(old_req), .sector_lba(old_lba),
        .sector_busy(old_sbusy), .sector_valid(old_svalid),
        .sector_byte(old_sbyte), .sector_byte_index(old_sidx),
        .sector_done(old_sdone), .sector_error(8'd0)
    );

    fat32_file_streamer #(
        .SECTOR_TIMEOUT_CYCLES (2_000)             // 20us
    ) u_new (
        .clk(clk), .rst_n(rst_n), .abort(1'b0), .file_start(file_start),
        .first_cluster(first_cluster), .file_size(file_size),
        .sectors_per_cluster(8'd1), .sectors_per_cluster_shift(3'd0),
        .fat_start_lba(32'd33),
        .data_start_lba(32'd49), .max_cluster(32'd100),
        .file_busy(new_busy), .file_done(new_fdone),
        .file_valid(new_fvalid), .file_byte(new_fbyte),
        .file_byte_offset(new_foff), .fs_error(new_ferr),
        .sector_req(new_req), .sector_lba(new_lba),
        .sector_busy(new_sbusy), .sector_valid(new_svalid),
        .sector_byte(new_sbyte), .sector_byte_index(new_sidx),
        .sector_done(new_sdone), .sector_error(8'd0)
    );

    // ---------------- 扇区服务器（两份，行为一致）----------------
    reg        old_bsy, old_vl, old_dn;
    reg [9:0]  old_bc;
    reg [8:0]  old_ix;
    reg [7:0]  old_by;
    assign old_sbusy  = old_bsy;
    assign old_svalid = old_vl;
    assign old_sdone  = old_dn;
    assign old_sbyte  = old_by;
    assign old_sidx   = old_ix;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            old_bsy <= 1'b0; old_bc <= 10'd0; old_dn <= 1'b0;
            old_vl  <= 1'b0; old_ix <= 9'd0;   old_by <= 8'hA5;
        end else begin
            old_dn <= 1'b0; old_vl <= 1'b0;
            if (!old_bsy) begin
                if (old_req) begin old_bsy <= 1'b1; old_bc <= 10'd0; end
            end else if (old_bc < 10'd512) begin
                old_vl <= 1'b1; old_by <= 8'hA5;
                old_ix <= old_bc[8:0];
                old_bc <= old_bc + 10'd1;
            end else begin
                old_bsy <= 1'b0;
                if (!drop_en) old_dn <= 1'b1;      // drop_en=1 => 完成脉冲丢失
            end
        end
    end

    reg        new_bsy, new_vl, new_dn;
    reg [9:0]  new_bc;
    reg [8:0]  new_ix;
    reg [7:0]  new_by;
    assign new_sbusy  = new_bsy;
    assign new_svalid = new_vl;
    assign new_sdone  = new_dn;
    assign new_sbyte  = new_by;
    assign new_sidx   = new_ix;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            new_bsy <= 1'b0; new_bc <= 10'd0; new_dn <= 1'b0;
            new_vl  <= 1'b0; new_ix <= 9'd0;   new_by <= 8'hA5;
        end else begin
            new_dn <= 1'b0; new_vl <= 1'b0;
            if (!new_bsy) begin
                if (new_req) begin new_bsy <= 1'b1; new_bc <= 10'd0; end
            end else if (new_bc < 10'd512) begin
                new_vl <= 1'b1; new_by <= 8'hA5;
                new_ix <= new_bc[8:0];
                new_bc <= new_bc + 10'd1;
            end else begin
                new_bsy <= 1'b0;
                if (!drop_en) new_dn <= 1'b1;
            end
        end
    end

    // ---------------- 计数 / 判定 ----------------
    integer old_done_cnt = 0;
    integer new_done_cnt = 0;
    integer errs = 0;

    always @(posedge clk) begin
        if (old_fdone) old_done_cnt <= old_done_cnt + 1;
        if (new_fdone) new_done_cnt <= new_done_cnt + 1;
    end

    task check;
        input        cond;
        input [8*72-1:0] msg;
        begin
            if (cond) $display("  [PASS] %0s", msg);
            else begin $display("  [FAIL] %0s", msg); errs = errs + 1; end
        end
    endtask

    integer o0, n0;

    initial begin
        #100 rst_n = 1'b1;
        #100;

        // ---- 第 1 次：正常读（两个实例都应完成）----
        file_start = 1'b1; #20; file_start = 1'b0;
        wait ((old_done_cnt > 0) && (new_done_cnt > 0));
        #100;
        $display("\n[1] normal read: old_done=%0d new_done=%0d", old_done_cnt, new_done_cnt);
        check (old_done_cnt > 0 && new_done_cnt > 0, "both instances read the file");

        // ---- 第 2 次：制造"完成脉冲丢失"----
        #200;
        drop_en    = 1'b1;
        file_start = 1'b1; #20; file_start = 1'b0;
        #2000;                       // 让两个实例都发出请求、都卡在等 done
        #30000;                      // 远超新实例的看门狗（20us）
        $display("\n[2] after lost completion (no done pulse):");
        $display("    old: busy=%0d fs_error=0x%02h", old_busy, old_ferr);
        $display("    new: busy=%0d fs_error=0x%02h", new_busy, new_ferr);
        check (old_busy === 1'b1 && old_ferr == 8'h00, "OLD streamer stuck forever (bug reproduced)");
        check (new_busy === 1'b0 && new_ferr == 8'h10, "NEW streamer timed out and left IDLE");

        // ---- 第 3 次：恢复后重新发起读，验证旧的收不到、新的能收 ----
        o0 = old_done_cnt;
        n0 = new_done_cnt;
        drop_en    = 1'b0;
        #200;
        file_start = 1'b1; #20; file_start = 1'b0;
        #60000;
        $display("\n[3] restart after recovery: old_done+%0d new_done+%0d",
                 old_done_cnt - o0, new_done_cnt - n0);
        check ((old_done_cnt - o0) == 0, "OLD streamer ignores new file_start (dead forever)");
        check ((new_done_cnt - n0) > 0, "NEW streamer accepts new file_start");

        $display("\n===== RESULT: %0s (errors=%0d) =====",
                 (errs == 0) ? "PASS" : "FAIL", errs);
        $finish;
    end

    initial begin
        #2000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
