# ============================================================================
# run_stage13.do —— 第13讲 图片缩放适配 回归
#
# 两块：
#   1) tb_bmp_scaler        最近邻缩放器单元测试（9 个用例，坐标编码逐点比对）
#        C0 640×480  1:1 强制**走行缓冲** —— 覆盖旁路 mux 的另一支路
#        C1 640×480  1:1 走**旁路**      —— 输出必须与输入逐像素完全一致
#        C2 1280×960 2:1 缩小
#        C3 320×240  2:1 放大      —— 每个源像素/源行重复 2 次
#        C4 800×600  非整数比例 1.25:1
#        C5 1024×768 非整数 + 反压
#        C6 1920×1080 全高清缩小   —— 真实场景（3:1）
#        C7 640×240  垂直 2:1 放大 —— 水平恰好 1:1，覆盖行缓冲里的水平直通通路
#        C8 320×240 + 1/8 反压     —— ★ 溢出探针：故意让行缓冲占满，验证
#           **即使大量 DROP，帧仍必须完整**（pub=源行数、out/fire=307200）。
#           这是实机 B 轮 ERR:0x38 的回归用例：修前 out 只有 48640/307200
#           → frame_commit 不来 → 500ms 写等待超时 → 0x38。
#      每个用例都强制检查：输出像素总数 307200、out_fire 次数 307200、无 overflow，
#      以及 ★ out_valid=1 时必须 out_ready=1（第13讲 B 轮 ERR:0x42 的回归项：
#      media_frame_loader 会在"valid 悬空"的那一拍判 FIFO 溢出并 abort 整帧）
#   2) tb_decoder_errcodes  BMP 头解析错误码回归（尺寸校验由「必须等于」放开
#      成「不超过上限」，并新增 800×600 / 1280×720 / 320×240 合法用例）
#
# 用法（在 sim 目录下）：
#   set PATH=D:\FG_software\modelsim\win64;%PATH%
#   vsim -c -do run_stage13.do
#
# 注意：do 文件里的 vsim 必须用库名限定，且 testbench 的 $finish 会结束整个
#       批处理会话，所以用 exec 起子进程。
# ============================================================================
vlib work_13_scale
vlog -sv -quiet -work work_13_scale \
    ../src/user_source/hdl_source/SD/bmp_scale_640x480.v \
    tb_bmp_scaler.v

echo "===== 1/2 tb_bmp_scaler ====="
exec vsim -c -quiet -work work_13_scale tb_bmp_scaler -do "run -all; quit -f"

vlib work_13_dec
vlog -sv -quiet -work work_13_dec \
    ../src/user_source/hdl_source/SD/bmp24_decoder.v \
    tb_decoder_errcodes.v

echo "===== 2/2 tb_decoder_errcodes ====="
exec vsim -c -quiet -work work_13_dec tb_decoder_errcodes -do "run -all; quit -f"

echo "===== ALL DONE ====="
quit -f
