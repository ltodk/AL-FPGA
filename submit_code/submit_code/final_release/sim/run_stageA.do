# ============================================================================
# run_stageA.do —— 第8讲 阶段A（四 bank 写通道）回归
#
# 用途：验证 media_frame_loader + media_sdram_write_cdc 这条新写通道
#   ① tb_loader_vflip  —— V_FLIP 行地址翻转序列、像素计数、短帧错误码
#   ② tb_wr_cdc_chain  —— 100MHz loader → 125MHz CDC 跨域：数据不丢、地址一致、
#                         dst_idle 只在全部搬完后拉高
#   ③ tb_wr_arbiter    —— 写通道与读通道互斥：burst 不断流、让位窗口、
#                         读写 burst 零重叠、读侧长期抢占时写侧不被饿死
#
# 用法（在 sim 目录下）：
#   set PATH=D:\FG_software\modelsim\win64;%PATH%
#   vsim -c -do run_stageA.do
# ============================================================================
vlib work_a
vlog -sv -quiet -work work_a ../src/user_source/hdl_source/SD/media_frame_loader.v ../src/user_source/hdl_source/SD/media_sdram_write_cdc.v tb_loader_vflip.v tb_wr_cdc_chain.v tb_wr_arbiter.v

echo "===== tb_loader_vflip ====="
vsim -c -work work_a tb_loader_vflip -do "run -all; quit -f"

echo "===== tb_wr_cdc_chain ====="
vsim -c -work work_a tb_wr_cdc_chain -do "run -all; quit -f"

echo "===== tb_wr_arbiter ====="
vsim -c -work work_a tb_wr_arbiter -do "run -all; quit -f"

