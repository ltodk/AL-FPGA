# ============================================================================
# run_fullframe.do —— 全帧写入链路验证
#   tb_loader_fullframe：307200 像素整帧，检查 CDC 输出的 (addr,data) 序列
#   期望：总写入 307200 次、无重复、无遗漏、数据全部符合 V_FLIP 映射
# 用法（sim 目录下）：vsim -c -do run_fullframe.do
# ============================================================================
vlib work_ff
vlog -sv -quiet -work work_ff ../src/user_source/hdl_source/SD/media_frame_loader.v ../src/user_source/hdl_source/SD/media_sdram_write_cdc.v tb_loader_fullframe.v
vsim -c -quiet work_ff.tb_loader_fullframe
run -all
quit -f
