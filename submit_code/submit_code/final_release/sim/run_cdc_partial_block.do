# ============================================================================
# run_cdc_partial_block.do —— ERR:3A / ERR:0x38 两个静默故障的定点回归
#
# 用例：tb_cdc_partial_block
#   TEST A  满判据相位差（loader 的写使能是寄存器输出，判满用的却是上一拍的
#           src_full）→ 边界上会丢字且不报错 → 实机 ERR:3A
#   TEST B  残留不满块（< BURST_SIZE 个字凑不满一块）→ 目的侧永远不发、
#           dst_idle 永远不拉高 → 上游等不到写完成 → 实机 ERR:0x38 切不了图
#   TEST C  正常整帧（字数恰为 BURST_SIZE 整数倍）不受"帧启动收尾"影响
#
# 用法（在 sim 目录下）：
#   vsim -c -do run_cdc_partial_block.do
#
# 注意：tb 里的 $finish 会结束整个批处理会话，所以必须用 exec 起子进程。
# ============================================================================
vlib work_cdc_pb
vlog -sv -quiet -work work_cdc_pb \
    ../src/user_source/hdl_source/SD/media_sdram_write_cdc.v \
    tb_cdc_partial_block.v

echo "===== 1/1 tb_cdc_partial_block ====="
exec vsim -c -quiet -work work_cdc_pb tb_cdc_partial_block -do "run -all; quit -f"

echo "===== ALL DONE ====="
quit -f
