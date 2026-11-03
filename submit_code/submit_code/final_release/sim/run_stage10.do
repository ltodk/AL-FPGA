# ============================================================================
# run_stage10.do —— 第10讲：图片转场（淡出→全黑→换bank→淡入）回归
#
#   tb_fade_apply        A1~A6  alpha 乘法（alpha=255 逐位精确 / alpha=0 纯黑 /
#                                256 档全扫与理论值比对 / 单调性 / 通道独立）
#   tb_fade_transition   T1~T5  状态机（复位初值 / 无图不介入 / 帧内 alpha 恒定 /
#                                淡出递减 / 全黑时刻换 bank / 淡入递增 /
#                                转场途中再切图仍能追到最新值）
#
#   运行： vsim -c -do run_stage10.do
# ============================================================================
vlib work_10

vlog -sv -quiet -work work_10 \
    ../src/user_source/hdl_source/SD/media_fade_transition.v \
    ../src/user_source/hdl_source/SD/fade_alpha_apply.v \
    tb_fade_apply.v tb_fade_transition.v

# ★ 每个 testbench 结尾都有 $finish，在 -c 模式下 $finish 会直接结束
#   整个 vsim 进程，所以不能在本脚本里顺序写多个 vsim。
#   改用 exec 各自拉起独立进程，依次执行。
echo "==================== tb_fade_apply ===================="
exec vsim -c -quiet work_10.tb_fade_apply -do "run -all; quit -f"

echo "==================== tb_fade_transition ===================="
exec vsim -c -quiet work_10.tb_fade_transition -do "run -all; quit -f"

quit -f
