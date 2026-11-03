# ============================================================================
# run_stage14.do —— 第14讲：亮度/对比度调节回归
#
#   三个 testbench 依次跑：
#     tb_bc_adjust    T1~T9  像素运算（严格旁路 / 亮度 / 对比度 / 饱和截断 /
#                            负数算术右移 / 帧边界锁存 / 2 拍延迟对齐）
#     tb_param_adjust P0~P6  按键控制（消抖 / 步进 / 长按连发 / 饱和不回绕 /
#                            显示窗口自动关闭）
#     tb_osd_param    P1~P8  参数回显（十进制补零 / 99→100 进位 / 200+ /
#                            字符渲染）—— 直接打印出 "B:255 C:128"
#
#   运行： vsim -c -do run_stage14.do
# ============================================================================
vlib work_14

vlog -sv -quiet -work work_14 \
    +incdir+../src/user_source/hdl_source/include \
    +incdir+../src/user_source/hdl_source/SD \
    ../src/user_source/hdl_source/SD/brightness_contrast_adjust.v \
    ../src/user_source/hdl_source/SD/param_adjust_ctrl.v \
    ../src/user_source/hdl_source/SD/osd_overlay.v \
    tb_bc_adjust.v tb_param_adjust.v tb_osd_param.v

# ★ 每个 testbench 结尾都有 $finish，在 -c 模式下 $finish 会直接结束
#   整个 vsim 进程，所以不能在本脚本里顺序写多个 vsim。
#   改用 exec 各自拉起独立进程，依次执行。
echo "==================== tb_bc_adjust ===================="
exec vsim -c -quiet work_14.tb_bc_adjust -do "run -all; quit -f"

echo "==================== tb_param_adjust ===================="
exec vsim -c -quiet work_14.tb_param_adjust -do "run -all; quit -f"

echo "==================== tb_osd_param ===================="
exec vsim -c -quiet work_14.tb_osd_param -do "run -all; quit -f"

quit -f
