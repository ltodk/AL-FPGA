# ============================================================================
# run_stage9.do —— 第9讲：OSD 滚动字幕层回归
#   验证：帧边界滚动 / 从右滚入 / Alpha 半透明底带 / 白字渲染 /
#         多条字幕循环 / enable 关闭透传 / 无撕裂
# ============================================================================
vlib work_9
vlog -quiet -work work_9 ../src/user_source/hdl_source/SD/subtitle_scroll_overlay.v tb_subtitle_scroll.v
vsim -c -quiet work_9.tb_subtitle_scroll
run -all
quit -f
