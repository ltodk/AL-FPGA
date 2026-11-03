# ============================================================================
# run_uart_text.do —— uart_logger 输出文本回归
#
# 用途：2026-10-21 对 uart_logger 做了时序优化（hex_ascii / 十进制数字提前算进
#       寄存器），本脚本验证 UART 输出的 10 种消息文本逐字符不变。
#
# 用法（在 sim 目录下）：
#   set PATH=D:\FG_software\modelsim\win64;%PATH%
#   vsim -c -do run_uart_text.do
#
# 期望输出（每段前面有 "---- phase N  expect: ..."）：
#   1  SD OK        2  SD OUT      3  IMG:2/4     4  ERR:0x38    5  WR:3S
#   6  RD:3S        7  DS:00004000 8  DS:FAIL     9  S1:00001234  10 S0:00002345
# ============================================================================
vlib work
vlog -sv -quiet -work work +incdir+../src/user_source/hdl_source/SD +incdir+../src/user_source/hdl_source/include ../src/user_source/hdl_source/SD/uart_tx.v ../src/user_source/hdl_source/SD/uart_logger.v tb_uart_text.v
vsim -c -quiet -work work tb_uart_text
run -all
quit -f
