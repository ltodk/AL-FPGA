# ============================================================================
# run_stage11b.do —— 第11讲 阶段B（WAV 音频播放 + SD 三路仲裁）回归
#
# 四个自包含单元测试，共 46 项检查：
#   ① tb_sd_sector_arbiter  —— 三路逐扇区仲裁：优先级、迟到请求不丢、
#                              逐扇区重判、数据不串户、abort、不饿死        (12 项)
#   ② tb_wav_async_fifo     —— 100MHz 写 / 25MHz 读异步 FIFO：
#                              灌满、排空顺序、并发 3000 字不丢不重、流控    (11 项)
#   ③ tb_wav_stream_player  —— 48kHz 节拍精度（4800 节拍 = 100ms 整）、
#                              音量缩放逐样本比对、断流补静音、active 状态机 (12 项)
#   ④ tb_wav_sector_feeder  —— PCM 字节→16bit 小端字、扇区级流控、循环、停止重启(11 项)
#
# 用法（在 sim 目录下）：
#   set PATH=D:\FG_software\modelsim\win64;%PATH%
#   vsim -c -do run_stage11b.do
#
# ★ 为什么用 exec 起子进程，而不是直接写四个 vsim 命令
#   实测两个坑（都踩过）：
#     ① do 文件里的 vsim 命令会**丢掉 -work 参数**（日志里能看到参数字符串变短），
#        必须写成「库名限定」work_11b_all.tb_xxx 才可靠；
#     ② testbench 里的 $finish 会把整个批处理会话一起结束掉，
#        后面三个 testbench 根本跑不到（日志里只有 1/4）。
#   用 exec 起独立的 vsim 子进程，子进程自己退出，父会话继续 —— 干净可靠。
# ============================================================================
vlib work_11b_all
vlog -sv -quiet -work work_11b_all \
    ../src/user_source/hdl_source/SD/sd_sector_arbiter.v \
    ../src/user_source/hdl_source/SD/wav_sector_feeder.v \
    ../src/user_source/hdl_source/SD/fat32_file_streamer.v \
    ../src/user_source/hdl_source/hdmi_audio/wav_async_fifo16.v \
    ../src/user_source/hdl_source/hdmi_audio/wav_stream_player.v \
    tb_sd_sector_arbiter.v \
    tb_wav_async_fifo.v \
    tb_wav_stream_player.v \
    tb_wav_sector_feeder.v

echo "===== 1/4 tb_sd_sector_arbiter ====="
exec vsim -c -quiet -work work_11b_all tb_sd_sector_arbiter -do "run -all; quit -f"

echo "===== 2/4 tb_wav_async_fifo ====="
exec vsim -c -quiet -work work_11b_all tb_wav_async_fifo -do "run -all; quit -f"

echo "===== 3/4 tb_wav_stream_player ====="
exec vsim -c -quiet -work work_11b_all tb_wav_stream_player -do "run -all; quit -f"

echo "===== 4/4 tb_wav_sector_feeder ====="
exec vsim -c -quiet -work work_11b_all tb_wav_sector_feeder -do "run -all; quit -f"

echo "===== ALL DONE ====="
quit -f
