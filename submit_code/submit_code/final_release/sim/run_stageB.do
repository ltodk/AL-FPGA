# ============================================================================
# run_stageB.do —— 第8讲 阶段B 顶层读侧链路集成验证
#
# 用法（在 sim 目录下）：
#   set PATH=D:\FG_software\modelsim\win64;%PATH%
#   vsim -c -do run_stageB.do
# ============================================================================
vlib work_b2
vlog -sv -work work_b2 \
    ../src/user_source/hdl_source/SD/video_define.v \
    ../src/user_source/hdl_source/SD/color_bar.v \
    ../src/user_source/hdl_source/SD/video_timing_data.v \
    ../src/user_source/hdl_source/SD/frame_fifo_read.v \
    ../src/user_source/hdl_source/IP/afifo_32_16_256.v \
    ../src/user_source/hdl_source/SD/fifo_prefetch_2.v \
    ../src/user_source/hdl_source/SD/media_frame_reader.v \
    ../src/user_source/hdl_source/SD/rgb_video_adapter.v \
    tb_stageB_readchain.v

vsim -c work_b2.tb_stageB_readchain -do "run -all; quit -f"