# ============================================================================
# run_stage13_regress.do —— 第13讲 改动点的既有回归
#
# 第13讲动了三个文件：
#   bmp24_decoder.v      尺寸校验 from「必须等于 640×480」改成「不超过上限」，
#                        文件长度改成按真实 W×H 计算
#   sd_card_bmp.v        BMP 行 4 字节对齐 padding 显式剔除，引出真实分辨率
#   media_frame_loader.v 新增 pixel_ready 反压输出
# 所以必须回归这些文件的下游使用者，确认行为没退化。
#
#   1) tb_bmp_read_stability  像素读通路 / 坏图静默跳过
#   2) tb_fat_fastboot        FAT32 快速启动（跨簇读）
#   3) tb_hotplug_recovery    拔插卡自恢复 + 扇区仲裁 + WAV
#   4) tb_stageC_preload      四 bank 预加载 + 零延迟切图
#
# 用法（在 sim 目录下）：
#   set PATH=D:\FG_software\modelsim\win64;%PATH%
#   vsim -c -do run_stage13_regress.do
#
# 注意：vsim 必须用库名限定，且 tb 的 $finish 会结束整个批处理会话，
#       所以每个用例用 exec 起子进程。
# ============================================================================
set SD ../src/user_source/hdl_source/SD
set AUD ../src/user_source/hdl_source/hdmi_audio
set INC +incdir+../src/user_source/hdl_source/SD

# ---- 1) 像素读通路稳定性 ----
vlib work_r13_stab
vlog -sv -quiet -work work_r13_stab $INC $SD/bmp_err_def.vh $SD/bmp24_decoder.v $SD/bmp_read.v tb_bmp_read_stability.v
echo "===== 1/4 tb_bmp_read_stability ====="
exec vsim -c -quiet -work work_r13_stab tb_bmp_read_stability -do "run -all; quit -f"

# ---- 2) FAT32 快速启动 ----
vlib work_r13_fat
vlog -sv -quiet -work work_r13_fat $INC $SD/bmp_err_def.vh $SD/bmp24_decoder.v $SD/bmp_read.v tb_fat_fastboot.v
echo "===== 2/4 tb_fat_fastboot ====="
exec vsim -c -quiet -work work_r13_fat tb_fat_fastboot -do "run -all; quit -f"

# ---- 3) 拔插卡自恢复 ----
vlib work_r13_hot
vlog -sv -quiet -work work_r13_hot $INC \
    $SD/spi_master.v $SD/sd_card_cmd.v $SD/sd_card_sec_read_write.v $SD/sd_card_top.v \
    $SD/sd_sector_adapter.v $SD/fat32_volume_scanner.v $SD/fat32_file_streamer.v \
    $SD/sd_sector_arbiter.v $SD/wav_sector_feeder.v $AUD/wav_progress.v \
    $SD/bmp24_decoder.v $SD/sd_card_bmp.v \
    sd_spi_card_model.v tb_hotplug_recovery.v
echo "===== 3/4 tb_hotplug_recovery ====="
exec vsim -c -quiet -work work_r13_hot tb_hotplug_recovery -do "run -all; quit -f"

# ---- 4) 四 bank 预加载 + 零延迟切图 ----
vlib work_r13_c
vlog -sv -quiet -work work_r13_c $INC \
    $SD/spi_master.v $SD/sd_card_cmd.v $SD/sd_card_sec_read_write.v $SD/sd_card_top.v \
    $SD/sd_sector_adapter.v $SD/fat32_volume_scanner.v $SD/fat32_file_streamer.v \
    $SD/sd_sector_arbiter.v $SD/wav_sector_feeder.v $AUD/wav_progress.v \
    $SD/bmp24_decoder.v $SD/sd_card_bmp.v \
    sd_spi_card_model.v tb_stageC_preload.v
echo "===== 4/4 tb_stageC_preload ====="
exec vsim -c -quiet -work work_r13_c tb_stageC_preload -do "run -all; quit -f"

echo "===== ALL DONE ====="
quit -f
