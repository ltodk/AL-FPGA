# ============================================================================
# run_hotplug.do —— 拔插卡死锁修复回归（第八讲）
#
# 用法（在 sim 目录下）：
#   set PATH=D:\FG_software\modelsim\win64;%PATH%
#   vsim -c -do run_hotplug.do
# ============================================================================
vlib work
vlog -sv -quiet -work work +incdir+../src/user_source/hdl_source/SD ../src/user_source/hdl_source/SD/spi_master.v ../src/user_source/hdl_source/SD/sd_card_cmd.v ../src/user_source/hdl_source/SD/sd_card_sec_read_write.v ../src/user_source/hdl_source/SD/sd_card_top.v ../src/user_source/hdl_source/SD/sd_sector_adapter.v ../src/user_source/hdl_source/SD/fat32_volume_scanner.v ../src/user_source/hdl_source/SD/fat32_file_streamer.v ../src/user_source/hdl_source/SD/sd_sector_arbiter.v ../src/user_source/hdl_source/SD/wav_sector_feeder.v ../src/user_source/hdl_source/hdmi_audio/wav_progress.v ../src/user_source/hdl_source/SD/bmp24_decoder.v ../src/user_source/hdl_source/SD/sd_card_bmp.v sd_spi_card_model.v tb_hotplug_recovery.v tb_streamer_lostdone.v
vsim -c -quiet -work work tb_hotplug_recovery
run -all
quit -f
