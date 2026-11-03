# ============================================================================
# run_stageC.do —— 第8讲 阶段C：四 bank 预加载 + 零延迟切图回归
# ============================================================================
vlib work_c
vlog -sv -quiet -work work_c +incdir+../src/user_source/hdl_source/SD ../src/user_source/hdl_source/SD/spi_master.v ../src/user_source/hdl_source/SD/sd_card_cmd.v ../src/user_source/hdl_source/SD/sd_card_sec_read_write.v ../src/user_source/hdl_source/SD/sd_card_top.v ../src/user_source/hdl_source/SD/sd_sector_adapter.v ../src/user_source/hdl_source/SD/fat32_volume_scanner.v ../src/user_source/hdl_source/SD/fat32_file_streamer.v ../src/user_source/hdl_source/SD/sd_sector_arbiter.v ../src/user_source/hdl_source/SD/wav_sector_feeder.v ../src/user_source/hdl_source/hdmi_audio/wav_progress.v ../src/user_source/hdl_source/SD/bmp24_decoder.v ../src/user_source/hdl_source/SD/sd_card_bmp.v sd_spi_card_model.v tb_stageC_preload.v
vsim -c -quiet work_c.tb_stageC_preload
run -all
quit -f
