vlib work_arb
vlog -sv -work work_arb ../src/user_source/hdl_source/SD/media_sdram_write_cdc.v tb_wr_arbiter.v
vsim -c work_arb.tb_wr_arbiter
run -all
quit -f
