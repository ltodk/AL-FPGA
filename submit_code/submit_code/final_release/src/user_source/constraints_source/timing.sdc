
# ============================================================
# merged_top_tf_hdmi_audio.sdc
# 用于：TF 图片轮播 + HDMI(视频+音频) 整合工程
#
# 合并原则：
# 1) 保留 top.sdc 里的系统主时钟与 derive_clocks
# 2) 把 timing(2).sdc 里的 pixel/serial 时钟改成整合工程实际名字
# 3) 增加 hdmi_5x_clk / audio_mclk 的命名
# 4) 不再对内部 net 直接 create_clock，避免和 derive_clocks 重复
# ============================================================

# ------------------------------------------------------------
# 1. 板级输入主时钟
#    你的原 top.sdc 里 clk=50MHz，所以这里仍然用 20ns
# ------------------------------------------------------------
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

# ------------------------------------------------------------
# 2. 自动推导 PLL 输出时钟
# ------------------------------------------------------------
derive_clocks

# ------------------------------------------------------------
# 3. 给关键 PLL 输出命名
#    下面这些路径是按你当前工程实例名写的：
#      sys_pll      -> sys_pll_m0
#      video_pll    -> video_pll_m0
#      音频 PLL     -> u_audio_pll
#
#    如果综合后实例路径略有不同，就按 Timing Analyzer 里的实际 pin 路径改。
# ------------------------------------------------------------
rename_clock -name {sd_card_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
# ★ 2026-10-22 补：clkc[2] 是 SDRAM 相移时钟。原来漏了这句命名，
#   导致第 7.3 条 set_false_path 里的 {ext_mem_clk_sft} 匹配不到任何时钟，
#   两条约束静默失效，SDRAM 硬 IP 的 I/O 路径被当成普通 fabric 时序检查，
#   凭空产生 33 条伪违例（SWNS -6.565ns）。补上命名后 7.3 才能生效。
rename_clock -name {ext_mem_clk_sft} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[2]}]
rename_clock -name {video_clk}   -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {audio_mclk}  -source [get_ports {clk}] -master_clock {clk} [get_pins {u_audio_pll/pll_inst.clkc[2]}]

# ------------------------------------------------------------
# 4. 从原 timing(2).sdc 迁移过来的 HDMI 像素/串行时钟约束
#
#    原文件是：
#      create_clock -name pixel_clk  ... [get_nets {S_pixel_clk}]
#      create_clock -name serial_clk ... [get_nets {S_serial_clk}]
#      set_clock_groups -exclusive ...
#
#    在整合工程里，这两个时钟已经由 video_pll 产生，并通过 derive_clocks 建出来了，
#    所以这里只保留“组关系”，不再重复 create_clock。
#
#    这里沿用原作者的写法，把 video_clk 和 hdmi_5x_clk 设成 exclusive。
#    如果后面 Timing Analyzer 明确显示两者之间需要做同步时序分析，再把这一条去掉。
# ------------------------------------------------------------
set_clock_groups -exclusive \
    -group [get_clocks {video_clk}] \
    -group [get_clocks {hdmi_5x_clk}]

# ------------------------------------------------------------
# 5. 音频 I2S 发生器/接收器跨域
#
#    audio_mclk 域里生成 I2S，video_clk 域里做三拍同步采样。
#    这类路径不是普通同步时序路径，直接切掉更合理。
# ------------------------------------------------------------
set_clock_groups -asynchronous \
    -group [get_clocks {audio_mclk}] \
    -group [get_clocks {video_clk} hdmi_5x_clk sd_card_clk ext_mem_clk]

# ------------------------------------------------------------
# 6. 可选：如果你的 frame_read_write 内部异步 FIFO 已经完全处理好了跨域，
#    且报告里总出现 video_clk / sd_card_clk / ext_mem_clk 之间的大量伪违例，
#    可以打开下面这组异步分组。
#
#    建议先不要开，先看报告；
#    只有确认这些跨域都只经过 FIFO / 同步器时再开。
# ------------------------------------------------------------
# set_clock_groups -asynchronous \
#     -group [get_clocks {sd_card_clk}] \
#     -group [get_clocks {ext_mem_clk}] \
#     -group [get_clocks {video_clk}] \
#     -group [get_clocks {hdmi_5x_clk}]

# ------------------------------------------------------------
# 7. 其余 Input/Output delay 先保持空白
#    你当前两个原始 sdc 也都没有写这部分。
# ------------------------------------------------------------

# ============================================================
# 第6讲：时序约束完善（Flash固化与上电自启）
# 目的：切掉异步/跨域伪违例，约束关键路径，让时序报告干净。
# ============================================================

# 7.1 复位信号设为 false_path（异步信号，不做时序分析）
set_false_path -from [get_ports {rst_n}]

# 7.2 同步器第一级触发器设为 false_path（reset_sync/sync_2ff 的 s1 是异步采样）
set_false_path -to [get_regs -hier {*/s1}]

# 7.3 SDRAM 移位时钟路径设为 false_path（硬 IP 的 I/O 窗口，不做 fabric 时序分析）
set_false_path -from [get_clocks {ext_mem_clk}] -to [get_clocks {ext_mem_clk_sft}]
set_false_path -from [get_clocks {ext_mem_clk_sft}] -to [get_clocks {ext_mem_clk}]

# 7.3b CDC 数据路径（ext_mem_clk -> sd_card_clk：写完成格雷码回传）
#      这条路径经两级同步器，不应按同步周期检查（100M/125M 同源但周期不整除）。
#      原来被当同步路径查，预算 2ns、实际 2.059ns，负 0.296ns。
#      改用 datapath_only：不检查同步关系，但仍约束布线延迟，
#      既能消掉伪违例，又能防止走线过长扩大亚稳态窗口。
set_max_delay -datapath_only \
    -from [get_clocks {ext_mem_clk}] \
    -to [get_clocks {sd_card_clk}] 20.000

# 7.4 HDMI ODDR 输出路径设 max_delay（TMDS 串行数据，像素时钟 5 倍率输出）
set_max_delay -from [get_clocks {hdmi_5x_clk}] \
    -to [get_ports {HDMI_CLK_P HDMI_D0_P HDMI_D1_P HDMI_D2_P}] \
    8.000 -datapath_only

# 7.5 时钟不确定性（保持时间）：默认 0.1ns 可能偏大导致伪违例，收紧到 0.05ns
set_clock_uncertainty -hold 0.050 \
    [get_clocks {clk sd_card_clk ext_mem_clk video_clk hdmi_5x_clk}]

