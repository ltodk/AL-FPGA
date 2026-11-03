# ============================================================================
# run_stage11c.do —— 第11讲 阶段C 回归（本讲收尾）
#
# 三个自包含单元测试，共 70 项检查（实测 70 PASS / 0 FAIL）：
#   ① tb_fat32_scan_multi —— 根目录多扇区扫描                          (24 项)
#        原实现只读根目录第一个扇区（16 个目录项），卡上文件一多
#        MUSIC.PCM 掉出前 16 项就再也找不到。改为读满第一个簇 + 0x00 提前收尾。
#        M1 音频在第 40 项（第 3 扇区）→ 跨扇区必须找到
#        M2 第 5 项是 0x00 → 只读 1 个扇区
#        M3 卡上无音频 → 读满 8 扇区，图片正常、不报错
#        M4 spc=2 簇边界 → 读 2 个扇区，不越界
#
#   ② tb_wav_progress     —— 字节→秒 一次性除法（时分复用）              (26 项)
#        整除/余数边界、0 值、超过 999 截顶、size_stable 门控、来回切换。
#
#   ③ tb_osd_audio        —— OSD 行5 音频状态 + 行6 播放进度             (20 项)
#        AU:NO FILE / AU:READY / AU:PLAY / AU:UNDER 四态与配色；
#        POS 3/6s 数字格式与空白抑制；无卡时两行都隐藏。
#
# 用法（在 sim 目录下）：
#   set PATH=D:\FG_software\modelsim\win64;%PATH%
#   vsim -c -do run_stage11c.do
#
# 注意：与 run_stage11b.do 同样的两个坑 —— do 文件里的 vsim 必须用库名限定，
#       且 testbench 的 $finish 会结束整个批处理会话，所以用 exec 起子进程。
# ============================================================================
vlib work_11c_all
vlog -sv -quiet -work work_11c_all \
    +incdir+../src/user_source/hdl_source/SD \
    ../src/user_source/hdl_source/SD/fat32_volume_scanner.v \
    ../src/user_source/hdl_source/SD/osd_overlay.v \
    ../src/user_source/hdl_source/hdmi_audio/wav_progress.v \
    tb_fat32_scan_multi.v \
    tb_wav_progress.v \
    tb_osd_audio.v

echo "===== 1/3 tb_fat32_scan_multi ====="
exec vsim -c -quiet -work work_11c_all tb_fat32_scan_multi -do "run -all; quit -f"

echo "===== 2/3 tb_wav_progress ====="
exec vsim -c -quiet -work work_11c_all tb_wav_progress -do "run -all; quit -f"

echo "===== 3/3 tb_osd_audio ====="
exec vsim -c -quiet -work work_11c_all tb_osd_audio -do "run -all; quit -f"

echo "===== ALL DONE ====="
quit -f
