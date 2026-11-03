# ============================================================================
# run_stage12.do —— 第12讲 音频波形可视化 回归
#
# 一个自包含单元测试，共 38 项检查（实测 38 PASS / 0 FAIL）
#
# ★ 2026-10-25 收尾做过「显示侧位宽收窄」（见工程状态文档 18.10 节）：
#   ya/yb/ylo/yhi/peak_height/sample_index 全部从 integer 压到 10 位以内，
#   样本→行偏移改走偏移二进制（sel ^ 8'h80）。改完本 tb **仍 38 项全 PASS**。
#   唯一行为差异是 +/4 的取整方向由「向零」变成 floor，对 -1~-4 差 1 像素；
#   本 tb 检查的 y 值都是 4 的倍数或正数，所以逐项数值不变。：
#   tb_audio_waveform —— audio_waveform_overlay 波形 + 峰值条叠加
#       W1  抽取正确：每 DECIMATE 个样本存 1 个，256 点全查              (7 项)
#       W2  峰值必须包含「未被抽取到」的样本  ★ 修正参考实现的一处低估   (1 项)
#       W3  帧末交换双缓冲（display_bank / write_bank / display_peak）    (4 项)
#       W4  未采满 SAMPLE_COUNT：不交换、保持上一帧                        (4 项)
#       W5  面板外 / 间隙：原样透传                                        (4 项)
#       W6  enable=0 / de=0：原样透传                                      (2 项)
#       W7  面板暗底（亮度减半 7F7F7F）                                    (2 项)
#       W8  中轴线 406060                                                  (1 项)
#       W9  波形像素 ±1 连线的上下沿                                       (4 项)
#       W10 满幅负样本不越界 / 正样本                                      (2 项)
#       W11 峰值条高度与底板                                              (5 项)
#       W12 波形区右边界裁剪                                              (2 项)
#
# 用法（在 sim 目录下）：
#   set PATH=D:\FG_software\modelsim\win64;%PATH%
#   vsim -c -do run_stage12.do
#
# 注意：与 stage11b/11c 同样的坑 —— do 文件里的 vsim 必须用库名限定，
#       且 testbench 的 $finish 会结束整个批处理会话，所以用 exec 起子进程。
#
# 配套的「上板前预览」脚本（不依赖仿真器，纯 Python 按 RTL 逻辑逐像素复现）：
#   python preview_waveform.py
#   输出 sim/preview/preview_waveform_1600Hz.png 与 preview_waveform_6seg.png
# ============================================================================
vlib work_12_all
vlog -sv -quiet -work work_12_all \
    ../src/user_source/hdl_source/hdmi_audio/audio_waveform_overlay.v \
    tb_audio_waveform.v

echo "===== 1/1 tb_audio_waveform ====="
exec vsim -c -quiet -work work_12_all tb_audio_waveform -do "run -all; quit -f"

echo "===== ALL DONE ====="
quit -f
