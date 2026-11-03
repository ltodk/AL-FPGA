# 时序与资源证据（Timing & Area Evidence）

致评委老师，本目录是可复现的静态时序/资源报告，供核验。

## 一、结论（Anlogic Tang Dynasty 6.2.1，place seed = 2）

| 项目 | 数值 |
|---|---|
| 全局建立时间 **SWNS** | **+0.159 ns**，**0 个建立违例端点**（共 9152 个端点 / 40348 条路径） |
| 全局保持时间 HWNS | -0.061 ns（该器件族在本工具链下的既有偏差，历次版本均在 -0.06 ~ -0.11 ns） |
| clk0 `sd_card_clk` 100 MHz | SWNS **+0.179 ns** |
| clk1 `video_clk` 25 MHz | SWNS +6.875 ns |
| clk2 `ext_mem_clk` 125 MHz | SWNS +0.159 ns |
| clk3 `clk` 50 MHz | SWNS +1.519 ns |
| clk4 `hdmi_5x_clk` 125 MHz | SWNS +5.504 ns |
| 资源 | **LUT 15,882 / 19,600 = 81.03%**、REG 6,396、BRAM 16/64、DSP 9/29、IO 33/188 |
| 生成的 bit | 636,608 字节 |

## 二、怎么复现

TD 6.2.1 的「Phy Opt -> Rerun All」不带种子参数，默认种子下本设计在 100 MHz 域
偶发负余量（实测一次 -0.493 ns / 6 个建立违例，属布局实现差异，bit 同样能生成）。

要复现上表，用工程里的 `tools/DefaultFlow_seed2.tcl`（TD 默认流程脚本的副本，唯一
差别是把布局种子固定为 2）：

```
1) 先用 TD 打开工程跑一次  Phy Opt -> Rerun All（任一结果，目的是生成 _Runs/phy_1）
2) cmd 里：
   cd /d <工程>/src/td_project/HDMI1.4b_Transmitter_v1.0_Runs/phy_1
   <TD安装目录>/bin/td_commands_prompt.exe <工程>/tools/DefaultFlow_seed2.tcl
3) 看 phy_1/final_timing.rpt：SWNS 应为 +0.159 ns、0 个建立违例
```

## 三、说明

- 种子（place seed）是**工具环境参数**，不随工程文件提交，所以附了上面那份流程
  脚本，而不是改工程；除种子外脚本与 Anlogic 自带 `doc/scripts/DefaultFlow.tcl` 一致。
- 功能正确性不依赖种子：已上板验证的交付 bit 在种子 2 下生成，四条素材、轮播、
  缩略图、插播即警报、拔卡恢复等 44 条用例全通过。

## 四、文件

| 文件 | 内容 |
|---|---|
| `final_timing.rpt` | TD 的完整时序报告（含每时钟域明细与违例路径） |
| `phy.area` | 布局后的面积/资源报告 |
