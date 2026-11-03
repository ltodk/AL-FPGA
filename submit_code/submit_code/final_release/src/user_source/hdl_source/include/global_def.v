


`define   DATA_WIDTH                        32
`define   ADDR_WIDTH                        21
`define   DM_WIDTH                          4

`define   ROW_WIDTH                        11
`define   BA_WIDTH                        2

`define	  SDR_CLK_PERIOD				1000000000/125000000
// ===== 【2026-10-28 噪点排查小结】=====
// · 试过 125MHz + 相位 180°/135°/90°：只影响"静止图多久开始退化"
//   （180°≈60s、135°≈180s、90°≥5min），切图后必花。
// · 试过 125→100MHz（DIV 8→10）：**实测更差，已回滚** ⇒ 不是时间余量问题。
// · 决定性线索：噪点**按行分层**（一张图上半段紫、下半段浅绿+粉棕），
//   且颜色精确对应 DQ15(G bit7) / DQ14(G bit6) / DQ22(R bit6) 单比特翻转。
//   显示是逐行扫描 ⇒ "按行分层"= 时间周期性 ⇒ 读命令撞上 SDRAM 内部刷新窗口
//   （刷新 3125 拍 ≈ 31.25µs，行周期 32µs，两者以 ~1ms 节奏互相滑过）。
// ⇒ 正解：把 Sdr_busy / Sdr_init_ref_vld 接进 frame_fifo_read 的 burst 发起条件。
//   刷新间隔必须与上面 SDR_CLK_PERIOD 保持一致，否则刷新率会跟着错。
`define   SELF_REFRESH_INTERVAL			64000000/`SDR_CLK_PERIOD/2**(`ROW_WIDTH)

