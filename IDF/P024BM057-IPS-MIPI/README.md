# P024BM057-IPS-MIPI

2.4 寸 BOE IPS 屏（ST7701P，480x640，MIPI DSI）显示实验。

- 平台：正点原子 ESP32-P4 开发板（ESP-IDF 5.4.x，target `esp32p4`）
- 转接板：`../P024BM057-IPS-CTP`（26P 面板 FPC + MIPI 2 lane + 1.8V/3.3V 电平/供电）
- 代码基线：`ESP32-LCD/IDF/P028TRM172-IPS-MIPI`（2.83 寸 ST7701S，480x640）
- 面板初始化序列来源：`../2.4BOE+ST7701P MIPI.txt`
- RGB 时序来源：面板厂 RGB Output Timing 图（本仓库附带的那张）

## 面板参数

| 项目 | 值 |
| --- | --- |
| 分辨率 | 480 x 640（竖屏） |
| 驱动IC | ST7701P |
| 像素格式 | RGB888（DPI `LCD_COLOR_PIXEL_FORMAT_RGB888`） |
| PCLK | 22 MHz |
| Hsync / HBP / HAdr / HFP | 10 / 30 / 480 / 30 |
| Vsync / VBP / VAdr / VFP | 4 / 15 / 640 / 12 |
| MIPI 数据通道 | 2 lane（CLK + D0 + D1） |
| 通道比特率 | 600 Mbps（22MHz x 24bit = 528Mbps，2 通道均分 264Mbps；600 让 TxClkEsc=18.75MHz 进 2~20MHz 规格） |
| PHY 供电 | LDO_VO3 = 2500 mV → VDD_MIPI_DPHY（IDF 示例值；1.9V 实测现象相同） |
| 复位 / 背光 | GPIO52（LCD_RST）/ GPIO53（LCD_BL，高电平点亮，经 S8050 驱动 LED_K） |

## 与 P028TRM172-IPS-MIPI 的差异

1. **驱动IC**：ST7701S → ST7701P，初始化序列全部替换为 `2.4BOE+ST7701P MIPI.txt` 的内容。
   该序列分三段发送：
   - `st7701p_init_cmd_head[]`：CMD2 页（`FF 77 01 00 00 13/10/11`）的电源、GIP、伽马参数；
   - `0x11`（SLPOUT）+ 120ms；
   - `st7701p_init_cmd_tail[]`：睡眠退出后必须补发的 `E8 00 0C`（20ms）→ `E8 00 00`，最后 `0x29` 开显示。

   顺序细节：`0x36`（MADCTL）/ `0x3A`（COLMOD，RGB888 即 0x77）放在整段厂序列**之前**（reset 之后）发送，
   厂序列本身与 `2.4BOE+ST7701P MIPI.txt` 逐条一致；`0x29` 只在表尾发一次。
2. **单面板**：删除了 HX8394/HX8399/ILI9881C/JD9855 的 ID 探测与多分支，时序常量直接固化在
   `mipi_lcd.h` 的 `MIPI_LCD_*` 宏里，读屏 ID（`0xDA/0xDB`）不再执行。
3. **时序**：与基线一致（同为 480x640@22MHz、H 10/30/30、V 4/15/12），故配电时序宏值相同。
4. **示例**：去掉 logo 图和触摸手写页面，改为纯显示验证（参数页、纯色、彩条、16 级灰阶循环）。

## 触摸

转接板有 TP_SCL/TP_SDA/TP_INT/TP_RST（6P FPC），但面板规格书未提供，CTP 控制器型号未知，
因此本工程**不含触摸代码**。确认控制器型号后可从基线工程拷 `components/BSP/TOUCH` 对应驱动
（FT6336 / CST816）并在 `LCD_DEMO()` 里初始化即可。

## 目录

```
P024BM057-IPS-MIPI/
├── CMakeLists.txt                 # 工程名 P024BM057_IPS_MIPI
├── sdkconfig                      # 由基线工程移植（esp32p4 / 16MiB flash / 自定义分区表）
├── partitions-16MiB.csv
├── components/BSP/
│   ├── CMakeLists.txt
│   └── LCD/
│       ├── mipi_lcd.c/.h          # ST7701P + DSI/DPI 初始化，480x640 时序
│       ├── lcd.c/.h               # 绘图/字符封装（RGB565 → RGB888 双缓冲）
│       └── lcdfont.h
└── main/
    ├── main.c
    ├── lcd_demo.c/.h              # 显示验证流程
```

## 编译 / 烧录

```bash
# 已安装 ESP-IDF（本文验证用 F:/idf/v5.4.2/esp-idf，工具链 F:/idf/tools）
idf.py -B C:/p4build_p024 build      # 见下方说明：构建目录放 ASCII 路径
idf.py -B C:/p4build_p024 -p COMx flash monitor
```

**Windows 上必须把构建目录放到 ASCII 路径**：工程源码路径含中文（`正点原子P4-MIPI转接板`），
而 `ldgen` 会把库路径交给 `riscv32-esp-elf-objdump.exe`，objdump 按本地代码页解析 UTF-8 路径，
在默认的 `<工程>/build` 目录下会报 `libriscv.a: No such file or directory`（构建失败在此步，
与驱动代码无关）。用 `-B` 指定纯 ASCII 的构建目录即可，源码留在中文路径不影响。

串口应打印：

```
I (xxx) mipi_lcd: MIPI DSI PHY power on: LDO_VO3 = 1900mV
I (xxx) mipi_lcd: panel: ST7701P 480x640, timing hsw=10 hbp=30 hfp=30 vsw=4 vbp=15 vfp=12 pclk=22MHz
I (xxx) lcd_demo: LCD 480x640 ready
```

实测构建输出（ESP-IDF v5.4.3 / EIM 环境，esp32p4）：`P024BM057_IPS_MIPI.bin` 0x4A260 字节，无本工程源码告警。

## 上板调试记录(2026-11-20)

### 根因（硬件：面板 VDDI 1.8V 未到位）

黑屏是**面板没供电**导致的，不是时序/初始化问题。修复供电后同一份固件立刻正常。

判定证据（`mipi_lcd_probe_read()` 用 DCS 读命令探测，读命令必须由面板驱动 D0 回数据）：

| 状态 | probe 0xDA（Read ID1） | 现象 |
| --- | --- | --- |
| VDDI 未到位 | `wait=50000us got=0`，`PHY_STATUS=0x000015AF`（D0 掉出 LP-11，host 卡在 BTA） | 初始化卡在第一条命令，task_wdt |
| VDDI 正常 | `wait=0us got=4 data=FF FF FF`，`PHY_STATUS=0x000015AD` | 初始化全流程通过，视频流正常 |

连带现象：面板无电时不回 ACK，而 `esp_lcd_new_panel_io_dbi()` 默认打开 `CMD_MODE_CFG.ACK_RQST_EN`，
host 每发一条 DCS 命令都会在 D0 上做总线周转等 ACK，于是永久卡死。供电修复后 **ACK 恢复正常，IDF 默认配置即可**，
调试期"清 ACK_RQST_EN"的补丁已删除（副作用是 DCS 读命令不可用，没必要保留）。
当时还验证过：只补开 `PCKHDL_CFG.BTA_EN` 而保留 ACK 无效（`0x00000019 -> 0x0000001D`，首包后仍 `0x15AF`），
说明瓶颈是面板不应答，不是 BTA 使能。

对照适配板（`../P024BM057-IPS-CTP`）：FPC2 的 16/17 脚是面板 VDDI，R1（3V3，0Ω）与 R2（1V8，0Ω）
都落在同一个网络上——两者是二选一的装配选项，**同时贴会直接把 3.3V 与 1.8V 短路**。

### 初始化序列与面板厂文本的逐条核对

用脚本把 `../2.4BOE+ST7701P MIPI.txt`（43 条，含 `Delay()`）和 `st7701p_init_cmd_head/tail[]` 逐字节比对：
**命令号、参数、延迟全部一致**（`D0=0x88` 后 100ms、SLPOUT 后 120ms、`E8=00 0C` 后 20ms、DISPON 后 20ms）。
另有 3 处顺序问题已修正：

1. `0x36 MADCTL / 0x3A COLMOD` 原来插在 SLPOUT 之后、page13 `E8 00 0E/0C` 之前，打断了厂序列的睡眠退出流程；
   现改为在厂序列开始前（reset 之后）发送，从此处起整段与厂文本逐条一致。
2. `0x29 DISPON` 在表内发一次、`mipi_lcd_init()` 里又调用一次 `esp_lcd_panel_disp_on_off(true)`，重复；已删除后者。
3. 调试期临时插入的 `0x36` 短包探针已删除。

PCLK 请求值保持 22MHz：DPI 源 240MHz 为整数分频（`div = src/request`），实际输出 24MHz（帧率 65Hz）。
22MHz 在 240MHz 下除不尽；若改请求 21MHz 会得到 21.8MHz（59.1Hz，更接近面板规格），
但 HAL 会用截断后的 21MHz 换算 host 侧行时序，与 bridge 实际 21.8MHz 产生约 3.8% 偏差，故不用。

### 排查过程（保留记录）


烧录后卡在 `esp_lcd_panel_init()` 里，表现为 task_wdt 反复触发、背光不亮。
用 USB-JTAG(openocd + gdb, 读 `g_lcd_stage` / DSI 寄存器) 抓到的证据：

| 观测 | 值 | 含义 |
| --- | --- | --- |
| `g_lcd_stage` (0x4FF16588) | `0x64` | 卡在 `st7701p_init_cmd_head[]` 内，tail 未执行 |
| `DSI_HOST_PHY_STATUS` (0x500A00B0) | `0x15AF` | `PHY_LOCK=1`、`STOPSTATECLKLANE=1`、`STOPSTATE1LANE=1`，**`STOPSTATE0LANE=0`** |
| `DSI_HOST_CMD_PKT_STATUS` (0x500A0074) | `0x12` | `GEN_CMD_FULL=1`：包头排队却发不出去 |
| `DSI_HOST_GEN_HDR` (0x500A006C) | `0x0078C115` | 最后写进去的包头 `0xC1,0x78` |

加上链路自检后（每发一条命令前检查 stop state，50ms 超时）日志明确到**第一条命令之后就堵住**，
而且与包长无关：先试一条 1 参数短包 `0x36 0x00`，D0 同样立刻掉出 stop state：

```
I mipi_lcd: link idle @bus init:  PHY_STATUS=0x000015BD (clk_stop=1 d0_stop=1 d1_stop=1)
I mipi_lcd: link idle @after reset: PHY_STATUS=0x000015BD (clk_stop=1 d0_stop=1 d1_stop=1)
I mipi_lcd: link BUSY @after short write 0x36: PHY_STATUS=0x000015AF (clk_stop=1 d0_stop=0 d1_stop=1)
```

DSI 的 LP 命令只在 **lane0** 上发送，所以正好只有 D0 受影响；时钟 lane 和 D1 一直正常。
把通道比特率从 500 改成 600Mbps（TxClkEsc 由 20.83MHz 降到 18.75MHz，落在 D-PHY 2~20MHz 规格内）
现象完全不变；把 VDD_MIPI_DPHY 从 1.9V 改到 2.5V（IDF 官方示例值）也一样。
短包/长包、500/600Mbps、1.9/2.5V 四种组合全部复现同一现象 → 不是速率、不是供电、不是包格式、不是初始化序列。

原始卡死形态：

```
I mipi_lcd: panel reset done
E mipi_lcd: link stuck before cmd 0xEF: PHY_STATUS=0x000015AF (clk_stop=1 d0_stop=0 d1_stop=1)
E mipi_lcd: panel init failed: ESP_ERR_TIMEOUT, skip DPI setup
E lcd: MIPI panel init failed, backlight kept on for hardware check
```

即首发 `0xFF 77 01 00 00 13`（长包）后 D0 差分对再也回不到 LP-11，DSI 主机的发送状态机停住；
IDF 的 `mipi_dsi_hal_host_gen_write_dcs_command()` 在 FIFO 满时是 `while(...)` 死等、无超时，
所以会整机卡死（现在已用 `mipi_lcd_wait_link_idle()` 拦住，失败后开背光并停在报错循环里，不再重启）。

**结论：软件侧没问题（前序命令/时钟 lane/1 号数据 lane 都正常），是 D0 这一对差分对被外部钳住。**

面板 26P 引脚对照（本工程转接板 vs 正点原子已知可用的 2.83" 面板 FPC）：

| 功能 | 本转接板 FPC2 | 正点原子 2.83" MIPI 面板 FPC |
| --- | --- | --- |
| D0N / D0P | 2 / 1 | 10 / 11 |
| CLKN / CLKP | 8 / 7 | 13 / 14 |
| LED_A | +3V3(19/20/24) | 1 / 2 |
| LED_K | 22 | 5 / 6 |
| LCD_RST | 18 | 7 |
| TP_* | 6P FPC1 | 21..24 |

本转接板把 `D0P/D0N` 放在 FPC2 的 1/2 脚；若 P024BM057 面板的 1/2 脚同样是 LED_A（正点原子这几款屏都是这样排），
就等价于把 DSI 数据对直接接到了背光 LED 上 —— 与 `STOPSTATE0LANE=0` 的现象完全吻合。
需要拿面板 FPC 图纸核一遍再改板/飞线。

`g_lcd_stage` 是本工程内置的进度变量，卡死时可不开串口直接读：

```bash
openocd -f board/esp32p4-builtin.cfg -c "init" -c "halt" -c "mdw 0x4FF16588 1" -c "shutdown"
```

## 风险点

- **MADCTL/COLMOD 位置**：沿用基线“SLPOUT 之后、DISPON 之前”的顺序；面板默认 MADCTL=0x00、COLMOD 需 0x77，
  若初版出现镜像或 565 色偏，先查这两个命令与面板初始化序列的先后关系。
- **通道比特率 500Mbps**：面板规格书未给出 DSI 速率上限，取同分辨率 ST7701S 基线值；若出现花屏/闪烁，
  先确认面板支持的最高 lane rate 再下调。
