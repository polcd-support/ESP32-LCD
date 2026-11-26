# P040B019-IPS-MIPI

3.97 寸 BOE IPS 屏（P040B019-MIPI-CTP，ST7701P，480x800，MIPI DSI）显示实验。

- 平台：正点原子 ESP32-P4 开发板（ESP-IDF 5.4.x，target `esp32p4`）
- 转接板：`../P024BM057-IPS-CTP`（P040B019 的 FPC 定义与之逐脚一致，直接插）
- 代码基线：`../P024BM057-IPS-MIPI`（2.4 寸 ST7701P，480x640）
- 面板初始化序列来源：`ST7701P+BOE3.97_IPS MIPI 20260110.txt`（屏厂给的 C 数组格式）
- 时序来源：`rk3568-kickpi-mipi0-P040B019-IPS-CTP.dtsi`（RK3568 上已点亮的参数）

## 面板参数

| 项目 | 值 |
| --- | --- |
| 分辨率 | 480 x 800（竖屏） |
| 驱动IC | ST7701P |
| 触摸IC | FT6336U（本工程未驱动） |
| 像素格式 | RGB888（DPI `LCD_COLOR_PIXEL_FORMAT_RGB888`） |
| PCLK | 27 MHz（面板值）→ 本工程实际 30 MHz（240/8，唯一可精确得到的档位） |
| Hsync / HBP / HAdr / HFP | 10 / 30 / 480 / 30 |
| Vsync / VBP / VAdr / VFP | 4 / 15 / 800 / 12 |
| 帧率 | 27MHz 规格值 59.1 Hz；本工程 30MHz 实际 65.6 Hz |
| MIPI 数据通道 | 2 lane（CLK + D0 + D1） |
| 通道比特率 | 600 Mbps（30MHz x 24bit = 720Mbps，2 通道各 360Mbps；600 让 TxClkEsc=18.75MHz 落在 2~20MHz 规格内，且 `600/30/8` 为整数） |
| PHY 供电 | LDO_VO3 = 2500 mV → VDD_MIPI_DPHY |
| 复位 / 背光 | GPIO52（LCD_RST）/ GPIO53（LCD_BL，高电平点亮，经 S8050 驱动 LED_K） |

面板 FPC（26P，与 P024BM057 转接板 FPC2 完全对应）：

| 脚 | 功能 | 脚 | 功能 |
| --- | --- | --- | --- |
| 1 / 2 | D0P / D0N | 16 / 17 | IOVCC（本板直接接 +1V8，LDO2 ME6211C18） |
| 4 / 5 | D1P / D1N | 18 | RESET |
| 7 / 8 | CLKP / CLKN | 19 / 20 | VCI（2.8V 典型，2.45~3.3V） |
| 21 | TE | 22 | LEDK |
| 24 | LEDA | 10/11/13/14/23 | NC |

背光：8 颗 LED 并联，If 典型 120mA / 最大 160mA，Vf 3.2V。

## PCLK 为什么是 30 MHz（关键，不能填 27）

ESP32-P4 的 DPI 时钟只能从 `XTAL / PLL_F160M / PLL_F240M` 里选，且**分频是整数截断**：

```c
// hal/mipi_dsi_hal.c
div = clk_src_mhz / expect_dpi_clk_mhz;          // 240/请求值
hal->dpi_clock_freq_mhz = clk_src_mhz / div;     // 截断成整数 MHz
```

而 host 侧行时序是拿这个**整数 MHz**去换算成 “lane 字节时钟” 周期的，bridge 侧却直接用像素数：

```c
float dpi2lane_clk_ratio = lane_bit_rate_mbps / dpi_clock_freq_mhz / 8;  // host 侧缩放
mipi_dsi_host_ll_dpi_set_horizontal_timing(hsw*ratio, hbp*ratio, active*ratio, hfp*ratio);
mipi_dsi_brg_ll_set_horizontal_timing(hsw, hbp, active, hfp);            // bridge 侧不缩放
```

所以**请求值必须让 `240/div` 严格等于实际像素时钟**，否则 host 认为的行长和 bridge 实际行长不一致，
bridge 的像素流填不进去 → FIFO 溢出丢像素 → 横竖条纹/花屏。

| 请求 | div | 实际 PCLK | `dpi_clock_freq_mhz` | 缩放比 600/clk/8 | 结果 |
| --- | --- | --- | --- | --- | --- |
| 27 | 8 | 30.00 MHz | 30 | 2.50（整数） | 可用，65.6 Hz |
| 26 | 9 | 26.67 MHz | **26**（截断） | 2.885（**+2.6%**） | **host/bridge 失配 → 花屏** |
| 24 | 10 | 24.00 MHz | 24 | 3.125 | 可用但只有 52.5 Hz，偏慢 |

`MIPI_LCD_PCLK_MHZ = 30`（请求 27 也行，结果同样落到 240/8）。`mipi_lcd.c` 顶部加了 `_Static_assert`
把这个约束固化下来，防止以后改成 26/25/27.5 这类值。

> 旧工程 480x640 请求 22 → div 10 → 24MHz 恰好是整数，所以没踩这个坑；它 README 里也记了
> “21MHz 会得到 21.8 但 HAL 按 21 换算，偏差 3.8%，故不用”，是同一件事。

若 65.6Hz 出现闪烁（面板标称 60Hz），把 `MIPI_LCD_VFP` 从 12 加到 104（V 总行 923 → 59.0Hz），
像素时钟仍是 30MHz 整数分频，比值不变。

## 与 P024BM057-IPS-MIPI 的差异

1. **分辨率/时序**：640 → 800 行；`MIPI_LCD_HEIGHT` 800、`MIPI_LCD_PCLK_MHZ`(22→30，因整数分频约束) 更新，
   HSW/HBP/HFP/VSW/VBP/VFP 均为 10/30/30/4/15/12，与新老屏相同。
2. **初始化序列**：整段替换为 P040B019 的厂序列（`st7701p_init_cmd_head/tail[]`），
   与 `rk3568-...P040B019-IPS-CTP.dtsi` 的 `panel-init-sequence` 逐条一致。
   新版序列末尾多一条 `0x35 00`（TEON），`0xD0 0x88` 之后不再有 100ms 延时。
3. **代码结构与驱动接口不变**：`lcd.c` 的 RGB565→RGB888 双缓冲、`LCD_DEMO()` 显示流程原样保留。

### 初始化序列核对（脚本逐字节比对，2026-11-26）

把 `st7701p_init_cmd_head[] + SLPOUT + st7701p_init_cmd_tail[]` 展开成 (命令, 参数, 延时) 序列，
与 `ST7701P+BOE3.97_IPS MIPI 20260110.txt`（51 条含延时）逐项比较：**命令号、参数、长度全部一致**，
仅 `0x11 SLPOUT` / `0x29 DISPON` 两条 DCS 无参命令用短包发送（厂文本/dtsi 里带 1 个 dummy 参数 0x00），
与旧工程做法相同。同一份 C 表与 dtsi 的 `panel-init-sequence` 也比较过：**命令与参数完全一致**。

延时（与 RK dtsi 一致，按“已在 RK3568 上点亮的配置”取值）：

| 位置 | dtsi | 厂文本 | 本工程 |
| --- | --- | --- | --- |
| SLPOUT 0x11 后 | 0xFF(=255ms) | 120ms | 255ms |
| `E8 00 0C` 后 | 0xFF | 10ms | 255ms |
| DISPON 0x29 后 | 0xFF | 20ms | 255ms |

dtsi 里的 `0xFF` 是 RK 平台“等到够”的写法。厂文本的 10ms 是原始代码里的值，但 RK 配置（本面板已实际点亮）
用的是 255ms，两者冲突时以已验证的 RK 值为准；多等的时间只影响开机约 0.7s，没有副作用。

`0x36 MADCTL` / `0x3A COLMOD(0x77)` 仍在厂序列之前（reset 之后）发送，沿用旧工程顺序；
面板厂序列本身不含这两条。若出现镜像或 565 色偏，先查这里。

## 上板记录（2026-11-26）：横竖条纹的根因是面板不回 ACK

现象：新屏点亮后满屏横竖条纹/线条，完全不可用；旧屏 P024BM057 在同一块板上正常。

串口日志（修复前）：

```
I mipi_lcd: link idle @after reset: PHY_STATUS=0x000015BD (clk_stop=1 d0_stop=1 d1_stop=1)
E mipi_lcd: link stuck before cmd 0x3A: PHY_STATUS=0x000015AF (clk_stop=1 d0_stop=0 d1_stop=1)
E mipi_lcd: panel init failed: ESP_ERR_TIMEOUT, skip DPI setup
E lcd_demo: panel init failed, stay here with backlight on
```

`d0_stop=0` 表示 host 发完第一条命令（`0x36 MADCTL`）后做了总线周转（BTA）在等面板应答，
面板不应答，链路就永远停在 HS 态。**初始化一条命令都没执行成功**，面板处于未配置状态，屏幕上就是条纹。

根因：`CMD_MODE_CFG.ACK_RQST_EN`（bit[1]）为 1，上电默认值读出来是 `0x010F7F02`。
每条 DCS 写都会请求 ACK 并等 BTA，本屏不返回 ACK（不是没供电：背光/画面都在，说明 VDDI/VCI 正常）。

修复（`mipi_lcd.c`，创建 DBI IO 之后、发任何命令之前）：

```c
REG_CLR_BIT(DSI_HOST_CMD_MODE_CFG_REG, 1 << 1);   // bit[1] = ack_rqst_en
```

副作用：不能再发 DCS 读命令（0xDA/0x0A 等），本工程不需要读 ID。
修复后日志走到 `DPI panel started, video streaming`，画面正常。

补充：ST7701S 规格书 7.5.5 节要求复位释放到可下命令的间隔 **TRT ≥ 120ms**，本工程 reset 后等 100ms
（旧工程同值，实测可用）。若冷启动偶发“首条命令丢失/画面异常”，把 `mipi_lcd_panelreset()` 里的
第二个 100ms 改成 200ms 即可。

## 触摸（未实现）

面板带 FT6336U（I2C 地址 0x38），转接板有 TP_SCL/TP_SDA/TP_INT/TP_RST（6P FPC1）。
本工程**不含触摸代码**：RGB 屏显示链路与触摸无关，且不同板子的 I2C 脚位不同。
需要时按 `focaltech_ft6336u_driver.patch` + `rk3568-...dtsi` 里的 `focaltech,*` 参数接入
（reset-gpio 低有效、irq 低有效、`display-coords = 0 0 480 800`、`max-touch-number = 5`）。

## 目录

```
P040B019-IPS-MIPI/
├── CMakeLists.txt                 # 工程名 P040B019_IPS_MIPI
├── sdkconfig                      # 由 P024BM057 工程移植（esp32p4 / 16MiB flash / 自定义分区表）
├── partitions-16MiB.csv
├── components/BSP/
│   ├── CMakeLists.txt
│   └── LCD/
│       ├── mipi_lcd.c/.h          # ST7701P + DSI/DPI 初始化，480x800 时序
│       ├── lcd.c/.h               # 绘图/字符封装（RGB565 → RGB888 双缓冲）
│       └── lcdfont.h
└── main/
    ├── main.c
    ├── lcd_demo.c/.h              # 显示验证流程（参数页、纯色、彩条、16 级灰阶）
```

## 编译 / 烧录

```bash
# 已安装 ESP-IDF（本文验证用 F:/idf/v5.4.2/esp-idf，工具链 F:/idf/tools）
idf.py -B C:/p4build_p040 build      # 构建目录放 ASCII 路径
idf.py -B C:/p4build_p040 -p COMx flash monitor
```

**Windows 上必须把构建目录放到 ASCII 路径**：工程源码路径含中文（`正点原子P4-MIPI转接板`），
而 `ldgen` 会把库路径交给 `riscv32-esp-elf-objdump.exe`，objdump 按本地代码页解析 UTF-8 路径，
在默认的 `<工程>/build` 目录下会报 `libriscv.a: No such file or directory`。用 `-B` 指定纯 ASCII 的
构建目录即可，源码留在中文路径不影响。

串口应打印：

```
I (xxx) mipi_lcd: MIPI DSI PHY power on: LDO_VO3 = 2500mV
I (xxx) mipi_lcd: DSI bus ready: 2 lane @ 600 Mbps
I (xxx) mipi_lcd: panel: ST7701P 480x800, timing hsw=10 hbp=30 hfp=30 vsw=4 vbp=15 vfp=12 pclk=26MHz
I (xxx) lcd_demo: LCD 480x800 ready
```

## 上板注意

- **IOVCC 固定 1.8V**：本版转接板 FPC2 的 16/17 脚直接接 LDO2（ME6211C18）输出的 +1V8，不是可贴选项
  （板上只有 R3/R4/R5/R6/R7，没有 3V3 备选）。P040B019 的 VDDI 范围 1.65~3.3V，1.8V 在规格内。
- **黑屏先查供电**：面板 VDDI/VCI 不到位时不回 ACK，`esp_lcd_new_panel_io_dbi()` 默认
  `ACK_RQST_EN=1`，初始化会卡在第一条命令（task_wdt），链路状态为 `PHY_STATUS=0x15AF`（d0_stop=0）。
  排查细节见 `../P024BM057-IPS-MIPI/README.md`。
- **背光电流**：面板 8 灯并联，若 3V3 直驱（无恒流/串阻）需确认 S8050 与限流电阻能承受 ~120mA。

## 风险点

- **帧率 65.6Hz（面板标称 60Hz）**：像素时钟 30MHz 是 240MHz 下唯一能整除且 ≥27MHz 的档位；若出现闪烁，
  把 `MIPI_LCD_VFP` 12 → 104（V 总 923 → 59.0Hz），时长与比值都不受影响。
- **初始化序列延时**：已按 RK dtsi 取 255ms（三处），不再是等待短板。
- **背光电流**：适配板 LEDA 直接接 +3V3，LEDK 经 R4=10Ω 到 S8050 拉地；本屏 Vf 典型 3.2V / 最大 3.3V、
  8 灯并联 If 120mA，3.3V 供电下 10Ω 只能给出几 mA~几十 mA，**偏暗属硬件预期**。要满亮度需升高 LEDA 电压或改恒流。
- **触摸未实现**：FT6336U 需要按实际板子的 I2C 引脚另加驱动。
