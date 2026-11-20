# P028B119 固件汇总

ESP32-S3 + ST7701S（BOE 2.83" 480×640，RGB565 16bit，无触摸）

固件按 **三个分片** 交付，烧录地址如下。

---

## 1. 固件分片（当前版本：16 MHz）

| 固件 | 文件 | 大小 | 烧录地址 |
|---|---|---|---|
| 引导 | `bootloader.bin` | 22,368 B | **0x0** |
| 分区表 | `partition-table.bin` | 3,072 B | **0x8000** |
| 应用 | `LCD_TEST.bin` | 366,768 B | **0x10000** |

Flash 参数：`dio / 80 MHz / 16 MB`
app 分区 1 MB，固件占 0x598b0，剩余 65%。

`merged/` 子目录放的是等价的合并镜像（整片烧录用，非交付主体）：

| 文件 | 说明 |
|---|---|
| `merged/P028B119_ST7701S_480x640_16MHz_merged.bin` | 本次 16 MHz 版合并镜像（= 上面三个分片按偏移拼合，烧 0x0） |
| `merged/P028B119_ST7701S_480x640_no_touch.bin` | 原有基线镜像（2026-09-12，未含本次时序调整） |

---

## 2. 屏幕时序（`main/Inc/lcd.h`）

```c
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (16 * 1000 * 1000) // clk 16
#define EXAMPLE_HSYNC_PULSE_WIDTH 2                   // hpw 2
#define EXAMPLE_HSYNC_BACK_PORCH 8                    // hbp 8
#define EXAMPLE_HSYNC_FRONT_PORCH 40                  // hfp 40
#define EXAMPLE_VSYNC_PULSE_WIDTH 2                   // vpw 2
#define EXAMPLE_VSYNC_BACK_PORCH 8                    // vbp 8
#define EXAMPLE_VSYNC_FRONT_PORCH 12                  // vfp 12
```

- 行总计 2+8+480+40 = **530**，场总计 2+8+640+12 = **662**
- 刷新率 = 16 MHz ÷ (530 × 662) ≈ **45.6 Hz**
- 分频链：`LCD_CLK_SRC_PLL240M`(240 MHz) ÷ N=15 = 16.000 MHz，MO=2 → 整数分频，无小数误差
- 其他：`data_width = 16`（RGB565）、DE 模式、`hsync/vsync_idle_low = 0`、`pclk_active_neg = false`

`main/Src/lcd.c` 中额外开启了 DRAM 反弹缓冲：

```c
.flags.fb_in_psram = true,
.bounce_buffer_size_px = EXAMPLE_LCD_H_RES * 10, // 两块DRAM反弹缓冲，避免LCD DMA实时读PSRAM被取指抢带宽导致整屏抖动
```

---

## 3. 烧录命令（三分片）

### 用 flash_args（在本文件夹内执行，推荐）

```powershell
python -m esptool --chip esp32s3 -p COM26 -b 460800 --before default_reset --after hard_reset `
  write_flash @flash_args.txt
```

`flash_args.txt` 内容：

```
--flash_mode dio --flash_freq 80m --flash_size 16MB
0x0 bootloader.bin
0x8000 partition-table.bin
0x10000 LCD_TEST.bin
```

### 手工指定三个文件

```powershell
python -m esptool --chip esp32s3 -p COM26 -b 460800 --before default_reset --after hard_reset `
  write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB `
  0x0 bootloader.bin 0x8000 partition-table.bin 0x10000 LCD_TEST.bin
```

### 走 EIM 环境（工程目录内）

```powershell
eim --log-file "$env:TEMP\eim.log" run "idf.py -p COM26 flash" v5.4.3
```

### 整片烧录合并镜像（可选）

```powershell
python -m esptool --chip esp32s3 -p COM26 -b 460800 --before default_reset --after hard_reset `
  write_flash 0x0 merged\P028B119_ST7701S_480x640_16MHz_merged.bin
```

> 编译环境：ESP-IDF **v5.4.3**（`C:\esp\.espressif\v5.4.3\esp-idf`），由 EIM 管理。
> 注意 EIM 当前"选中"版本是 v5.5.4，与本工程 `build/` 的配置不一致，务必显式指定 `v5.4.3`。

---

## 4. 端口注意事项

- 这块板的 USB-Serial/JTAG 口 **端口号会变**：本机先后出现过 `COM50`、`COM26`。
  **认板子请认 MAC，不要认端口号。**
- 目标板 MAC（USB-Serial/JTAG）：`e8:3d:c1:f4:61:50`
- 另一块 S3（非本模组）：`28:84:85:55:9f:08`
- 板上另一个 USB 口（`\Device\USBSER001`，如 COM6）连不上芯片，别用。
- 若报 `port is busy`：检查是否有 `IEMonitor` 或 VS Code 串口监视器占着端口。
- `.vscode/settings.json` 里的 `idf.portWin` 仍是失效的 `COM3`，用之前记得改。

---

## 5. PCLK 选型记录（本轮实测）

| PCLK | 刷新率 | 现象 |
|---|---|---|
| 12 MHz | 34.2 Hz | 与面板内部 ~60 Hz 差拍 25.8 Hz，**整屏抖动最严重** |
| **16 MHz** | **45.6 Hz** | **目前最好，定档** |
| 20 MHz | 57.0 Hz | 不如 16 MHz |
| 22 MHz | 59.6 Hz | 帧率最正，但**噪点/花屏**（PCLK 速率超了信号/带宽余量） |

结论：面板对 PCLK 有上界（22 MHz 出噪点），而刷新率又受 PCLK 与消隐共同约束，
往下压会因帧率偏离面板内部 60 Hz 时序而出现差拍抖动。**16 MHz 是当前窗口内的最优点。**
原厂初始化注释里写的 `clk 12/16` 也指向这一档。

### 若还要继续优化（不动 PCLK）

1. `hfp 40→2`、`vfp 12→2`：帧像素降到 492×662 → **49.9 Hz**，缩小差拍
2. 关 `CONFIG_SPIRAM_FETCH_INSTRUCTIONS` / `CONFIG_SPIRAM_XIP_FROM_PSRAM`：把取指从 PSRAM 挪回 flash，给 LCD DMA 让带宽
3. 开 `CONFIG_LCD_RGB_RESTART_IN_VSYNC`：VSYNC 中断里强制重启传输，防永久失步
4. `bounce_buffer_size_px` 加到 `480×20`：吸收更大的 PSRAM 延迟尖峰
5. 硬件：PCLK 串 22~33 Ω、缩短排线、加地线回流
