/**
 ****************************************************************************************************
 * @file        mipi_lcd.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2026-11-20
 * @brief       P024BM057-IPS-CTP(2.4寸 BOE, ST7701P, 480x640) MIPI屏驱动
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32-P4 开发板 + P024BM057-IPS-CTP MIPI转接板
 *
 ****************************************************************************************************
 */

#ifndef __MIPI_LCD_H
#define __MIPI_LCD_H

#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_mipi_dsi.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lcd.h"
#include "esp_check.h"

/* 面板: P024BM057-IPS-CTP, 驱动IC ST7701P, 480x640, RGB888 */
#define MIPI_LCD_WIDTH                  480         /* 面板宽度,单位:像素 */
#define MIPI_LCD_HEIGHT                 640         /* 面板高度,单位:像素 */

/* 面板RGB时序(RGB Output Timing, PCLK = 22MHz) */
#define MIPI_LCD_HSW                    10          /* Hsync  脉冲宽度 */
#define MIPI_LCD_HBP                    30          /* HBP    水平后廊 */
#define MIPI_LCD_HFP                    30          /* HFP    水平前廊 */
#define MIPI_LCD_VSW                    4           /* Vsync  脉冲宽度 */
#define MIPI_LCD_VBP                    15          /* VBP    垂直后廊 */
#define MIPI_LCD_VFP                    12          /* VFP    垂直前廊 */
#define MIPI_LCD_PCLK_MHZ               22          /* 请求值: DPI源240MHz整数分频 -> 实际24MHz(240/10), 帧率65Hz; 22MHz面板值除不尽 */

/* MIPI DSI总线配置(转接板只引出CLK + D0/D1,即2条数据通道) */
#define MIPI_DSI_LANE_NUM               2           /* 2个通道数据线 */
#define MIPI_DSI_LANE_BITRATE_MBPS      600         /* 通道比特率: 528Mbps有效带宽/2通道; 600 让 TxClkEsc=18.75MHz 落在 2~20MHz 规格内 */

/* 设置VDD_MIPI_DPHY输出电压 */
#define MIPI_DSI_PHY_PWR_LDO_CHAN       3           /* LDO_VO3 连接 VDD_MIPI_DPHY */
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500        /* VDD_MIPI_DPHY: IDF示例取2.5V, 1.9V时D0回不到LP-11 */

/* 用于存放初始化序列结构体类型 */
typedef struct {
    int cmd;                /* 命令 */
    const void *data;       /* 数据 */
    size_t data_bytes;      /* 数据大小 */
    uint16_t delay_ms;      /* 发送命令后的延时 */
} mipi_lcd_init_cmd_t;

/* 初始化屏幕结构体 */
typedef struct {
    esp_lcd_panel_t base;           /* LCD设备的基础接口函数 */
    esp_lcd_panel_io_handle_t io;   /* LCD设备的IO接口函数配置 */
    int reset_gpio_num;             /* 复位管脚 */
    int x_gap;                      /* x偏移 */
    int y_gap;                      /* y偏移 */
    uint8_t madctl_val;             /* 保存LCD CMD MADCTL寄存器的当前值 */
    uint8_t colmod_val;             /* 保存LCD_CMD_COLMOD寄存器的当前值 */
    bool reset_level;               /* 复位电平 */
} mipi_panel_t;

/* 调试用: 初始化进度, 卡死时可经JTAG读取(不依赖串口) */
extern volatile uint32_t g_lcd_stage;
#define LCD_STAGE(n)    do { g_lcd_stage = (n); } while (0)

/* 函数声明 */
esp_lcd_panel_handle_t mipi_lcd_init(void);     /* mipi_lcd初始化 */
void mipi_dev_bsp_enable_dsi_phy_power(void);   /* 配置mipi phy电压 */

#endif
