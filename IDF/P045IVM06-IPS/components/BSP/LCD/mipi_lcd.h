/**
 ****************************************************************************************************
 * @file        mipi_lcd.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-01
 * @brief       mipilcd驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32-P4 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
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
#include <math.h>
#include <string.h>

/* MIPI DSI总线配置 */
#define MIPI_DSI_LANE_NUM               2       /* 2个通道数据线 */
#define MIPI_DSI_LANE_BITRATE_MBPS      500     /* 通道比特率 */

/* 设置VDD_MIPI_DPHY输出电压 */
#define MIPI_DSI_PHY_PWR_LDO_CHAN       3       /* LDO_VO3 连接 VDD_MIPI_DPHY */
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 1900    /* 输出1.8V给到MIPI屏 */

/* ILI9881C User Define command set 用户自定义命令集 */
#define ILI9881C_CMD_CNDBKxSEL                  (0xFF)
#define ILI9881C_CMD_BKxSEL_BYTE0               (0x98)
#define ILI9881C_CMD_BKxSEL_BYTE1               (0x81)
#define ILI9881C_CMD_BKxSEL_BYTE2_PAGE0         (0x00)
#define ILI9881C_CMD_BKxSEL_BYTE2_PAGE1         (0x01)
#define ILI9881C_CMD_BKxSEL_BYTE2_PAGE2         (0x02)
#define ILI9881C_CMD_BKxSEL_BYTE2_PAGE3         (0x03)
#define ILI9881C_CMD_BKxSEL_BYTE2_PAGE4         (0x04)
#define ILI9881C_PAD_CONTROL                    (0xB7)
#define ILI9881C_DSI_2_LANE                     (0x03)
#define ILI9881C_DSI_3_4_LANE                   (0x02)

/* HX83xx User Define command set 用户自定义命令集 */
#define UD_SETADDRESSMODE                       0x36    /* Set address mode */
#define UD_SETSEQUENCE                          0xB0    /* Set sequence */
#define UD_SETPOWER                             0xB1    /* Set power */
#define UD_SETDISP                              0xB2    /* Set display related register */
#define UD_SETCYC                               0xB4    /* Set display waveform cycles */
#define UD_SETVCOM                              0xB6    /* Set VCOM voltage */
#define UD_SETTE                                0xB7    /* Set internal TE function */
#define UD_SETSENSOR                            0xB8    /* Set temperature sensor */
#define UD_SETEXTC                              0xB9    /* Set extension command */
#define UD_SETMIPI                              0xBA    /* Set MIPI control */
#define UD_SETOTP                               0xBB    /* Set OTP */
#define UD_SETREGBANK                           0xBD    /* Set register bank */
#define UD_SETDGCLUT                            0xC1    /* Set DGC LUT */
#define UD_SETID                                0xC3    /* Set ID */
#define UD_SETDDB                               0xC4    /* Set DDB */
#define UD_SETCABC                              0xC9    /* Set CABC control */
#define UD_SETCABCGAIN                          0xCA
#define UD_SETPANEL                             0xCC
#define UD_SETOFFSET                            0xD2
#define UD_SETGIP0                              0xD3    /* Set GIP Option0 */
#define UD_SETGIP1                              0xD5    /* Set GIP Option1 */
#define UD_SETGIP2                              0xD6    /* Set GIP Option2 */
#define UD_SETGIP3                              0xD8    /* Set GIP Option2 */
#define UD_SETGPO                               0xD9
#define UD_SETSCALING                           0xDD
#define UD_SETIDLE                              0xDF
#define UD_SETGAMMA                             0xE0    /* Set gamma curve related setting */
#define UD_SETCHEMODE_DYN                       0xE4
#define UD_SETCHE                               0xE5
#define UD_SETCESEL                             0xE6    /* Enable color enhance */
#define UD_SET_SP_CMD                           0xE9
#define UD_SETREADINDEX                         0xFE    /* Set SPI Read Index */
#define UD_GETSPIREAD                           0xFF    /* SPI Read Command Data */

#define HX_F_GS_PANEL                           (1 << 0)    /* 垂直翻转 */
#define HX_F_SS_PANEL                           (1 << 1)    /* 水平翻转 */
#define HX_F_BGR_PANEL                          (1 << 3)    /* 颜色选择BGR / RGB */

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
    uint16_t init_cmds_size;        /* 初始化序列大小 */
    bool reset_level;               /* 复位电平 */
} mipi_panel_t;

/* MIPI LCD重要参数集 */
typedef struct  
{
    uint16_t id;                    /* 720P还是1080p */
    uint32_t pwidth;                /* MIPI面板的宽度,固定参数,不随显示方向改变 */
    uint32_t pheight;               /* MIPI面板的高度,固定参数,不随显示方向改变 */
    uint16_t hsw;                   /* 水平同步宽度 */
    uint16_t vsw;                   /* 垂直同步宽度 */
    uint16_t hbp;                   /* 水平后廊 */
    uint16_t vbp;                   /* 垂直后廊 */
    uint16_t hfp;                   /* 水平前廊 */
    uint16_t vfp;                   /* 垂直前廊  */
    uint8_t dir;                    /* 0,竖屏;1,横屏; */
    uint32_t pclk_mhz;              /* 设置像素时钟 */
} _mipilcd_dev; 

/* MIPI参数 */
extern _mipilcd_dev mipidev;        /* 管理MIPI屏幕的重要参数 */

/* 函数声明 */
esp_lcd_panel_handle_t mipi_lcd_init(void);     /* mipi_lcd初始化 */
void mipi_dev_bsp_enable_dsi_phy_power(void);   /* 配置mipi phy电压 */

#endif
