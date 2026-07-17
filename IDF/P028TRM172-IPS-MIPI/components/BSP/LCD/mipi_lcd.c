/**
 ****************************************************************************************************
 * @file        mipi_lcd.c
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

#include "mipi_lcd.h"


static const char *mipi_lcd_tag = "mipi_lcd";
uint8_t mipi_id[2];     /* 存放MIPI屏驱动IC的ID */
_mipilcd_dev mipidev;   /* 管理MIPI重要参数 */

#define MIPI_LCD_ID_JD9855   (0x9855)
#define MIPI_LCD_ID_ST7701S  (0x7701)

static const mipi_lcd_init_cmd_t vendor_specific_init_code_default_800p[] = {
    /* {cmd, { data }, data_size} */
    /* CMD_Page 3 */
    {ILI9881C_CMD_CNDBKxSEL, (uint8_t []){ILI9881C_CMD_BKxSEL_BYTE0, ILI9881C_CMD_BKxSEL_BYTE1, ILI9881C_CMD_BKxSEL_BYTE2_PAGE3}, 3},
    {0x01, (uint8_t []){0x00}, 1},
    {0x02, (uint8_t []){0x00}, 1},
    {0x03, (uint8_t []){0x53}, 1},
    {0x04, (uint8_t []){0xD3}, 1},
    {0x05, (uint8_t []){0x00}, 1},
    {0x06, (uint8_t []){0x0D}, 1},
    {0x07, (uint8_t []){0x08}, 1},
    {0x08, (uint8_t []){0x00}, 1},
    {0x09, (uint8_t []){0x00}, 1},
    {0x0a, (uint8_t []){0x00}, 1},
    {0x0b, (uint8_t []){0x00}, 1},
    {0x0c, (uint8_t []){0x00}, 1},
    {0x0d, (uint8_t []){0x00}, 1},
    {0x0e, (uint8_t []){0x00}, 1},
    {0x0f, (uint8_t []){0x28}, 1},
    {0x10, (uint8_t []){0x28}, 1},
    {0x11, (uint8_t []){0x00}, 1},
    {0x12, (uint8_t []){0x00}, 1},
    {0x13, (uint8_t []){0x00}, 1},
    {0x14, (uint8_t []){0x00}, 1},
    {0x15, (uint8_t []){0x00}, 1},
    {0x16, (uint8_t []){0x00}, 1},
    {0x17, (uint8_t []){0x00}, 1},
    {0x18, (uint8_t []){0x00}, 1},
    {0x19, (uint8_t []){0x00}, 1},
    {0x1a, (uint8_t []){0x00}, 1},
    {0x1b, (uint8_t []){0x00}, 1},
    {0x1c, (uint8_t []){0x00}, 1},
    {0x1d, (uint8_t []){0x00}, 1},
    {0x1e, (uint8_t []){0x40}, 1},
    {0x1f, (uint8_t []){0x80}, 1},
    {0x20, (uint8_t []){0x06}, 1},
    {0x21, (uint8_t []){0x01}, 1},
    {0x22, (uint8_t []){0x00}, 1},
    {0x23, (uint8_t []){0x00}, 1},
    {0x24, (uint8_t []){0x00}, 1},
    {0x25, (uint8_t []){0x00}, 1},
    {0x26, (uint8_t []){0x00}, 1},
    {0x27, (uint8_t []){0x00}, 1},
    {0x28, (uint8_t []){0x33}, 1},
    {0x29, (uint8_t []){0x33}, 1},
    {0x2a, (uint8_t []){0x00}, 1},
    {0x2b, (uint8_t []){0x00}, 1},
    {0x2c, (uint8_t []){0x00}, 1},
    {0x2d, (uint8_t []){0x00}, 1},
    {0x2e, (uint8_t []){0x00}, 1},
    {0x2f, (uint8_t []){0x00}, 1},
    {0x30, (uint8_t []){0x00}, 1},
    {0x31, (uint8_t []){0x00}, 1},
    {0x32, (uint8_t []){0x00}, 1},
    {0x33, (uint8_t []){0x00}, 1},
    {0x34, (uint8_t []){0x03}, 1},
    {0x35, (uint8_t []){0x00}, 1},
    {0x36, (uint8_t []){0x00}, 1},
    {0x37, (uint8_t []){0x00}, 1},
    {0x38, (uint8_t []){0x96}, 1},
    {0x39, (uint8_t []){0x00}, 1},
    {0x3a, (uint8_t []){0x00}, 1},
    {0x3b, (uint8_t []){0x00}, 1},
    {0x3c, (uint8_t []){0x00}, 1},
    {0x3d, (uint8_t []){0x00}, 1},
    {0x3e, (uint8_t []){0x00}, 1},
    {0x3f, (uint8_t []){0x00}, 1},
    {0x40, (uint8_t []){0x00}, 1},
    {0x41, (uint8_t []){0x00}, 1},
    {0x42, (uint8_t []){0x00}, 1},
    {0x43, (uint8_t []){0x00}, 1},
    {0x44, (uint8_t []){0x00}, 1},
    {0x50, (uint8_t []){0x00}, 1},
    {0x51, (uint8_t []){0x23}, 1},
    {0x52, (uint8_t []){0x45}, 1},
    {0x53, (uint8_t []){0x67}, 1},
    {0x54, (uint8_t []){0x89}, 1},
    {0x55, (uint8_t []){0xab}, 1},
    {0x56, (uint8_t []){0x01}, 1},
    {0x57, (uint8_t []){0x23}, 1},
    {0x58, (uint8_t []){0x45}, 1},
    {0x59, (uint8_t []){0x67}, 1},
    {0x5a, (uint8_t []){0x89}, 1},
    {0x5b, (uint8_t []){0xab}, 1},
    {0x5c, (uint8_t []){0xcd}, 1},
    {0x5d, (uint8_t []){0xef}, 1},
    {0x5e, (uint8_t []){0x00}, 1},
    {0x5f, (uint8_t []){0x08}, 1},
    {0x60, (uint8_t []){0x08}, 1},
    {0x61, (uint8_t []){0x06}, 1},
    {0x62, (uint8_t []){0x06}, 1},
    {0x63, (uint8_t []){0x01}, 1},
    {0x64, (uint8_t []){0x01}, 1},
    {0x65, (uint8_t []){0x00}, 1},
    {0x66, (uint8_t []){0x00}, 1},
    {0x67, (uint8_t []){0x02}, 1},
    {0x68, (uint8_t []){0x15}, 1},
    {0x69, (uint8_t []){0x15}, 1},
    {0x6a, (uint8_t []){0x14}, 1},
    {0x6b, (uint8_t []){0x14}, 1},
    {0x6c, (uint8_t []){0x0D}, 1},
    {0x6d, (uint8_t []){0x0D}, 1},
    {0x6e, (uint8_t []){0x0C}, 1},
    {0x6f, (uint8_t []){0x0C}, 1},
    {0x70, (uint8_t []){0x0F}, 1},
    {0x71, (uint8_t []){0x0F}, 1},
    {0x72, (uint8_t []){0x0E}, 1},
    {0x73, (uint8_t []){0x0E}, 1},
    {0x74, (uint8_t []){0x02}, 1},
    {0x75, (uint8_t []){0x08}, 1},
    {0x76, (uint8_t []){0x08}, 1},
    {0x77, (uint8_t []){0x06}, 1},
    {0x78, (uint8_t []){0x06}, 1},
    {0x79, (uint8_t []){0x01}, 1},
    {0x7a, (uint8_t []){0x01}, 1},
    {0x7b, (uint8_t []){0x00}, 1},
    {0x7c, (uint8_t []){0x00}, 1},
    {0x7d, (uint8_t []){0x02}, 1},
    {0x7e, (uint8_t []){0x15}, 1},
    {0x7f, (uint8_t []){0x15}, 1},
    {0x80, (uint8_t []){0x14}, 1},
    {0x81, (uint8_t []){0x14}, 1},
    {0x82, (uint8_t []){0x0D}, 1},
    {0x83, (uint8_t []){0x0D}, 1},
    {0x84, (uint8_t []){0x0C}, 1},
    {0x85, (uint8_t []){0x0C}, 1},
    {0x86, (uint8_t []){0x0F}, 1},
    {0x87, (uint8_t []){0x0F}, 1},
    {0x88, (uint8_t []){0x0E}, 1},
    {0x89, (uint8_t []){0x0E}, 1},
    {0x8A, (uint8_t []){0x02}, 1},
    {ILI9881C_CMD_CNDBKxSEL, (uint8_t []){ILI9881C_CMD_BKxSEL_BYTE0, ILI9881C_CMD_BKxSEL_BYTE1, ILI9881C_CMD_BKxSEL_BYTE2_PAGE4}, 3},
    {0x6E, (uint8_t []){0x2B}, 1},
    {0x6F, (uint8_t []){0x37}, 1},
    {0x3A, (uint8_t []){0x24}, 1},
    {0x8D, (uint8_t []){0x1A}, 1},
    {0x87, (uint8_t []){0xBA}, 1},
    {0xB2, (uint8_t []){0xD1}, 1},
    {0x88, (uint8_t []){0x0B}, 1},
    {0x38, (uint8_t []){0x01}, 1},
    {0x39, (uint8_t []){0x00}, 1},
    {0xB5, (uint8_t []){0x02}, 1},
    {0x31, (uint8_t []){0x25}, 1},
    {0x3B, (uint8_t []){0x98}, 1},
    {ILI9881C_CMD_CNDBKxSEL, (uint8_t []){ILI9881C_CMD_BKxSEL_BYTE0, ILI9881C_CMD_BKxSEL_BYTE1, ILI9881C_CMD_BKxSEL_BYTE2_PAGE1}, 3},
    {0x22, (uint8_t []){0x0A}, 1},
    {0x31, (uint8_t []){0x00}, 1},
    {0x53, (uint8_t []){0x3D}, 1},
    {0x55, (uint8_t []){0x3D}, 1},
    {0x50, (uint8_t []){0xB5}, 1},
    {0x51, (uint8_t []){0xAD}, 1},
    {0x60, (uint8_t []){0x06}, 1},
    {0x62, (uint8_t []){0x20}, 1},
    {0xB7, (uint8_t []){0x03}, 1},
    {0xA0, (uint8_t []){0x00}, 1},
    {0xA1, (uint8_t []){0x21}, 1},
    {0xA2, (uint8_t []){0x35}, 1},
    {0xA3, (uint8_t []){0x19}, 1},
    {0xA4, (uint8_t []){0x1E}, 1},
    {0xA5, (uint8_t []){0x33}, 1},
    {0xA6, (uint8_t []){0x27}, 1},
    {0xA7, (uint8_t []){0x26}, 1},
    {0xA8, (uint8_t []){0xAF}, 1},
    {0xA9, (uint8_t []){0x1B}, 1},
    {0xAA, (uint8_t []){0x27}, 1},
    {0xAB, (uint8_t []){0x8D}, 1},
    {0xAC, (uint8_t []){0x1A}, 1},
    {0xAD, (uint8_t []){0x1B}, 1},
    {0xAE, (uint8_t []){0x50}, 1},
    {0xAF, (uint8_t []){0x26}, 1},
    {0xB0, (uint8_t []){0x2B}, 1},
    {0xB1, (uint8_t []){0x54}, 1},
    {0xB2, (uint8_t []){0x5E}, 1},
    {0xB3, (uint8_t []){0x23}, 1},
    {0xC0, (uint8_t []){0x00}, 1},
    {0xC1, (uint8_t []){0x21}, 1},
    {0xC2, (uint8_t []){0x35}, 1},
    {0xC3, (uint8_t []){0x19}, 1},
    {0xC4, (uint8_t []){0x1E}, 1},
    {0xC5, (uint8_t []){0x33}, 1},
    {0xC6, (uint8_t []){0x27}, 1},
    {0xC7, (uint8_t []){0x26}, 1},
    {0xC8, (uint8_t []){0xAF}, 1},
    {0xC9, (uint8_t []){0x1B}, 1},
    {0xCA, (uint8_t []){0x27}, 1},
    {0xCB, (uint8_t []){0x8D}, 1},
    {0xCC, (uint8_t []){0x1A}, 1},
    {0xCD, (uint8_t []){0x1B}, 1},
    {0xCE, (uint8_t []){0x50}, 1},
    {0xCF, (uint8_t []){0x26}, 1},
    {0xD0, (uint8_t []){0x2B}, 1},
    {0xD1, (uint8_t []){0x54}, 1},
    {0xD2, (uint8_t []){0x5E}, 1},
    {0xD3, (uint8_t []){0x23}, 1},
    {ILI9881C_CMD_CNDBKxSEL, (uint8_t []){ILI9881C_CMD_BKxSEL_BYTE0, ILI9881C_CMD_BKxSEL_BYTE1, ILI9881C_CMD_BKxSEL_BYTE2_PAGE0}, 3},
    {0x35, (uint8_t []){0x00}, 1},
    {0x29, (uint8_t []){0x00}, 0},
};

/* 5.5寸MIPI 1080P初始化序列 */
static const mipi_lcd_init_cmd_t vendor_specific_init_code_default_1080p[] = {
    /* {cmd, { data }, data_size} */
    /* 1080 * 1920 */
    {UD_SETEXTC,        (uint8_t []){0xFF, 0x83, 0x99}, 3},
    {UD_SETOFFSET,      (uint8_t []){0x77}, 1},
    {UD_SETPOWER,       (uint8_t []){0x02, 0x04, 0x74, 0x94, 0x01, 0x32, 0x33, 0x11, 0x11, 0xAB, 0x4D, 0x56, 0x73, 0x02, 0x02}, 15},
    {UD_SETDISP,        (uint8_t []){0x00, 0x80, 0x80, 0xAE, 0x05, 0x07, 0x5A, 0x11, 0x00, 0x00, 0x10, 0x1E, 0x70, 0x03, 0xD4}, 15},
    {UD_SETCYC,         (uint8_t []){0x00, 0xFF, 0x02, 0xC0, 0x02, 0xC0, 0x00, 0x00, 0x08, 0x00, 0x04, 0x06, 0x00, 0x32, 0x04, 0x0A, 0x08, 0x21, 0x03, 0x01, 0x00, 0x0F, 0xB8, 0x8B, 0x02, 0xC0, 0x02, 0xC0, 0x00, 0x00, 0x08, 0x00, 0x04, 0x06, 0x00, 0x32, 0x04, 0x0A, 0x08, 0x01, 0x00, 0x0F, 0xB8, 0x01}, 44},
    {UD_SETMIPI,        (uint8_t []){0x61}, 1},
    {UD_SETGIP0,        (uint8_t []){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x10, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x05, 0x05, 0x07, 0x00, 0x00, 0x00, 0x05, 0x40}, 33},
    {UD_SETGIP1,        (uint8_t []){0x18, 0x18, 0x19, 0x19, 0x18, 0x18, 0x21, 0x20, 0x01, 0x00, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x2F, 0x2F, 0x30, 0x30, 0x31, 0x31, 0x18, 0x18, 0x18, 0x18}, 32},
    {UD_SETGIP2,        (uint8_t []){0x18, 0x18, 0x19, 0x19, 0x40, 0x40, 0x20, 0x21, 0x06, 0x07, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x2F, 0x2F, 0x30, 0x30, 0x31, 0x31, 0x40, 0x40, 0x40, 0x40}, 32},
    {UD_SETGIP3,        (uint8_t []){0xA2, 0xAA, 0x02, 0xA0, 0xA2, 0xA8, 0x02, 0xA0, 0xB0, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x00, 0x00}, 16},
    {UD_SETREGBANK,     (uint8_t []){0x01}, 1},
    {UD_SETGIP3,        (uint8_t []){0xB0, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x00, 0x00, 0xE2, 0xAA, 0x03, 0xF0, 0xE2, 0xAA, 0x03, 0xF0}, 16},
    {UD_SETREGBANK,     (uint8_t []){0x02}, 1},
    {UD_SETGIP3,        (uint8_t []){0xE2, 0xAA, 0x03, 0xF0, 0xE2, 0xAA, 0x03, 0xF0}, 8},
    {UD_SETREGBANK,     (uint8_t []){0x00}, 1},
    {UD_SETVCOM,        (uint8_t []){0x8D, 0x8D}, 2},
    {UD_SETGAMMA,       (uint8_t []){0x00, 0x0E, 0x19, 0x13, 0x2E, 0x39, 0x48, 0x44, 0x4D, 0x57, 0x5F, 0x66, 0x6C, 0x76, 0x7F, 0x85, 0x8A, 0x95, 0x9A, 0xA4, 0x9B, 0xAB, 0xB0, 0x5C, 0x58, 0x64, 0x77, 0x00, 0x0E, 0x19, 0x13, 0x2E, 0x39, 0x48, 0x44, 0x4D, 0x57, 0x5F, 0x66, 0x6C, 0x76, 0x7F, 0x85, 0x8A, 0x95, 0x9A, 0xA4, 0x9B, 0xAB, 0xB0, 0x5C, 0x58, 0x64, 0x77}, 54},
};

/* 5.5寸MIPI 720P初始化序列 */
static const mipi_lcd_init_cmd_t vendor_specific_init_code_default_720p[] = {
    /* {cmd, { data }, data_size} */
    /* 720 * 1280 */
    {UD_SETADDRESSMODE, (uint8_t []){0x01}, 1},
    {UD_SETEXTC,        (uint8_t []){0xFF, 0x83, 0x94}, 3},
    {UD_SETMIPI,        (uint8_t []){0x61, 0x03, 0x68, 0x6B, 0xB2, 0xC0}, 6},
    {UD_SETPOWER,       (uint8_t []){0x48, 0x12, 0x72, 0x09, 0x32, 0x54, 0x71, 0x71, 0x57, 0x47}, 10},
    {UD_SETDISP,        (uint8_t []){0x00, 0x80, 0x64, 0x0C, 0x0D, 0x2F}, 6},
    {UD_SETCYC,         (uint8_t []){0x73, 0x74, 0x73, 0x74, 0x73, 0x74, 0x01, 0x0C, 0x86, 0x75, 0x00, 0x3F, 0x73, 0x74, 0x73, 0x74, 0x73, 0x74, 0x01, 0x0C, 0x86}, 21},
    {UD_SETGIP0,        (uint8_t []){0x00, 0x00, 0x07, 0x07, 0x40, 0x07, 0x0C, 0x00, 0x08, 0x10, 0x08, 0x00, 0x08, 0x54, 0x15, 0x0A, 0x05, 0x0A, 0x02, 0x15, 0x06, 0x05, 0x06, 0x47, 0x44, 0x0A, 0x0A, 0x4B, 0x10, 0x07, 0x07, 0x0C, 0x40}, 33},
    {UD_SETGIP1,        (uint8_t []){0x1C, 0x1C, 0x1D, 0x1D, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x24, 0x25, 0x18, 0x18, 0x26, 0x27, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x20, 0x21, 0x18, 0x18, 0x18, 0x18}, 44},
    {UD_SETGIP2,        (uint8_t []){0x1C, 0x1C, 0x1D, 0x1D, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00, 0x0B, 0x0A, 0x09, 0x08, 0x21, 0x20, 0x18, 0x18, 0x27, 0x26, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x25, 0x24, 0x18, 0x18, 0x18, 0x18}, 44},
    {UD_SETVCOM,        (uint8_t []){0x6E, 0x6E}, 2},
    {UD_SETGAMMA,       (uint8_t []){0x00, 0x0A, 0x15, 0x1B, 0x1E, 0x21, 0x24, 0x22, 0x47, 0x56, 0x65, 0x66, 0x6E, 0x82, 0x88, 0x8B, 0x9A, 0x9D, 0x98, 0xA8, 0xB9, 0x5D, 0x5C, 0x61, 0x66, 0x6A, 0x6F, 0x7F, 0x7F, 0x00, 0x0A, 0x15, 0x1B, 0x1E, 0x21, 0x24, 0x22, 0x47, 0x56, 0x65, 0x65, 0x6E, 0x81, 0x87, 0x8B, 0x98, 0x9D, 0x99, 0xA8, 0xBA, 0x5D, 0x5D, 0x62, 0x67, 0x6B, 0x72, 0x7F, 0x7F}, 58},
    {0xC0,              (uint8_t []){0x1F, 0x31}, 2},
    {UD_SETPANEL,       (uint8_t []){0x03}, 1},
    {0xD4,              (uint8_t []){0x02}, 1},
    {UD_SETREGBANK,     (uint8_t []){0x02}, 1},
    {UD_SETGIP3,        (uint8_t []){0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, 12},
    {UD_SETREGBANK,     (uint8_t []){0x00}, 1},
    {UD_SETREGBANK,     (uint8_t []){0x01}, 1},
    {UD_SETPOWER,       (uint8_t []){0x00}, 1},
    {UD_SETREGBANK,     (uint8_t []){0x00}, 1},
    {0xBF,              (uint8_t []){0x40, 0x81, 0x50, 0x00, 0x1A, 0xFC, 0x01}, 7},
    {0xC6,              (uint8_t []){0xED}, 1},
};

/* JD9855 360*360 初始化序列 */
static const mipi_lcd_init_cmd_t vendor_specific_init_code_jd9855[] = {
    {0xDE, (uint8_t[]){0x00}, 1},
    {0xDF, (uint8_t[]){0x98, 0x55}, 2},
    {0xB2, (uint8_t[]){0x1F}, 1},
    {0xB7, (uint8_t[]){0x00, 0x1D, 0x00, 0x45}, 4},
    {0xBB, (uint8_t[]){0x1B, 0x64, 0xC4, 0x1E, 0x3E, 0xF5}, 6},
    {0xBC, (uint8_t[]){0x03, 0x20, 0xF3, 0xC0}, 4},
    {0xC0, (uint8_t[]){0x22, 0xA1}, 2},
    {0xC3, (uint8_t[]){0x00, 0x02, 0x2A, 0x0B, 0x08, 0x48, 0x08, 0x04, 0x62, 0x30, 0x30}, 11},
    {0xC4, (uint8_t[]){0x40, 0x00, 0xAD, 0x68, 0x43, 0x07, 0x04, 0x16, 0x43, 0x07, 0x04}, 11},
    {0xC8, (uint8_t[]){0x3F, 0x36, 0x30, 0x2D, 0x2F, 0x31, 0x2B, 0x2A, 0x27, 0x25, 0x1F, 0x14, 0x10, 0x09, 0x04, 0x00, 0x3F, 0x36, 0x30, 0x2D, 0x2F, 0x31, 0x2B, 0x2A, 0x27, 0x25, 0x1F, 0x14, 0x10, 0x09, 0x04, 0x00}, 32},
    {0xD3, (uint8_t[]){0x28, 0x13}, 2},
    {0xD9, (uint8_t[]){0x00, 0x00, 0xFF, 0x00, 0xF0, 0x00}, 6},
    {0xDE, (uint8_t[]){0x01}, 1},
    {0xB7, (uint8_t[]){0x13, 0xE7, 0x64, 0x39, 0x06, 0x36, 0x19, 0x1C}, 8},
    {0xBE, (uint8_t[]){0x00}, 1},
    {0xC1, (uint8_t[]){0x00, 0x4A, 0x80}, 3},
    {0xC2, (uint8_t[]){0x00, 0x16, 0xDA, 0xE7}, 4},
    {0xC7, (uint8_t[]){0x00, 0x00, 0x00, 0x38, 0x08, 0x08, 0x00, 0x01}, 8},
    {0xC8, (uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x15, 0x3B}, 6},
    {0xC9, (uint8_t[]){0x00, 0x16, 0x06, 0x04, 0x0A}, 5},
    {0xCA, (uint8_t[]){0x08, 0x35, 0x16, 0x1F, 0x1F}, 5},
    {0xCB, (uint8_t[]){0x01, 0x16, 0x07, 0x05, 0x0B}, 5},
    {0xCC, (uint8_t[]){0x09, 0x35, 0x16, 0x1F, 0x1F}, 5},
    {0xCD, (uint8_t[]){0x01, 0x16, 0x09, 0x0B, 0x05}, 5},
    {0xCE, (uint8_t[]){0x07, 0x15, 0x1F, 0x16, 0x1F}, 5},
    {0xCF, (uint8_t[]){0x00, 0x16, 0x08, 0x0A, 0x04}, 5},
    {0xD0, (uint8_t[]){0x06, 0x15, 0x1F, 0x16, 0x1F}, 5},
    {0xD1, (uint8_t[]){0x02, 0x30}, 2},
    {0xD2, (uint8_t[]){0x02, 0x03, 0x52, 0xDF, 0xDD}, 5},
    {0xD3, (uint8_t[]){0x3B, 0x04, 0x48}, 3},
    {0xD5, (uint8_t[]){0x10, 0x10, 0x07, 0x07, 0x0F, 0x94, 0x26}, 7},
    {0xD6, (uint8_t[]){0x00, 0x00, 0x40}, 3},
    {0xD7, (uint8_t[]){0x00, 0x00, 0x20}, 3},
    {0xDE, (uint8_t[]){0x02}, 1},
    {0xB6, (uint8_t[]){0x1C}, 1},
    {0xDE, (uint8_t[]){0x00}, 1},
    {0x4D, (uint8_t[]){0x00}, 1},
    {0x4E, (uint8_t[]){0x00}, 1},
    {0x4F, (uint8_t[]){0x00}, 1},
    {0x4C, (uint8_t[]){0x01}, 1},
    {0x4C, (uint8_t[]){0x00}, 1},
    {0x35, (uint8_t[]){0x00}, 1},
    {0x3A, (uint8_t[]){0x55}, 1},
    {0x2A, (uint8_t[]){0x00, 0x00, 0x01, 0x67}, 4},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0x67}, 4},
    {0x11, NULL, 0},
    {0x29, NULL, 0},
};

/* BOE 2.83寸 480*640 ST7701S 初始化序列 */
static const mipi_lcd_init_cmd_t vendor_specific_init_code_st7701s[] = {
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x13}, 5},
    {0xEF, (uint8_t[]){0x08}, 1},
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x10}, 5},
    {0xC0, (uint8_t[]){0x4F, 0x00}, 2},
    {0xC1, (uint8_t[]){0x0C, 0x02}, 2},
    {0xC2, (uint8_t[]){0x37, 0x06}, 2},
    {0xB0, (uint8_t[]){0x40, 0xC3, 0x96, 0x0C, 0x12, 0x08, 0x07, 0x06, 0x06, 0x20, 0x03, 0x92, 0x10, 0x66, 0xAF, 0x9D}, 16},
    {0xB1, (uint8_t[]){0x40, 0xC3, 0x95, 0x0E, 0x11, 0x07, 0x0A, 0x09, 0x09, 0x22, 0x05, 0x11, 0x0F, 0x28, 0xEE, 0xD9}, 16},
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x11}, 5},
    {0xB0, (uint8_t[]){0x3D}, 1},
    {0xB1, (uint8_t[]){0x7F}, 1},
    {0xB2, (uint8_t[]){0x89}, 1},
    {0xB3, (uint8_t[]){0x80}, 1},
    {0xB5, (uint8_t[]){0x49}, 1},
    {0xB7, (uint8_t[]){0x85}, 1},
    {0xB8, (uint8_t[]){0x23}, 1},
    {0xC1, (uint8_t[]){0x78}, 1},
    {0xC2, (uint8_t[]){0x78}, 1},
    {0xD0, (uint8_t[]){0x88}, 1},
    {0xE0, (uint8_t[]){0x00, 0x00, 0x02}, 3},
    {0xE1, (uint8_t[]){0x07, 0xC0, 0x07, 0xC0, 0x06, 0xC0, 0x06, 0xC0, 0x00, 0x24, 0x24}, 11},
    {0xE2, (uint8_t[]){0x20, 0x20, 0x22, 0x22, 0x8D, 0xC0, 0x00, 0x00, 0x8C, 0xC0, 0x00, 0x00, 0x00}, 13},
    {0xE3, (uint8_t[]){0x00, 0x00, 0x22, 0x22}, 4},
    {0xE4, (uint8_t[]){0x44, 0x44}, 2},
    {0xE5, (uint8_t[]){0x09, 0x91, 0x10, 0xFA, 0x0B, 0x93, 0x10, 0xFA, 0x0D, 0x8D, 0x10, 0xFA, 0x0F, 0x8F, 0x10, 0xFA}, 16},
    {0xE6, (uint8_t[]){0x00, 0x00, 0x22, 0x22}, 4},
    {0xE7, (uint8_t[]){0x44, 0x44}, 2},
    {0xE8, (uint8_t[]){0x08, 0x90, 0x10, 0xFA, 0x0A, 0x92, 0x10, 0xFA, 0x0C, 0x8C, 0x10, 0xFA, 0x0E, 0x8E, 0x10, 0xFA}, 16},
    {0xEB, (uint8_t[]){0x02, 0x01, 0xE4, 0xE4, 0x88, 0x00, 0x00}, 7},
    {0xED, (uint8_t[]){0xFF, 0x04, 0x56, 0x7F, 0xBA, 0x2F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF2, 0xAB, 0xF7, 0x65, 0x40, 0xFF}, 16},
    {0xEF, (uint8_t[]){0x08, 0x08, 0x08, 0x45, 0x3F, 0x54}, 6},
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x00}, 5},
    {0x11, NULL, 0, 120},
    {0x29, NULL, 0, 20},
};

/**
 * @brief       配置mipi phy电压
 * @param       无
 * @retval      无
 */
void mipi_dev_bsp_enable_dsi_phy_power(void)
{
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
#ifdef MIPI_DSI_PHY_PWR_LDO_CHAN
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id    = MIPI_DSI_PHY_PWR_LDO_CHAN,        /* 选择内存LDO */
        .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,  /* 输出标准电压提供VDD_MIPI_DPHY */
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));
    ESP_LOGI(mipi_lcd_tag, "MIPI DSI PHY Powered on");
#endif
}

/**
 * @brief       删除LCD面板
 * @param       panel:LCD接口句柄
 * @retval      ESP_OK:删除成功
 */
static esp_err_t mipi_lcd_paneldel(esp_lcd_panel_t *panel)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);
    if (mipi_lcd->reset_gpio_num >= 0)
    {
        gpio_reset_pin(mipi_lcd->reset_gpio_num);
    }

    free(mipi_lcd);

    return ESP_OK;
}

/**
 * @brief       复位LCD面板
 * @param       panel:LCD接口句柄
 * @retval      ESP_OK:复位成功
 */
static esp_err_t mipi_lcd_panelreset(esp_lcd_panel_t *panel)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);
    esp_lcd_panel_io_handle_t io = mipi_lcd->io;

    /* 如果有GPIO控制LCD的复位管脚 */
    if (mipi_lcd->reset_gpio_num >= 0)      /* 硬件复位 */
    {
        gpio_set_level(mipi_lcd->reset_gpio_num, mipi_lcd->reset_level);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(mipi_lcd->reset_gpio_num, !mipi_lcd->reset_level);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    else                                    /* 软件复位 */
    { 
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), mipi_lcd_tag, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return ESP_OK;
}

/**
 * @brief       初始化LCD面板
 * @param       panel:LCD接口句柄
 * @retval      ESP_OK:复位成功
 */
static esp_err_t mipi_lcd_panelinit(esp_lcd_panel_t *panel)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);
    esp_lcd_panel_io_handle_t io = mipi_lcd->io;
    
    const mipi_lcd_init_cmd_t *init_cmds = {0};
    uint16_t init_cmds_size = 0;
    bool mirror_x = true;
    bool mirror_y = false;

    if (mipidev.id == MIPI_LCD_ID_JD9855) /* 1.46寸,360*360 */
    {
        init_cmds = vendor_specific_init_code_jd9855;
        init_cmds_size = sizeof(vendor_specific_init_code_jd9855) / sizeof(mipi_lcd_init_cmd_t);
        mirror_x = false;
        mirror_y = false;
    }
    else if (mipidev.id == MIPI_LCD_ID_ST7701S) /* 2.83寸,480*640 */
    {
        init_cmds = vendor_specific_init_code_st7701s;
        init_cmds_size = sizeof(vendor_specific_init_code_st7701s) / sizeof(mipi_lcd_init_cmd_t);
        mirror_x = false;
        mirror_y = false;
    }
    else if (mipidev.id == 0x8399)       /* 5寸,720P */
    {
        init_cmds = vendor_specific_init_code_default_1080p;
        init_cmds_size = sizeof(vendor_specific_init_code_default_1080p) / sizeof(mipi_lcd_init_cmd_t);
    }
    else if (mipidev.id == 0x8394)  /* 5寸,1080p */
    {
        init_cmds = vendor_specific_init_code_default_720p;
        init_cmds_size = sizeof(vendor_specific_init_code_default_720p) / sizeof(mipi_lcd_init_cmd_t);
    }
    else if (mipidev.id == 0x9881)  /* 10.1寸,800p */
    {
        init_cmds = vendor_specific_init_code_default_800p;
        init_cmds_size = sizeof(vendor_specific_init_code_default_800p) / sizeof(mipi_lcd_init_cmd_t);

        /* 返回 命令页 1 */
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, ILI9881C_CMD_CNDBKxSEL, (uint8_t[]) {
            ILI9881C_CMD_BKxSEL_BYTE0, ILI9881C_CMD_BKxSEL_BYTE1, ILI9881C_CMD_BKxSEL_BYTE2_PAGE1
        }, 3), mipi_lcd_tag, "send command failed");
        /* 设置2 lane */
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, ILI9881C_PAD_CONTROL, (uint8_t[]) {
            ILI9881C_DSI_2_LANE,
        }, 1), mipi_lcd_tag, "send command failed");

        /* 返回 命令页 0 */
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, ILI9881C_CMD_CNDBKxSEL, (uint8_t[]) {
            ILI9881C_CMD_BKxSEL_BYTE0, ILI9881C_CMD_BKxSEL_BYTE1, ILI9881C_CMD_BKxSEL_BYTE2_PAGE0
        }, 3), mipi_lcd_tag, "send command failed");
        mirror_x = false;
    }
    else
    {
        init_cmds = vendor_specific_init_code_jd9855;
        init_cmds_size = sizeof(vendor_specific_init_code_jd9855) / sizeof(mipi_lcd_init_cmd_t);
        mirror_x = false;
        mirror_y = false;
    }

    /* 发送初始化序列 */
    for (int i = 0; i < init_cmds_size; i++)
    {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, init_cmds[i].cmd, init_cmds[i].data, init_cmds[i].data_bytes), mipi_lcd_tag, "send command failed");
        if (init_cmds[i].delay_ms > 0)
        {
            vTaskDelay(pdMS_TO_TICKS(init_cmds[i].delay_ms));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(120));

    /* 退出睡眠 */
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0), mipi_lcd_tag, "io tx param failed");
    
    /* 根据MIPI屏放置位置调整显示方向(也可调用mipi_lcd_panelmirror函数设置) */
    if (mirror_x)
    {
        mipi_lcd->madctl_val |= HX_F_SS_PANEL;      /* 扫描方向水平翻转 */
    }
    else
    {
        mipi_lcd->madctl_val &= ~HX_F_SS_PANEL;     /* 扫描方向水平不翻转 */
    }

    if (mirror_y)
    {
        mipi_lcd->madctl_val |= HX_F_GS_PANEL;      /* 扫描方向垂直翻转 */
    }
    else
    {
        mipi_lcd->madctl_val &= ~HX_F_GS_PANEL;     /* 扫描方向垂直不翻转 */
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[])
    {
        mipi_lcd->madctl_val,
    }, 1), mipi_lcd_tag, "send command failed");    /* 配置MIPILCD的显示 */

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, (uint8_t[])
    {
        mipi_lcd->colmod_val,
    }, 1), mipi_lcd_tag, "send command failed");    /* 配置像素格式 */
    vTaskDelay(pdMS_TO_TICKS(120));

    return ESP_OK;
}

/**
 * @brief       反转显示
 * @param       panel             :LCD接口句柄
 * @param       invert_color_data :true反转显示;false正常显示
 * @retval      ESP_OK:复位成功
 */
static esp_err_t mipi_lcd_panelinvert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);
    esp_lcd_panel_io_handle_t io = mipi_lcd->io;
    int command = 0;

    if (invert_color_data)
    {
        command = LCD_CMD_INVON;    /* 反转显示打开 */
    }
    else
    {
        command = LCD_CMD_INVOFF;   /* 反转显示关闭 */
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), mipi_lcd_tag, "send command failed");
    
    return ESP_OK;
}

/**
 * @brief       在特定轴上镜像LCD面板
 * @param       panel       :LCD接口句柄
 * @param       mirror_x    :true对x轴进行镜像,false不操作镜像
 * @param       mirror_y    :true对y轴进行镜像,false不进行镜像
 * @retval      ESP_OK:复位成功
 */
static esp_err_t mipi_lcd_panelmirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);
    esp_lcd_panel_io_handle_t io = mipi_lcd->io;

    if (mirror_x)
    {
        mipi_lcd->madctl_val |= HX_F_SS_PANEL;      /* 扫描方向水平翻转(镜像x) */
    }
    else
    {
        mipi_lcd->madctl_val &= ~HX_F_SS_PANEL;     /* 扫描方向水平不翻转 */
    }

    if (mirror_y)
    {
        mipi_lcd->madctl_val |= HX_F_GS_PANEL;      /* 扫描方向垂直翻转(镜像y) */
    }
    else
    {
        mipi_lcd->madctl_val &= ~HX_F_GS_PANEL;     /* 扫描方向垂直不翻转 */
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[])
    {
        mipi_lcd->madctl_val,
    }, 1), mipi_lcd_tag, "send command failed");    /* 配置MIPILCD的显示 */

    return ESP_OK;
}

/**
 * @brief       交换x和y轴
 * @param       panel       :LCD接口句柄
 * @param       swap_axes   :是否对x和y轴进行方向互换
 * @retval      ESP_ERR_NOT_SUPPORTED:适配的MIPILCD不支持
 */
static esp_err_t mipi_lcd_panelswap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    ESP_LOGW(mipi_lcd_tag, "Swap XY is not supported in HX8394/8399-F driver. Please use SW rotation.");
    return ESP_ERR_NOT_SUPPORTED;
}

/**
 * @brief       设置x轴和y轴的偏移
 * @param       panel :LCD接口句柄
 * @param       x_gap :X方向偏移
 * @param       y_gap :Y方向偏移
 * @retval      ESP_OK:设置成功
 */
static esp_err_t mipi_lcd_panelset_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);
    mipi_lcd->x_gap = x_gap;
    mipi_lcd->y_gap = y_gap;
    return ESP_OK;
}

/**
 * @brief       打开或者关闭显示
 * @param       panel :LCD接口句柄
 * @param       on_off:true打开显示,flase关闭显示
 * @retval      ESP_OK:设置成功
 */
static esp_err_t mipi_lcd_paneldisp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);
    esp_lcd_panel_io_handle_t io = mipi_lcd->io;
    int command = 0;

    if (on_off)
    {
        command = LCD_CMD_DISPON;   /* 打开显示命令 */
    }
    else
    {
        command = LCD_CMD_DISPOFF;  /* 关闭显示命令 */
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), mipi_lcd_tag, "send command failed");
    
    return ESP_OK;
}

/**
 * @brief       进入或退出睡眠模式
 * @param       sleep :true进入睡眠模式,flase:退出睡眠模式
 * @retval      ESP_OK:设置成功
 */
static esp_err_t mipi_lcd_panelsleep(esp_lcd_panel_t *panel, bool sleep)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);
    esp_lcd_panel_io_handle_t io = mipi_lcd->io;
    int command = 0;

    if (sleep)
    {
        command = LCD_CMD_SLPIN;    /* 进入睡眠模式命令 */
    }
    else
    {
        command = LCD_CMD_SLPOUT;   /* 退出睡眠模式命令 */
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), mipi_lcd_tag, "io tx param failed");
    
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

/**
 * @brief       新建mipilcd句柄，并配置函数
 * @param       io              :MIPI IO句柄
 * @param       panel_dev_config:MIPI 设备配置结构体
 * @param       ret_panel       :MIPI屏句柄
 * @retval      esp_err_t 返回值
 */
esp_err_t mipi_lcd_new_panel(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, mipi_lcd_tag, "invalid argument");

    mipi_panel_t *mipi_lcd = (mipi_panel_t *)calloc(1, sizeof(mipi_panel_t));
    ESP_RETURN_ON_FALSE(mipi_lcd, ESP_ERR_NO_MEM, mipi_lcd_tag, "no mem for mipi_lcd panel");

    if (panel_dev_config->reset_gpio_num >= 0)      /* 配置LCD复位引脚 */
    {
        gpio_config_t io_conf = {
            .mode         = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, mipi_lcd_tag, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->rgb_ele_order)        /* 颜色顺序RGB/BGR */
    {
        case LCD_RGB_ELEMENT_ORDER_RGB:
            mipi_lcd->madctl_val = 0;
            break;
        case LCD_RGB_ELEMENT_ORDER_BGR:
            mipi_lcd->madctl_val |= LCD_CMD_BGR_BIT;
            break;
        default:
            ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, mipi_lcd_tag, "unsupported rgb element order");
            break;
    }

    switch (panel_dev_config->bits_per_pixel)       /* 像素格式 */
    {
        case 16:    /* RGB565 */
            mipi_lcd->colmod_val = 0x55;
            break;
        case 18:    /* RGB666 */
            mipi_lcd->colmod_val = 0x66;
            break;
        case 24:    /* RGB888 */
            mipi_lcd->colmod_val = 0x77;
            break;
        default:
            ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, mipi_lcd_tag, "unsupported pixel width");
            break;
    }

    mipi_lcd->io                = io;
    mipi_lcd->reset_gpio_num    = panel_dev_config->reset_gpio_num;         /* LCD的复位引脚 */
    mipi_lcd->reset_level       = panel_dev_config->flags.reset_active_high;/* 复位引脚的有效电平 */
    mipi_lcd->base.reset        = mipi_lcd_panelreset;                      /* 复位LCD面板 */
    mipi_lcd->base.init         = mipi_lcd_panelinit;                       /* 初始化LCD面板 */
    mipi_lcd->base.del          = mipi_lcd_paneldel;                        /* 删除LCD面板 */
    mipi_lcd->base.mirror       = mipi_lcd_panelmirror;                     /* 在特定轴上镜像LCD面板 */
    mipi_lcd->base.swap_xy      = mipi_lcd_panelswap_xy;                    /* 交换x轴和y轴 */
    mipi_lcd->base.set_gap      = mipi_lcd_panelset_gap;                    /* 设置x轴和y轴的偏移 */
    mipi_lcd->base.invert_color = mipi_lcd_panelinvert_color;               /* 反转显示 */
    mipi_lcd->base.disp_on_off  = mipi_lcd_paneldisp_on_off;                /* 打开或关闭显示 */
    mipi_lcd->base.disp_sleep   = mipi_lcd_panelsleep;                      /* 进入或退出睡眠模式 */

    *ret_panel = &mipi_lcd->base;

    return ESP_OK;

err:
    if (mipi_lcd)
    {
        mipi_lcd_paneldel(&mipi_lcd->base);
    }

    return ret;
}

/**
 * @brief       mipi_lcd初始化
 * @param       无
 * @retval      LCD控制句柄
 */
esp_lcd_panel_handle_t mipi_lcd_init(void)
{
    mipi_dev_bsp_enable_dsi_phy_power();                    /* 配置DSI接口电压1.8V */
    
    /* 创建DSI总线 */
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id             = 0,                            /* 总线ID */
        .num_data_lanes     = MIPI_DSI_LANE_NUM,            /* DSI数据通道数 */
        .phy_clk_src        = MIPI_DSI_PHY_CLK_SRC_DEFAULT, /* DPHY时钟源为20M */
        .lane_bit_rate_mbps = MIPI_DSI_LANE_BITRATE_MBPS,   /* 数据通道比特率(Mbps) */
    };
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));   /* 新建DSI总线 */

    /* 配置DSI总线的DBI接口 */
    esp_lcd_panel_io_handle_t mipi_dbi_io;
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,                               /* 虚拟通道(只有一个LCD连接,设置0即可) */
        .lcd_cmd_bits    = 8,                               /* 根据MIPI LCD驱动IC规格设置 命令位宽度 */
        .lcd_param_bits  = 8,                               /* 根据MIPI LCD驱动IC规格设置 参数位宽度 */
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &mipi_dbi_io));

    /* 创建LCD控制器驱动 */
    esp_lcd_panel_handle_t mipi_lcd_ctrl_panel;             /* MIPI控制句柄 */
    esp_lcd_panel_dev_config_t lcd_dev_config = {
        .bits_per_pixel = 24,                               /* MIPILCD的像素位宽度(RGB565即16位,RGB888即24位) */
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,         /* 像素数据的RGB元素顺序,根据实际色彩情况选择BGR或RGB */
        .reset_gpio_num = lcddev.ctrl.lcd_rst,              /* MIPILCD屏的复位引脚 */
    };
    ESP_ERROR_CHECK(mipi_lcd_new_panel(mipi_dbi_io, &lcd_dev_config, &mipi_lcd_ctrl_panel));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(mipi_lcd_ctrl_panel));              /* 复位MIPILCD屏 */

    /* 读取屏幕ID */
    esp_lcd_panel_io_rx_param(mipi_dbi_io, 0xDA, &mipi_id[0], 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    esp_lcd_panel_io_rx_param(mipi_dbi_io, 0xDB, &mipi_id[1], 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    /* 不是HX8399和HX8394 */
    if (mipi_id[0] == 0x00 || mipi_id[1] == 0x00)
    {
        /* 读取ILI9881 ID */
        esp_lcd_panel_io_tx_param(mipi_dbi_io, ILI9881C_CMD_CNDBKxSEL, (uint8_t[]) {
            ILI9881C_CMD_BKxSEL_BYTE0, ILI9881C_CMD_BKxSEL_BYTE1, ILI9881C_CMD_BKxSEL_BYTE2_PAGE1
        }, 3);
        esp_lcd_panel_io_rx_param(mipi_dbi_io, 0x00, &mipi_id[0], 1);
        esp_lcd_panel_io_rx_param(mipi_dbi_io, 0x01, &mipi_id[1], 1);
    }

    uint16_t read_id = (uint16_t)(mipi_id[0] << 8) | mipi_id[1];
    mipidev.id = MIPI_LCD_ID_ST7701S;
    ESP_LOGI(mipi_lcd_tag, "mipilcd_read_id:%#x, use ST7701S config", read_id);

    ESP_ERROR_CHECK(esp_lcd_panel_init(mipi_lcd_ctrl_panel));               /* 初始化MIPILCD屏 */
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(mipi_lcd_ctrl_panel, true));  /* 打开MIPILCD屏 */

    if (mipidev.id == MIPI_LCD_ID_ST7701S)                      /* 2.83寸480*640屏幕 */
    {
        mipidev.pwidth   = 480;                                 /* 面板宽度,单位:像素 */
        mipidev.pheight  = 640;                                 /* 面板高度,单位:像素 */
        mipidev.hbp      = 30;                                  /* 水平后廊 */
        mipidev.hfp      = 30;                                  /* 水平前廊 */
        mipidev.hsw      = 10;                                  /* 水平同步宽度 */
        mipidev.vbp      = 15;                                  /* 垂直后廊 */
        mipidev.vfp      = 12;                                  /* 垂直前廊 */
        mipidev.vsw      = 4;                                   /* 垂直同步宽度 */
        mipidev.pclk_mhz = 22;                                  /* 设置像素时钟 22Mhz */
        mipidev.dir      = 0;                                   /* 竖屏 */
    }
    else if (mipidev.id == MIPI_LCD_ID_JD9855)                  /* 1.46寸360*360屏幕 */
    {
        mipidev.pwidth   = 360;                                 /* 面板宽度,单位:像素 */
        mipidev.pheight  = 360;                                 /* 面板高度,单位:像素 */
        mipidev.hbp      = 30;                                  /* 水平后廊 */
        mipidev.hfp      = 40;                                  /* 水平前廊 */
        mipidev.hsw      = 20;                                  /* 水平同步宽度 */
        mipidev.vbp      = 10;                                  /* 垂直后廊 */
        mipidev.vfp      = 20;                                  /* 垂直前廊 */
        mipidev.vsw      = 4;                                   /* 垂直同步宽度 */
        mipidev.pclk_mhz = 11;                                  /* 设置像素时钟 11Mhz */
        mipidev.dir      = 0;                                   /* 竖屏 */
    }
    else if (mipidev.id == 0x8394)                              /* 5.5寸720P屏幕 */
    {
        mipidev.pwidth   = 720;                                 /* 面板宽度,单位:像素 */
        mipidev.pheight  = 1280;                                /* 面板高度,单位:像素 */
        mipidev.hbp      = 52;                                  /* 水平后廊 */
        mipidev.hfp      = 48;                                  /* 水平前廊 */
        mipidev.hsw      = 8;                                   /* 水平同步宽度 */
        mipidev.vbp      = 15;                                  /* 垂直后廊 */
        mipidev.vfp      = 16;                                  /* 垂直前廊 */
        mipidev.vsw      = 5;                                   /* 垂直同步宽度 */
        mipidev.pclk_mhz = 60;                                  /* 设置像素时钟 60Mhz */
        mipidev.dir      = 0;                                   /* 只能竖屏 */
    }
    else if (mipidev.id == 0x8399)                              /* 5.5寸1080P屏幕 */
    {
        mipidev.pwidth   = 1080;                                /* 面板宽度,单位:像素 */
        mipidev.pheight  = 1920;                                /* 面板高度,单位:像素 */
        mipidev.hbp      = 22;                                  /* 水平后廊 */
        mipidev.hfp      = 22;                                  /* 水平前廊 */
        mipidev.hsw      = 20;                                  /* 水平同步宽度 */
        mipidev.vbp      = 9;                                   /* 垂直后廊 */
        mipidev.vfp      = 7;                                   /* 垂直前廊 */
        mipidev.vsw      = 7;                                   /* 垂直同步宽度 */
        mipidev.pclk_mhz = 60;                                  /* 设置像素时钟 60Mhz */
        mipidev.dir      = 0;                                   /* 只能竖屏 */
    }
    else if (mipidev.id == 0x9881)                              /* 10.1寸800P屏幕 */
    {
        mipidev.pwidth   = 800;                                 /* 面板宽度,单位:像素 */
        mipidev.pheight  = 1280;                                /* 面板高度,单位:像素 */
        mipidev.hbp      = 24;                                  /* 水平后廊 */
        mipidev.hfp      = 15;                                  /* 水平前廊 */
        mipidev.hsw      = 24;                                  /* 水平同步宽度 */
        mipidev.vbp      = 9;                                   /* 垂直后廊 */
        mipidev.vfp      = 7;                                   /* 垂直前廊 */
        mipidev.vsw      = 2;                                   /* 垂直同步宽度 */
        mipidev.pclk_mhz = 60;                                  /* 设置像素时钟 60Mhz */
        mipidev.dir      = 0;                                   /* 只能竖屏 */
    }

    ESP_LOGI(mipi_lcd_tag, "panel: id=0x%04x, res=%lux%lu, dir=%u",
             mipidev.id, mipidev.pwidth, mipidev.pheight, mipidev.dir);
    ESP_LOGI(mipi_lcd_tag, "timing: hsw=%u, hbp=%u, hfp=%u, vsw=%u, vbp=%u, vfp=%u, pclk=%luMHz",
             mipidev.hsw, mipidev.hbp, mipidev.hfp, mipidev.vsw, mipidev.vbp, mipidev.vfp, mipidev.pclk_mhz);
    ESP_LOGI(mipi_lcd_tag, "dsi: lanes=%u, lane_bitrate=%uMbps, bpp=%u, pixel_format=RGB888",
             MIPI_DSI_LANE_NUM, MIPI_DSI_LANE_BITRATE_MBPS, 24);

    lcddev.id     = mipidev.id;                                 /* LCD_ID */
    lcddev.width  = mipidev.pwidth;                             /* 宽度 */
    lcddev.height = mipidev.pheight;                            /* 高度 */
    lcddev.lcd_dbi_io = mipi_dbi_io;                            /* LCD IO控制句柄 */
    esp_lcd_panel_handle_t mipi_dpi_panel;                      /* MIPILCD控制句柄 */
    esp_lcd_dpi_panel_config_t dpi_config = {                   /* DSI数据配置 */
        .virtual_channel    = 0,                                /* 虚拟通道 */
        .dpi_clk_src        = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     /* 时钟源 */
        .dpi_clock_freq_mhz = mipidev.pclk_mhz,                 /* 像素时钟频率 */
        .pixel_format       = LCD_COLOR_PIXEL_FORMAT_RGB888,    /* 颜色格式 */
        .num_fbs            = 2,                                /* 帧缓冲区数量 */
        .video_timing       = {                                 /* LCD面板特定时序参数 */
            .h_size            = mipidev.pwidth,                /* 水平分辨率,即一行中的像素数 */
            .v_size            = mipidev.pheight,               /* 垂直分辨率,即帧中的行数 */
            .hsync_back_porch  = mipidev.hbp,                   /* 水平后廊,hsync和行活动数据开始之间的PCLK数 */
            .hsync_pulse_width = mipidev.hsw,                   /* 水平同步宽度,单位:PCLK周期*/
            .hsync_front_porch = mipidev.hfp,                   /* 水平前廊,活动数据结束和下一个hsync之间的PCLK数 */
            .vsync_back_porch  = mipidev.vbp,                   /* 垂直后廊,vsync和帧开始之间的无效行数 */
            .vsync_pulse_width = mipidev.vsw,                   /* 垂直同步宽度,单位:行数 */
            .vsync_front_porch = mipidev.vfp,                   /* 垂直前廊,帧结束和下一个vsync之间的无效行数 */
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_dpi(mipi_dsi_bus, &dpi_config, &mipi_dpi_panel));     /* 为MIPI DSI DPI接口创建LCD控制句柄 */
    ESP_ERROR_CHECK(esp_lcd_panel_init(mipi_dpi_panel));                                    /* 初始化MIPILCD */

    return mipi_dpi_panel;
}
