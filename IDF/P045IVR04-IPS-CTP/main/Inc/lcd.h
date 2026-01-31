#ifndef __LDC_H
#define __LCD_H

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 屏幕分辨率参数，需要根据你的实际情况配置
#define EXAMPLE_LCD_H_RES (480)
#define EXAMPLE_LCD_V_RES (854)

// SPI引脚配置，需要根据你的实际情况配置
#define EXAMPLE_LCD_IO_SPI_CS (GPIO_NUM_11)
#define EXAMPLE_LCD_IO_SPI_SCL (GPIO_NUM_47)
#define EXAMPLE_LCD_IO_SPI_SDA (GPIO_NUM_48)

// RGB屏幕引脚配置，需要根据你的实际情况配置
#define EXAMPLE_RGB_DATA_WIDTH (16)
#define EXAMPLE_LCD_BIT_PER_PIXEL (16)

#define EXAMPLE_LCD_IO_RGB_DISP (-1) // -1 if not used
#define EXAMPLE_LCD_IO_RGB_VSYNC (GPIO_NUM_45)
#define EXAMPLE_LCD_IO_RGB_HSYNC (GPIO_NUM_46) //
#define EXAMPLE_LCD_IO_RGB_DE (GPIO_NUM_9)     //
#define EXAMPLE_LCD_IO_RGB_PCLK (GPIO_NUM_0)

// B
#define EXAMPLE_LCD_IO_RGB_DATA0 (GPIO_NUM_38)
#define EXAMPLE_LCD_IO_RGB_DATA1 (GPIO_NUM_17)
#define EXAMPLE_LCD_IO_RGB_DATA2 (GPIO_NUM_18)
#define EXAMPLE_LCD_IO_RGB_DATA3 (GPIO_NUM_8)
#define EXAMPLE_LCD_IO_RGB_DATA4 (GPIO_NUM_3)
// G
#define EXAMPLE_LCD_IO_RGB_DATA5 (GPIO_NUM_41)
#define EXAMPLE_LCD_IO_RGB_DATA6 (GPIO_NUM_7)
#define EXAMPLE_LCD_IO_RGB_DATA7 (GPIO_NUM_40)
#define EXAMPLE_LCD_IO_RGB_DATA8 (GPIO_NUM_15)
#define EXAMPLE_LCD_IO_RGB_DATA9 (GPIO_NUM_39)
#define EXAMPLE_LCD_IO_RGB_DATA10 (GPIO_NUM_16)
// R
#define EXAMPLE_LCD_IO_RGB_DATA11 (GPIO_NUM_4)
#define EXAMPLE_LCD_IO_RGB_DATA12 (GPIO_NUM_2)
#define EXAMPLE_LCD_IO_RGB_DATA13 (GPIO_NUM_5)
#define EXAMPLE_LCD_IO_RGB_DATA14 (GPIO_NUM_42)
#define EXAMPLE_LCD_IO_RGB_DATA15 (GPIO_NUM_6)

#define EXAMPLE_LCD_IO_RST (GPIO_NUM_10)        // -1 if not used
#define EXAMPLE_PIN_NUM_BK_LIGHT (GPIO_NUM_1) // -1 if not used
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL (1)
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL

// RGB通信时序相关参数，需要根据你的实际情况配置
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (16 * 1000 * 1000) // clk 12/16
#define EXAMPLE_HSYNC_PULSE_WIDTH 2                   // hpw 2
#define EXAMPLE_HSYNC_BACK_PORCH 8                    // hbp 8  
#define EXAMPLE_HSYNC_FRONT_PORCH 40                  // hfp 40 
#define EXAMPLE_VSYNC_PULSE_WIDTH 2                   // vpw 2  
#define EXAMPLE_VSYNC_BACK_PORCH 8                    // vbp 8  
#define EXAMPLE_VSYNC_FRONT_PORCH 12                  // vfp 12 

extern esp_lcd_panel_handle_t panel_handle; // 屏幕

void lcd_init(void);                                                                                                                     // 屏幕初始化
void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color);                                                                             // 画点函数
void lcd_clear(uint16_t color);                                                                                                          // 清屏
void lcd_fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color);                                               // 区域填充
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                                                  // 画线
void lcd_app_show_mono_icos(uint16_t x, uint16_t y, uint8_t width, uint8_t height, uint8_t *icosbase, uint16_t color, uint16_t bkcolor); // 单色图标
void lcd_draw_rectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);                                             // 画矩形
void lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);                                                               // 画圆
void lcd_show_char(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);                           // 显示字符
void lcd_show_string(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
void lcd_show_int_num(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size);
void lcd_show_float_num(uint16_t x, uint16_t y, float num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size);
void lcd_show_picture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[]);
void lcd_fill_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);
void lcd_show_chinese(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);

void lcd_draw_color_bars(void);
void lcd_draw_grayscale(void);
void lcd_draw_clear_button(void);
void lcd_draw_thick_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint8_t size);
// 画笔颜色
#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define BRED 0XF81F
#define GRED 0XFFE0
#define GBLUE 0X07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define BROWN 0XBC40      // 棕色
#define BRRED 0XFC07      // 棕红色
#define GRAY 0X8430       // 灰色
#define DARKBLUE 0X01CF   // 深蓝色
#define LIGHTBLUE 0X7D7C  // 浅蓝色
#define GRAYBLUE 0X5458   // 灰蓝色
#define LIGHTGREEN 0X841F // 浅绿色
#define LGRAY 0XC618      // 浅灰色(PANNEL),窗体背景色
#define LGRAYBLUE 0XA651  // 浅灰蓝色(中间层颜色)
#define LBBLUE 0X2B12     // 浅棕蓝色(选择条目的反色)

/* 宏定义 */
#define LOGO_DURATION 3000
#define TEXT_DURATION 2000
#define IMAGE_INTERVAL 2000
#define COLOR_FULL_INTERVAL 1000
#define EFFECT_DURATION 3000
#define BTN_WIDTH 100
#define BTN_HEIGHT 60

#define RGB(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))

#endif //__LCD_H