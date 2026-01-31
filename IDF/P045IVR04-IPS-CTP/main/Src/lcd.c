#include "../Inc/lcd.h"
#include "esp_lcd_st7701.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io_additions.h"
#include "freertos/FreeRTOS.h"
#include "../Inc/lcdfont.h"

static const char *TAG = "RGB_LCD";
esp_lcd_panel_handle_t panel_handle = NULL;                     // 屏幕
static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED; /* 定义 portMUX_TYPE 类型的自旋锁变量, 用于临界区保护 */

// st7701初始化指令
static const st7701_lcd_init_cmd_t lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0x11, (uint8_t[]){}, 0, 120}, // Sleep Out

    //{0x36, (uint8_t[]){0x00}, 1, 0},

    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
    {0xC0, (uint8_t[]){0xE9, 0x03}, 2, 0},
    {0xC1, (uint8_t[]){0x12, 0x02}, 2, 0},
    {0xC2, (uint8_t[]){0x31, 0x08}, 2, 0},
    {0xCC, (uint8_t[]){0x10}, 1, 0},

    {0xB0, (uint8_t[]){0x00, 0x0A, 0x13, 0x0E, 0x12, 0x07, 0x05, 0x08, 0x08, 0x1F, 0x07, 0x15, 0x13, 0xE3, 0x2A, 0x11}, 16, 0},
    {0xB1, (uint8_t[]){0x00, 0x0A, 0x12, 0x0E, 0x12, 0x07, 0x04, 0x07, 0x07, 0x1E, 0x04, 0x13, 0x10, 0x23, 0x29, 0x11}, 16, 0},

    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
    {0xB0, (uint8_t[]){0x4D}, 1, 0},
    {0xB1, (uint8_t[]){0x1C}, 1, 0},
    {0xB2, (uint8_t[]){0x07}, 1, 0},
    {0xB3, (uint8_t[]){0x80}, 1, 0},
    {0xB5, (uint8_t[]){0x47}, 1, 0},
    {0xB7, (uint8_t[]){0x85}, 1, 0},
    {0xB8, (uint8_t[]){0x21}, 1, 0},
    {0xB9, (uint8_t[]){0x10}, 1, 0},
    {0xC1, (uint8_t[]){0x78}, 1, 0},
    {0xC2, (uint8_t[]){0x78}, 1, 0},
    {0xD0, (uint8_t[]){0x88}, 1, 0},

    {0xE0, (uint8_t[]){0x00, 0x00, 0x02}, 3, 0},
    {0xE1, (uint8_t[]){0x0B, 0x00, 0x0D, 0x00, 0x0C, 0x00, 0x0E, 0x00, 0x00, 0x44, 0x44}, 11, 0},
    {0xE2, (uint8_t[]){0x33, 0x33, 0x44, 0x44, 0x64, 0x00, 0x66, 0x00, 0x65, 0x00, 0x67, 0x00, 0x00}, 13, 0},
    {0xE3, (uint8_t[]){0x00, 0x00, 0x33, 0x33}, 4, 0},
    {0xE4, (uint8_t[]){0x44, 0x44}, 2, 0},
    {0xE5, (uint8_t[]){0x0C, 0x78, 0xA0, 0xA0, 0x0E, 0x78, 0xA0, 0xA0, 0x10, 0x78, 0xA0, 0xA0, 0x12, 0x78, 0xA0, 0xA0}, 16, 0},

    {0xE6, (uint8_t[]){0x00, 0x00, 0x33, 0x33}, 4, 0},
    {0xE7, (uint8_t[]){0x44, 0x44}, 2, 0},
    {0xE8, (uint8_t[]){0x0D, 0x78, 0xA0, 0xA0, 0x0F, 0x78, 0xA0, 0xA0, 0x11, 0x78, 0xA0, 0xA0, 0x13, 0x78, 0xA0, 0xA0}, 16, 0},

    {0xEB, (uint8_t[]){0x02, 0x00, 0x39, 0x39, 0xEE, 0x44, 0x00}, 7, 0},
    {0xEC, (uint8_t[]){0x00, 0x00}, 2, 0},
    {0xED, (uint8_t[]){0xFF, 0xF1, 0x04, 0x56, 0x72, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF3, 0x27, 0x65, 0x40, 0x1F, 0xFF}, 16, 0},

    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},

    {0x29, (uint8_t[]){}, 0, 20}, // Display On
};

void lcd_init(void)
{
    // 背光初始化
    if (EXAMPLE_PIN_NUM_BK_LIGHT >= 0)
    {
        ESP_LOGI(TAG, "Turn off LCD backlight");
        gpio_config_t bk_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT};
        ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    }

    // 初始化SPI
    ESP_LOGI(TAG, "Install 3-wire SPI panel IO");
    spi_line_config_t line_config = {
        .cs_io_type = IO_TYPE_GPIO, // Set to `IO_TYPE_GPIO` if using GPIO, same to below
        .cs_gpio_num = EXAMPLE_LCD_IO_SPI_CS,
        .scl_io_type = IO_TYPE_GPIO,
        .scl_gpio_num = EXAMPLE_LCD_IO_SPI_SCL,
        .sda_io_type = IO_TYPE_GPIO,
        .sda_gpio_num = EXAMPLE_LCD_IO_SPI_SDA,
        .io_expander = NULL, // Set to NULL if not using IO expander
    };
    esp_lcd_panel_io_3wire_spi_config_t io_config = ST7701_PANEL_IO_3WIRE_SPI_CONFIG(line_config, 0);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle));

    // 初始化
    ESP_LOGI(TAG, "Install ST7701 panel driver");
    esp_lcd_rgb_panel_config_t rgb_config = {
        .clk_src = LCD_CLK_SRC_PLL240M,       // 时钟选择
        .psram_trans_align = 64,              // 对齐方式
        .data_width = EXAMPLE_RGB_DATA_WIDTH, // 数据宽度16位RGB565
        .de_gpio_num = EXAMPLE_LCD_IO_RGB_DE,
        .pclk_gpio_num = EXAMPLE_LCD_IO_RGB_PCLK,
        .vsync_gpio_num = EXAMPLE_LCD_IO_RGB_VSYNC,
        .hsync_gpio_num = EXAMPLE_LCD_IO_RGB_HSYNC,
        .disp_gpio_num = EXAMPLE_LCD_IO_RGB_DISP,
        .data_gpio_nums = {
            EXAMPLE_LCD_IO_RGB_DATA0,
            EXAMPLE_LCD_IO_RGB_DATA1,
            EXAMPLE_LCD_IO_RGB_DATA2,
            EXAMPLE_LCD_IO_RGB_DATA3,
            EXAMPLE_LCD_IO_RGB_DATA4,
            EXAMPLE_LCD_IO_RGB_DATA5,
            EXAMPLE_LCD_IO_RGB_DATA6,
            EXAMPLE_LCD_IO_RGB_DATA7,
            EXAMPLE_LCD_IO_RGB_DATA8,
            EXAMPLE_LCD_IO_RGB_DATA9,
            EXAMPLE_LCD_IO_RGB_DATA10,
            EXAMPLE_LCD_IO_RGB_DATA11,
            EXAMPLE_LCD_IO_RGB_DATA12,
            EXAMPLE_LCD_IO_RGB_DATA13,
            EXAMPLE_LCD_IO_RGB_DATA14,
            EXAMPLE_LCD_IO_RGB_DATA15,
        },
        .timings = {
            .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
            .h_res = EXAMPLE_LCD_H_RES,
            .v_res = EXAMPLE_LCD_V_RES,
            .hsync_pulse_width = EXAMPLE_HSYNC_PULSE_WIDTH,
            .hsync_back_porch = EXAMPLE_HSYNC_BACK_PORCH,
            .hsync_front_porch = EXAMPLE_HSYNC_FRONT_PORCH,
            .vsync_pulse_width = EXAMPLE_VSYNC_PULSE_WIDTH,
            .vsync_back_porch = EXAMPLE_VSYNC_BACK_PORCH,
            .vsync_front_porch = EXAMPLE_VSYNC_FRONT_PORCH,
            .flags = {
                // 由于一些 LCD 可以通过硬件引脚或者软件命令配置这些参数，需要确保它们与配置保持一致，但通常情况下均为 `0`
                .hsync_idle_low = 0,      // HSYNC 信号空闲时的电平，0：高电平，1：低电平
                .vsync_idle_low = 0,      // VSYNC 信号空闲时的电平，0 表示高电平，1：低电平
                .de_idle_high = 0,        // DE 信号空闲时的电平，0：高电平，1：低电平
                .pclk_active_neg = false, // 时钟信号的有效边沿，0：上升沿有效，1：下降沿有效
                .pclk_idle_high = 0,      // PCLK 信号空闲时的电平，0：高电平，1：低电平
            },
        },
        .flags.fb_in_psram = true,
    };

    // SPI命令结构体
    st7701_vendor_config_t vendor_config = {
        .rgb_config = &rgb_config,
        .init_cmds = lcd_init_cmds, // Uncomment these line if use custom initialization commands
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(st7701_lcd_init_cmd_t),
        .flags = {
            .auto_del_panel_io = 0, /**
                                     * Set to 1 if panel IO is no longer needed after LCD initialization.
                                     * If the panel IO pins are sharing other pins of the RGB interface to save GPIOs,
                                     * Please set it to 1 to release the pins.
                                     */
            .mirror_by_cmd = 1,     // Set to 0 if `auto_del_panel_io` is enabled
        },
    };

    // 屏幕控制结构体
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_LCD_IO_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = EXAMPLE_LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };

    // 初始化ST7701S
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7701(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    esp_lcd_panel_disp_on_off(panel_handle, true);

    if (EXAMPLE_PIN_NUM_BK_LIGHT >= 0)
    {
        ESP_LOGI(TAG, "Turn on LCD backlight");
        gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
    }

    ESP_LOGI(TAG, "Display LVGL demos");
}

/**
 * @brief      RGB画点函数
 * @param       x,y     :写入坐标
 * @param       color   :颜色值
 * @retval      无
 */
void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    taskENTER_CRITICAL(&my_spinlock);
    esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + 1, y + 1, &color);
    taskEXIT_CRITICAL(&my_spinlock);
}

/**
 * @brief      RGB清屏函数
 * @param       color   :颜色值
 * @retval      无
 */
void lcd_clear(uint16_t color)
{
    uint16_t *buffer = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint32_t count = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES;

    if (NULL == buffer)
    {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
    }
    else
    {
        for (uint32_t i = 0; i < count; i++)
        {
            buffer[i] = color;
        }

        /* 使用taskENTER_CRITICAL()和taskEXIT_CRITICAL()保护画点过程,禁止任务调度 */
        taskENTER_CRITICAL(&my_spinlock); /* 屏蔽中断 */
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, buffer);
        taskEXIT_CRITICAL(&my_spinlock); /* 重新使能中断 */

        heap_caps_free(buffer);
    }
}

/**
 * @brief      在指定区域内填充指定颜色块
 * @param       xsta,ysta:起始坐标
 * @param       xend,yend:结束坐标
 * @param       color:填充的颜色数组首地址
 * @retval      无
 */
void lcd_fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    /* 确保坐标在LCD范围内 */
    if (xend > EXAMPLE_LCD_H_RES || yend > EXAMPLE_LCD_V_RES)
    {
        return; /* 坐标超出LCD范围，不执行填充 */
    }

    /* 确保起始坐标小于结束坐标 */
    if (xsta > xend || ysta > yend)
    {
        return; /* 无效的填充区域，不执行填充 */
    }

    uint16_t *buffer = heap_caps_malloc((xend - xsta) * (yend - ysta) * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint32_t count = (xend - xsta) * (yend - ysta);

    if (NULL == buffer)
    {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
    }
    else
    {
        for (uint32_t i = 0; i < count; i++)
        {
            buffer[i] = color;
        }
        /* 使用taskENTER_CRITICAL()和taskEXIT_CRITICAL()保护画点过程,禁止任务调度 */
        taskENTER_CRITICAL(&my_spinlock); /* 屏蔽中断 */
        esp_lcd_panel_draw_bitmap(panel_handle, xsta, ysta, xend, yend, buffer);
        taskEXIT_CRITICAL(&my_spinlock); /* 重新使能中断 */

        heap_caps_free(buffer);
    }
}

/**
 * @brief       画线
 * @param       x1,y1:起点坐标
 * @param       x2,y2:终点坐标
 * @param       color:线的颜色
 * @retval      无
 */
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, row, col;
    delta_x = x2 - x1; /* 计算坐标增量 */
    delta_y = y2 - y1;
    row = x1;
    col = y1;

    if (delta_x > 0)
    {
        incx = 1; /* 设置单步方向 */
    }
    else if (delta_x == 0)
    {
        incx = 0; /* 垂直线 */
    }
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0)
    {
        incy = 1;
    }
    else if (delta_y == 0)
    {
        incy = 0; /* 水平线 */
    }
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }

    if (delta_x > delta_y)
    {
        distance = delta_x; /* 选取基本增量坐标轴 */
    }
    else
    {
        distance = delta_y;
    }

    for (t = 0; t <= distance + 1; t++) /* 画线输出 */
    {
        lcd_draw_point(row, col, color); /* 画点 */
        xerr += delta_x;
        yerr += delta_y;

        if (xerr > distance)
        {
            xerr -= distance;
            row += incx;
        }

        if (yerr > distance)
        {
            yerr -= distance;
            col += incy;
        }
    }
}

/**
 * @brief       显示单色图标
 * @param       x,y,width,height:坐标及尺寸
 * @param       icosbase:点整位置
 * @param       color:画点的颜色
 * @param       bkcolor:背景色
 * @retval      无
 */
void lcd_app_show_mono_icos(uint16_t x, uint16_t y, uint8_t width, uint8_t height, uint8_t *icosbase, uint16_t color, uint16_t bkcolor)
{
    uint16_t rsize;
    uint16_t i, j;
    uint8_t temp;
    uint8_t t = 0;
    uint16_t x0 = x;                           // 保留x的位置
    rsize = width / 8 + ((width % 8) ? 1 : 0); // 每行的字节数

    for (i = 0; i < rsize * height; i++)
    {
        temp = icosbase[i];

        for (j = 0; j < 8; j++)
        {
            if (temp & 0x80)
            {
                lcd_draw_point(x, y, color);
            }
            else
            {
                lcd_draw_point(x, y, bkcolor);
            }

            temp <<= 1;
            x++;
            t++; // 宽度计数器

            if (t == width)
            {
                t = 0;
                x = x0;
                y++;
                break;
            }
        }
    }
}

/**
 * @brief       画一个矩形
 * @param       x1,y1   起点坐标
 * @param       x2,y2   终点坐标
 * @param       color 填充颜色
 * @retval      无
 */
void lcd_draw_rectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    lcd_draw_line(x0, y0, x1, y0, color);
    lcd_draw_line(x0, y0, x0, y1, color);
    lcd_draw_line(x0, y1, x1, y1, color);
    lcd_draw_line(x1, y0, x1, y1, color);
}

/**
 * @brief       画圆
 * @param       x0,y0:圆中心坐标
 * @param       r    :半径
 * @param       color:圆的颜色
 * @retval      无
 */
void lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    int a, b;
    int di;
    a = 0;
    b = r;
    di = 3 - (r << 1); /* 判断下个点位置的标志 */

    while (a <= b)
    {
        lcd_draw_point(x0 + a, y0 - b, color); /* 5 */
        lcd_draw_point(x0 + b, y0 - a, color); /* 0 */
        lcd_draw_point(x0 + b, y0 + a, color); /* 4 */
        lcd_draw_point(x0 + a, y0 + b, color); /* 6 */
        lcd_draw_point(x0 - a, y0 + b, color); /* 1 */
        lcd_draw_point(x0 - b, y0 + a, color);
        lcd_draw_point(x0 - a, y0 - b, color); /* 2 */
        lcd_draw_point(x0 - b, y0 - a, color); /* 7 */
        a++;

        /* 使用Bresenham算法画圆 */
        if (di < 0)
        {
            di += 4 * a + 6;
        }
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }
    }
}

/**
 * @brief       在指定位置显示一个字符
 * @param       x,y  :坐标
 * @param       num  :要显示的字符:" "--->"~"
 * @param       fc   ：字符颜色
 * @param       bc   ：背景颜色
 * @param       size ：字体大小 12/16/24/32
 * @param       mode :叠加方式(1); 非叠加方式(0);
 * @retval      无
 */
void lcd_show_char(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
    uint8_t temp, t1, t;
    uint16_t x0 = x;
    uint8_t csize = 0;
    uint8_t *pfont = NULL;
    uint8_t sizex = size / 2;

    csize = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * size; /* 得到字体一个字符对应点阵集所占的字节数 */
    num = (char)num - ' ';                              /* 得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库） */

    switch (size)
    {
    case 12:
        pfont = (uint8_t *)ascii_1206[(uint8_t)num]; /* 调用1206字体 */
        break;

    case 16:
        pfont = (uint8_t *)ascii_1608[(uint8_t)num]; /* 调用1608字体 */
        break;

    case 24:
        pfont = (uint8_t *)ascii_2412[(uint8_t)num]; /* 调用2412字体 */
        break;

    case 32:
        pfont = (uint8_t *)ascii_3216[(uint8_t)num]; /* 调用3216字体 */
        break;

    default:
        return;
    }

    if (pfont == NULL)
    {
        return;
    }

    for (t = 0; t < csize; t++)
    {
        temp = pfont[t]; /* 获取字符的点阵数据 */

        for (t1 = 0; t1 < 8; t1++) /* 一个字节8个点 */
        {
            if (temp & (0x01 << t1)) /* 有效点,需要显示 */
            {
                lcd_draw_point(x, y, fc); /* 画点出来,要显示这个点 */
            }
            else if (mode == 0) /* 无效点,不显示 */
            {
                lcd_draw_point(x, y, bc); /* 画背景色,相当于这个点不显示(注意背景色由全局变量控制) */
            }

            x++;

            if (x >= EXAMPLE_LCD_H_RES)
                return; /* 超区域了 */

            if ((x - x0) == sizex) /* 显示完一列了? */
            {
                x = x0; /* y坐标复位 */
                y++;    /* x坐标递增 */

                if (y >= EXAMPLE_LCD_V_RES)
                {
                    return; /* x坐标超区域了 */
                }

                break;
            }
        }
    }
}

/**
 * @brief       在指定位置显示字符串
 * @param       x,y  :坐标
 * @param       *p   :要显示的字符串
 * @param       fc   ：字符颜色
 * @param       bc   ：背景颜色
 * @param       size ：字体大小 12/16/24/32
 * @param       mode :叠加方式(1); 非叠加方式(0);
 * @retval      无
 */
void lcd_show_string(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
    while (*p != '\0')
    {
        lcd_show_char(x, y, *p, fc, bc, size, mode);
        x += size / 2;
        p++;
    }
}

/**
 * @brief       平方函数, m^n
 * @param       m:底数
 * @param       n:指数
 * @retval      m的n次方
 */
static uint32_t lcd_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;

    while (n--)
    {
        result *= m;
    }

    return result;
}

/**
 * @brief       显示len个数字
 * @param       x,y     :起始坐标
 * @param       num     :数值(0 ~ 2^16)
 * @param       len     :显示数字的位数
 * @param       fc      :字的颜色
 * @param       bc      :字的背景色
 * @param       size    :选择字体 12/16/24/32
 * @retval      无
 */
void lcd_show_int_num(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++) /* 按总显示位数循环 */
    {
        temp = (num / lcd_pow(10, len - t - 1)) % 10; /* 获取对应位的数字 */

        if (enshow == 0 && t < (len - 1)) /* 没有使能显示,且还有位要显示 */
        {
            if (temp == 0)
            {
                lcd_show_char(x + t * (size / 2), y, ' ', fc, bc, size, 0);
                continue; /* 继续下个一位 */
            }
            else
            {
                enshow = 1; /* 使能显示 */
            }
        }

        lcd_show_char(x + t * (size / 2), y, temp + '0', fc, bc, size, 0); /* 显示字符 */
    }
}

/**
 * @brief       在指定位置显示len长度，小数显示两位的数
 * @param       x,y  :坐标
 * @param       num  :要显示的字符串
 * @param       len  :要显示的长度
 * @param       fc   ：字符颜色
 * @param       bc   ：背景颜色
 * @param       sizey ：字体大小 12/16/24/32
 * @retval      无
 */
void lcd_show_float_num(uint16_t x, uint16_t y, float num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size)
{
    uint8_t t, temp;
    uint16_t num1;
    num1 = num * 100;
    for (t = 0; t < len; t++)
    {
        temp = (num1 / lcd_pow(10, len - t - 1)) % 10;
        if (t == (len - 2))
        {
            lcd_show_char(x + (len - 2) * (size / 2), y, '.', fc, bc, size, 0);
            t++;
            len += 1;
        }
        lcd_show_char(x + t * (size / 2), y, temp + '0', fc, bc, size, 0);
    }
}

/**
 * @brief       在指定位置显示图片
 * @param       x,y  :坐标
 * @param       length  :图片长度
 * @param       width  :图片宽度
 * @param       pic   ：图片数据
 * @retval      无
 */
void lcd_show_picture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[])
{
    // 计算显示区域的结束坐标
    uint16_t x_end = x + length;
    uint16_t y_end = y + width;

    uint8_t *buffer = heap_caps_malloc(length * width * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint32_t count = length * width;

    if (NULL == buffer)
    {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
    }
    else
    {
        for (uint32_t i = 0; i < count; i++)
        {
            buffer[i * 2] = pic[i * 2 + 1];
            buffer[i * 2 + 1] = pic[i * 2];
        }
        /* 使用taskENTER_CRITICAL()和taskEXIT_CRITICAL()保护画点过程,禁止任务调度 */
        taskENTER_CRITICAL(&my_spinlock); /* 屏蔽中断 */
        esp_lcd_panel_draw_bitmap(panel_handle, x, y, x_end, y_end, buffer);
        taskEXIT_CRITICAL(&my_spinlock); /* 重新使能中断 */

        heap_caps_free(buffer);
    }
}

/**
 * @brief       绘制颜色条
 * @param       无
 * @retval      无
 */
void lcd_draw_color_bars(void)
{
    uint16_t colors[] = {RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, BLACK, WHITE};

    for (int i = 0; i < 8; i++)
    {
        lcd_fill(i * (EXAMPLE_LCD_H_RES / 8), 0, ((i + 1) * (EXAMPLE_LCD_H_RES / 8)), EXAMPLE_LCD_V_RES, colors[i]);
    }
}

/**
 * @brief       绘制灰度渐变
 * @param       无
 * @retval      无
 */
void lcd_draw_grayscale(void)
{
#define GRAY_LEVELS 24
    const uint16_t level_width = EXAMPLE_LCD_H_RES / GRAY_LEVELS;
    const uint16_t remainder = EXAMPLE_LCD_H_RES % GRAY_LEVELS;

    for (uint8_t n = 0; n < GRAY_LEVELS; n++)
    {
        // 计算当前灰度级区域范围
        uint16_t start_x = n * level_width;
        uint16_t end_x = start_x + level_width - 1;

        // 处理余数像素，加在最后一级
        if (n == GRAY_LEVELS - 1)
        {
            end_x += remainder;
        }

        // 计算24级灰度值 (0~23 → 0~255)
        uint8_t gray = (n * 255) / (GRAY_LEVELS - 1);

        // 生成RGB565颜色（需确保已实现RGB宏）
        uint16_t color = RGB(gray, gray, gray);

        // 填充灰度带区域
        lcd_fill(start_x, 0, end_x, EXAMPLE_LCD_V_RES - 1, color);
    }
}

/**
 * @brief       绘制清屏按钮
 * @param       无
 * @retval      无
 */
void lcd_draw_clear_button(void)
{
    lcd_fill(EXAMPLE_LCD_H_RES - BTN_WIDTH, EXAMPLE_LCD_V_RES - BTN_HEIGHT,
             EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, DARKBLUE);
    lcd_show_string(EXAMPLE_LCD_H_RES - BTN_WIDTH + 5, EXAMPLE_LCD_V_RES - BTN_HEIGHT + 8,
                    (const uint8_t *)"Clear", BLACK, DARKBLUE, 32, 0);
}

/**
 * @brief  在指定位置填充一个圆
 * @param  x0,y0: 圆心坐标
 * @param  r: 圆的半径
 * @param  color: 填充颜色
 * @retval 无
 */
void lcd_fill_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    if (r == 0)
        return;

    int16_t x = r;
    int16_t y = 0;
    int16_t err = 0;

    while (x >= y)
    {
        // 填充水平线
        lcd_fill(x0 - x, y0 + y, x0 + x, y0 + y, color);
        lcd_fill(x0 - y, y0 + x, x0 + y, y0 + x, color);
        lcd_fill(x0 - x, y0 - y, x0 + x, y0 - y, color);
        lcd_fill(x0 - y, y0 - x, x0 + y, y0 - x, color);

        if (err <= 0)
        {
            y += 1;
            err += 2 * y + 1;
        }
        if (err > 0)
        {
            x -= 1;
            err -= 2 * x + 1;
        }
    }
}

/**
 * @brief  显示单个12x12汉字
 * @param  x,y :显示坐标
 * @param  *s :要显示的汉字
 * @param  fc :字的颜色
 * @param  bc :背景颜色
 * @param  sizey :字号
 * @param  mode:  0非叠加模式  1叠加模式
 * @retval 无
 */
void lcd_show_chinese12x12(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t i, j;
    uint16_t k;
    uint16_t HZnum;       // 汉字数目
    uint16_t TypefaceNum; // 一个字符所占字节大小
    uint16_t x0 = x;
    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;

    HZnum = sizeof(tfont12) / sizeof(typFNT_GB12); // 统计汉字数目
    for (k = 0; k < HZnum; k++)
    {
        if ((tfont12[k].Index[0] == *(s)) && (tfont12[k].Index[1] == *(s + 1)) && (tfont12[k].Index[2] == *(s + 2)))
        {
            for (i = 0; i < TypefaceNum; i++)
            {
                for (j = 0; j < 8; j++)
                {
                    if (!mode) // 非叠加方式
                    {
                        if (tfont12[k].Msk[i] & (0x01 << j))
                            lcd_draw_point(x, y, fc);
                        else
                            lcd_draw_point(x, y, bc);
                    }
                    else // 叠加方式
                    {
                        if (tfont12[k].Msk[i] & (0x01 << j))
                            lcd_draw_point(x, y, fc); // 画一个点
                    }
                    x++;
                    if ((x - x0) == sizey)
                    {
                        x = x0;
                        y++;
                        break;
                    }
                }
            }
        }
        continue; // 查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
    }
}

/**
 * @brief  显示单个16x16汉字
 * @param  x,y :显示坐标
 * @param  *s :要显示的汉字
 * @param  fc :字的颜色
 * @param  bc :背景颜色
 * @param  sizey :字号
 * @param  mode:  0非叠加模式  1叠加模式
 * @retval 无
 */
void lcd_show_chinese16x16(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t i, j;
    uint16_t k;
    uint16_t HZnum;       // 汉字数目
    uint16_t TypefaceNum; // 一个字符所占字节大小
    uint16_t x0 = x;
    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;

    HZnum = sizeof(tfont16) / sizeof(typFNT_GB16); // 统计汉字数目
    for (k = 0; k < HZnum; k++)
    {
        if ((tfont16[k].Index[0] == *(s)) && (tfont16[k].Index[1] == *(s + 1)) && (tfont16[k].Index[2] == *(s + 2)))
        {
            for (i = 0; i < TypefaceNum; i++)
            {
                for (j = 0; j < 8; j++)
                {
                    if (!mode) // 非叠加方式
                    {
                        if (tfont16[k].Msk[i] & (0x01 << j))
                            lcd_draw_point(x, y, fc);
                        else
                            lcd_draw_point(x, y, bc);
                    }
                    else // 叠加方式
                    {
                        if (tfont16[k].Msk[i] & (0x01 << j))
                            lcd_draw_point(x, y, fc); // 画一个点
                    }
                    x++;
                    if ((x - x0) == sizey)
                    {
                        x = x0;
                        y++;
                        break;
                    }
                }
            }
        }
        continue; // 查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
    }
}

/**
 * @brief  显示单个24x24汉字
 * @param  x,y :显示坐标
 * @param  *s :要显示的汉字
 * @param  fc :字的颜色
 * @param  bc :背景颜色
 * @param  sizey :字号
 * @param  mode:  0非叠加模式  1叠加模式
 * @retval 无
 */
void lcd_show_chinese24x24(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t i, j;
    uint16_t k;
    uint16_t HZnum;       // 汉字数目
    uint16_t TypefaceNum; // 一个字符所占字节大小
    uint16_t x0 = x;
    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;

    HZnum = sizeof(tfont24) / sizeof(typFNT_GB24); // 统计汉字数目
    for (k = 0; k < HZnum; k++)
    {
        if ((tfont24[k].Index[0] == *(s)) && (tfont24[k].Index[1] == *(s + 1)) && (tfont24[k].Index[2] == *(s + 2)))
        {
            for (i = 0; i < TypefaceNum; i++)
            {
                for (j = 0; j < 8; j++)
                {
                    if (!mode) // 非叠加方式
                    {
                        if (tfont24[k].Msk[i] & (0x01 << j))
                            lcd_draw_point(x, y, fc);
                        else
                            lcd_draw_point(x, y, bc);
                    }
                    else // 叠加方式
                    {
                        if (tfont24[k].Msk[i] & (0x01 << j))
                            lcd_draw_point(x, y, fc); // 画一个点
                    }
                    x++;
                    if ((x - x0) == sizey)
                    {
                        x = x0;
                        y++;
                        break;
                    }
                }
            }
        }
        continue; // 查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
    }
}

/**
 * @brief  显示单个32x32汉字
 * @param  x,y :显示坐标
 * @param  *s :要显示的汉字
 * @param  fc :字的颜色
 * @param  bc :背景颜色
 * @param  sizey :字号
 * @param  mode:  0非叠加模式  1叠加模式
 * @retval 无
 */
void lcd_show_chinese32x32(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t i, j;
    uint16_t k;
    uint16_t HZnum;       // 汉字数目
    uint16_t TypefaceNum; // 一个字符所占字节大小
    uint16_t x0 = x;
    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;

    HZnum = sizeof(tfont32) / sizeof(typFNT_GB32); // 统计汉字数目
    for (k = 0; k < HZnum; k++)
    {
        if ((tfont32[k].Index[0] == *(s)) && (tfont32[k].Index[1] == *(s + 1)) && (tfont32[k].Index[2] == *(s + 2)))
        {
            for (i = 0; i < TypefaceNum; i++)
            {
                for (j = 0; j < 8; j++)
                {
                    if (!mode) // 非叠加方式
                    {
                        if (tfont32[k].Msk[i] & (0x01 << j))
                            lcd_draw_point(x, y, fc);
                        else
                            lcd_draw_point(x, y, bc);
                    }
                    else // 叠加方式
                    {
                        if (tfont32[k].Msk[i] & (0x01 << j))
                            lcd_draw_point(x, y, fc); // 画一个点
                    }
                    x++;
                    if ((x - x0) == sizey)
                    {
                        x = x0;
                        y++;
                        break;
                    }
                }
            }
        }
        continue; // 查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
    }
}

/**
 * @brief  显示汉字串
 * @param  x,y :显示坐标
 * @param  *s :要显示的汉字串
 * @param  fc :字的颜色
 * @param  bc :背景颜色
 * @param  sizey :字号
 * @param  mode:  0非叠加模式  1叠加模式
 * @retval 无
 */
void lcd_show_chinese(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    while (*s != 0)
    {
        if (sizey == 12)
            lcd_show_chinese12x12(x, y, s, fc, bc, sizey, mode);
        else if (sizey == 16)
            lcd_show_chinese16x16(x, y, s, fc, bc, sizey, mode);
        else if (sizey == 24)
            lcd_show_chinese24x24(x, y, s, fc, bc, sizey, mode);
        else if (sizey == 32)
            lcd_show_chinese32x32(x, y, s, fc, bc, sizey, mode);
        else
            return;
        s += 3;
        x += sizey;
    }
}

// 电容触摸屏专有部分
// 画水平线
// x0,y0:坐标
// len:线长度
// color:颜色
void gui_draw_hline(uint16_t x0, uint16_t y0, uint16_t len, uint16_t color)
{
    if (len == 0)
        return;
    lcd_draw_line(x0, y0, x0 + len - 1, y0, color);
}

void gui_fill_circle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
    uint32_t i;
    uint32_t imax = (r * 724) >> 10; // r * 707/1000 ≈ r * 724/1024
    uint32_t sqmax = r * r + (r >> 1);
    uint32_t x = r;
    uint32_t i_squared = 1; // 1^2 = 1

    gui_draw_hline(x0 - r, y0, 2 * r, color);

    for (i = 1; i < imax + 1; i++)
    {
        if ((i_squared + x * x) > sqmax)
        {
            if (x > imax)
            {
                gui_draw_hline(x0 - i + 1, y0 + x, 2 * (i - 1), color);
                gui_draw_hline(x0 - i + 1, y0 - x, 2 * (i - 1), color);
            }
            x--;
        }
        // 绘制内部线
        gui_draw_hline(x0 - x, y0 + i, 2 * x, color);
        gui_draw_hline(x0 - x, y0 - i, 2 * x, color);

        i_squared += (i << 1) + 1; // 计算下一个i的平方
    }
}

void lcd_draw_thick_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint8_t size)
{
    if (size == 1)
    {
        lcd_draw_line(x1, y1, x2, y2, color);
        return;
    }

    // 快速边界检查
    if (x1 < size || x2 < size || y1 < size || y2 < size)
        return;

    int16_t dx = x2 - x1;
    int16_t dy = y2 - y1;

    // 快速距离检查（近似）
    uint16_t abs_dx = dx > 0 ? dx : -dx;
    uint16_t abs_dy = dy > 0 ? dy : -dy;
    if ((abs_dx > 50) || (abs_dy > 50))
    {
        return;
    }

    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    if (x1 < size || x2 < size || y1 < size || y2 < size)
        return;
    delta_x = x2 - x1; // 计算坐标增量
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    if (delta_x > 0)
        incx = 1; // 设置单步方向
    else if (delta_x == 0)
        incx = 0; // 垂直线
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0; // 水平线
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }
    if (delta_x > delta_y)
        distance = delta_x; // 选取基本增量坐标轴
    else
        distance = delta_y;
    for (t = 0; t <= distance + 1; t++) // 画线输出
    {
        gui_fill_circle(uRow, uCol, size, color); // 画点
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}