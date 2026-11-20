/**
 ******************************************************************************
 * @file        lcd_demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2026-11-20
 * @brief       P024BM057-IPS-CTP MIPI屏显示实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ******************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32-P4 开发板 + P024BM057-IPS-CTP MIPI转接板
 * 屏幕参数:2.4寸 BOE IPS, ST7701P, 480x640, MIPI DSI 2 lane
 *
 ******************************************************************************
 */

#include "lcd_demo.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd.h"
#include "mipi_lcd.h"
#include "nvs_flash.h"

typedef enum {
    STATE_INFO = 0,         /* 参数页 */
    STATE_COLOR_FULL,       /* 纯色循环: 检查色彩/坏点 */
    STATE_COLOR_BAR,        /* 彩条 */
    STATE_GRAYSCALE,        /* 16级灰阶 */
} demo_state_t;

static const char *TAG = "lcd_demo";

static const uint16_t s_full_colors[] = {RED, GREEN, BLUE, WHITE, BLACK};

static int64_t s_state_start_us;

static int64_t elapsed_ms(void)
{
    return (esp_timer_get_time() - s_state_start_us) / 1000;
}

static void next_state(demo_state_t *state, demo_state_t next)
{
    *state = next;
    s_state_start_us = esp_timer_get_time();
}

static void draw_info_page(void)
{
    char line[48];

    lcd_clear(BLACK);
    lcd_set_back_color(BLACK);

    lcd_show_string(24, 60, 432, 40, 32, "ST7701P MIPI", WHITE);
    lcd_show_string(24, 108, 432, 40, 32, "2.4 inch IPS", GREEN);
    lcd_show_string(24, 168, 432, 32, 24, "BOE P024BM057", CYAN);

    snprintf(line, sizeof(line), "%u x %u RGB888", (unsigned)lcddev.width, (unsigned)lcddev.height);
    lcd_show_string(24, 216, 432, 32, 24, line, YELLOW);

    snprintf(line, sizeof(line), "PCLK %u MHz", (unsigned)MIPI_LCD_PCLK_MHZ);
    lcd_show_string(24, 264, 432, 32, 24, line, YELLOW);

    snprintf(line, sizeof(line), "H %u/%u/%u  V %u/%u/%u",
             (unsigned)MIPI_LCD_HSW, (unsigned)MIPI_LCD_HBP, (unsigned)MIPI_LCD_HFP,
             (unsigned)MIPI_LCD_VSW, (unsigned)MIPI_LCD_VBP, (unsigned)MIPI_LCD_VFP);
    lcd_show_string(24, 312, 432, 32, 24, line, LGRAY);

    lcd_draw_rectangle(0, 0, (uint16_t)(lcddev.width - 1), (uint16_t)(lcddev.height - 1), WHITE);
}

static void draw_color_bars(void)
{
    const uint16_t bars[] = {RED, GREEN, BLUE, WHITE, CYAN, MAGENTA, YELLOW, BLACK};
    const uint32_t n = sizeof(bars) / sizeof(bars[0]);
    uint16_t bar_h = (uint16_t)(lcddev.height / n);

    for (uint32_t i = 0; i < n; i++)
    {
        uint16_t y0 = (uint16_t)(i * bar_h);
        uint16_t y1 = (i == (n - 1)) ? (uint16_t)lcddev.height : (uint16_t)((i + 1) * bar_h);
        lcd_fill(0, y0, (uint16_t)lcddev.width, y1, bars[i]);
    }
}

static void draw_grayscale(void)
{
    const uint16_t steps = 16;
    uint16_t bar_w = (uint16_t)(lcddev.width / steps);

    for (uint16_t i = 0; i < steps; i++)
    {
        uint8_t gray = (uint8_t)((i * 255) / (steps - 1));
        uint16_t color = (uint16_t)(((gray >> 3) << 11) | ((gray >> 2) << 5) | (gray >> 3));
        uint16_t x0 = (uint16_t)(i * bar_w);
        uint16_t x1 = (i == (steps - 1)) ? (uint16_t)lcddev.width : (uint16_t)((i + 1) * bar_w);
        lcd_fill(x0, 0, x1, (uint16_t)lcddev.height, color);
    }
}

void LCD_DEMO(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    lcd_init();

    if (lcddev.lcd_panel_handle == NULL)
    {
        ESP_LOGE(TAG, "panel init failed, stay here with backlight on");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "LCD %ux%u ready", (unsigned)lcddev.width, (unsigned)lcddev.height);

    demo_state_t state = STATE_INFO;
    bool state_drawn = false;
    uint8_t color_index = 0;

    s_state_start_us = esp_timer_get_time();

    while (1)
    {
        int64_t ms = elapsed_ms();

        switch (state)
        {
            case STATE_INFO:
                if (!state_drawn)
                {
                    draw_info_page();
                    state_drawn = true;
                }
                if (ms >= 3000)
                {
                    next_state(&state, STATE_COLOR_FULL);
                    state_drawn = false;
                }
                break;

            case STATE_COLOR_FULL:
                if (!state_drawn || ms >= 400)
                {
                    lcd_clear(s_full_colors[color_index]);
                    color_index = (uint8_t)((color_index + 1) % (sizeof(s_full_colors) / sizeof(s_full_colors[0])));
                    s_state_start_us = esp_timer_get_time();
                    state_drawn = true;
                }
                if (color_index == 0)
                {
                    next_state(&state, STATE_COLOR_BAR);
                    state_drawn = false;
                }
                break;

            case STATE_COLOR_BAR:
                if (!state_drawn)
                {
                    draw_color_bars();
                    state_drawn = true;
                }
                if (ms >= 1200)
                {
                    next_state(&state, STATE_GRAYSCALE);
                    state_drawn = false;
                }
                break;

            case STATE_GRAYSCALE:
                if (!state_drawn)
                {
                    draw_grayscale();
                    state_drawn = true;
                }
                if (ms >= 1200)
                {
                    next_state(&state, STATE_INFO);
                    state_drawn = false;
                }
                break;

            default:
                next_state(&state, STATE_INFO);
                state_drawn = false;
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(8));
    }
}
