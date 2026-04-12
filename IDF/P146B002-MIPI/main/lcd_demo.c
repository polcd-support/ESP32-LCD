#include "lcd_demo.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "cst816.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd.h"
#include "led.h"
#include "nvs_flash.h"

typedef enum {
    STATE_LOGO = 0,
    STATE_TEXT,
    STATE_COLOR_FULL,
    STATE_COLOR_BAR,
    STATE_GRAYSCALE,
    STATE_HANDWRITING,
} demo_state_t;

static const char *TAG = "lcd_demo";
static const uint16_t s_full_colors[] = {RED, GREEN, BLUE, WHITE, BLACK};

#define BTN_WIDTH      92
#define BTN_HEIGHT     42
#define LINE_THICKNESS 2

static int64_t s_state_start_us;
static uint8_t s_color_index;

static void draw_thick_point(int x, int y, uint16_t color, uint8_t thickness)
{
    if (x < 0 || y < 0 || x >= (int)lcddev.width || y >= (int)lcddev.height) {
        return;
    }
    lcd_fill_circle((uint16_t)x, (uint16_t)y, thickness, color);
}

static void draw_thick_line(int x0, int y0, int x1, int y1, uint16_t color, uint8_t thickness)
{
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = -((y1 > y0) ? (y1 - y0) : (y0 - y1));
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (1) {
        draw_thick_point(x0, y0, color, thickness);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void draw_color_bars(void)
{
    const uint16_t bars[] = {RED, YELLOW, GREEN, CYAN, BLUE, MAGENTA};
    const uint32_t n = sizeof(bars) / sizeof(bars[0]);
    uint16_t bar_h = (uint16_t)(lcddev.height / n);

    for (uint32_t i = 0; i < n; i++) {
        uint16_t y0 = (uint16_t)(i * bar_h);
        uint16_t y1 = (i == n - 1) ? (uint16_t)lcddev.height : (uint16_t)((i + 1) * bar_h);
        lcd_fill(0, y0, (uint16_t)lcddev.width, y1, bars[i]);
    }
}

static void draw_grayscale(void)
{
    const uint16_t steps = 16;
    uint16_t bar_w = (uint16_t)(lcddev.width / steps);

    for (uint16_t i = 0; i < steps; i++) {
        uint8_t gray = (uint8_t)((i * 255) / (steps - 1));
        uint16_t color = (uint16_t)(((gray >> 3) << 11) | ((gray >> 2) << 5) | (gray >> 3));
        uint16_t x0 = (uint16_t)(i * bar_w);
        uint16_t x1 = (i == steps - 1) ? (uint16_t)lcddev.width : (uint16_t)((i + 1) * bar_w);
        lcd_fill(x0, 0, x1, (uint16_t)lcddev.height, color);
    }
}

static void draw_clear_button(void)
{
    uint16_t x0 = (uint16_t)(lcddev.width - BTN_WIDTH);
    uint16_t y0 = (uint16_t)(lcddev.height - BTN_HEIGHT);

    lcd_fill(x0, y0, (uint16_t)lcddev.width, (uint16_t)lcddev.height, BLUE);
    lcd_draw_rectangle(x0, y0, (uint16_t)(lcddev.width - 1), (uint16_t)(lcddev.height - 1), WHITE);
    lcd_show_string((uint16_t)(x0 + 20), (uint16_t)(y0 + 12), BTN_WIDTH - 20, 20, 16, "CLEAR", WHITE);
}

static bool touch_in_clear_button(uint16_t x, uint16_t y)
{
    return (x >= (lcddev.width - BTN_WIDTH)) && (y >= (lcddev.height - BTN_HEIGHT));
}

static void transition_to(demo_state_t *state, demo_state_t next)
{
    *state = next;
    s_state_start_us = esp_timer_get_time();
}

void LCD_DEMO(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    led_init();
    lcd_init();
    ESP_ERROR_CHECK(cst816_init());

    ESP_LOGI(TAG, "CST816 chip id: 0x%02X", cst816_get_chip_id());

    demo_state_t state = STATE_TEXT;
    bool state_drawn = false;
    bool last_valid = false;
    uint16_t last_x = 0;
    uint16_t last_y = 0;

    s_state_start_us = esp_timer_get_time();

    while (1) {
        int64_t elapsed_ms = (esp_timer_get_time() - s_state_start_us) / 1000;

        switch (state) {
        case STATE_LOGO:
            if (!state_drawn) {
                lcd_clear(BLACK);
                lcd_show_string(20, 45, 280, 32, 24, "ESP32-P4", WHITE);
                lcd_show_string(20, 80, 320, 32, 24, "MIPI LCD DEMO", GREEN);
                lcd_show_string(20, 120, 320, 20, 16, "Touch demo will start soon", CYAN);
                state_drawn = true;
            }
            if (elapsed_ms >= 1200) {
                transition_to(&state, STATE_TEXT);
                state_drawn = false;
            }
            break;

        case STATE_TEXT:
            if (!state_drawn) {
                lcd_clear(BLACK);
                lcd_show_string(12, 32, 340, 24, 16, "Color / Bar / Handwriting test", WHITE);
                lcd_show_string(12, 58, 340, 24, 16, "Touch board from CST816 reference", YELLOW);
                lcd_show_string(12, 84, 340, 24, 16, "Button at bottom-right clears canvas", GREEN);
                state_drawn = true;
            }
            if (elapsed_ms >= 1600) {
                transition_to(&state, STATE_COLOR_FULL);
                state_drawn = false;
            }
            break;

        case STATE_COLOR_FULL:
            if (!state_drawn || elapsed_ms >= 380) {
                lcd_clear(s_full_colors[s_color_index]);
                s_color_index = (uint8_t)((s_color_index + 1) % (sizeof(s_full_colors) / sizeof(s_full_colors[0])));
                s_state_start_us = esp_timer_get_time();
                state_drawn = true;
            }
            if (s_color_index == 0) {
                transition_to(&state, STATE_COLOR_BAR);
                state_drawn = false;
            }
            break;

        case STATE_COLOR_BAR:
            if (!state_drawn) {
                draw_color_bars();
                state_drawn = true;
            }
            if (elapsed_ms >= 1400) {
                transition_to(&state, STATE_GRAYSCALE);
                state_drawn = false;
            }
            break;

        case STATE_GRAYSCALE:
            if (!state_drawn) {
                draw_grayscale();
                state_drawn = true;
            }
            if (elapsed_ms >= 1400) {
                transition_to(&state, STATE_HANDWRITING);
                state_drawn = false;
            }
            break;

        case STATE_HANDWRITING: {
            cst816_state_t touch = {0};

            if (!state_drawn) {
                lcd_clear(BLACK);
                draw_clear_button();
                lcd_show_string(12, 8, 260, 20, 16, "Draw with finger", WHITE);
                state_drawn = true;
            }

            if (cst816_read(&touch)) {
                uint16_t x = touch.x;
                uint16_t y = touch.y;

                if (x >= lcddev.width) {
                    x = (uint16_t)(lcddev.width - 1);
                }
                if (y >= lcddev.height) {
                    y = (uint16_t)(lcddev.height - 1);
                }

                if (touch_in_clear_button(x, y)) {
                    lcd_clear(BLACK);
                    draw_clear_button();
                    lcd_show_string(12, 8, 260, 20, 16, "Draw with finger", WHITE);
                    last_valid = false;
                    vTaskDelay(pdMS_TO_TICKS(120));
                } else {
                    if (last_valid) {
                        draw_thick_line(last_x, last_y, x, y, WHITE, LINE_THICKNESS);
                    }
                    last_x = x;
                    last_y = y;
                    last_valid = true;
                }
            } else {
                last_valid = false;
            }
            break;
        }

        default:
            transition_to(&state, STATE_LOGO);
            state_drawn = false;
            break;
        }

        LED0_TOGGLE();
        vTaskDelay(pdMS_TO_TICKS(8));
    }
}
