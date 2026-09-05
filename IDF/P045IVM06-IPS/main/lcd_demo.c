#include "lcd_demo.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "ft6336.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd.h"
#include "led.h"
#include "logo_380.h"
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

#define BTN_WIDTH            112
#define BTN_HEIGHT           56
#define LINE_THICKNESS       3
#define TOUCH_JUMP_LIMIT     50
#define TOUCH_CLEAR_HOLD_MS  250

static int64_t s_state_start_us;
static uint8_t s_color_index;

static uint16_t abs_diff_u16(uint16_t a, uint16_t b)
{
    return (a > b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static void draw_logo_page(void)
{
    uint16_t x = (uint16_t)((lcddev.width - LOGO_380_WIDTH) / 2);
    uint16_t y = (uint16_t)((lcddev.height - LOGO_380_HEIGHT) / 2);

    lcd_clear(WHITE);
    lcd_draw_rgb565_bitmap(x, y, LOGO_380_WIDTH, LOGO_380_HEIGHT, logo_380_rgb565);
}

static void draw_touch_info(const ft6336_state_t *touch, uint16_t lcd_x, uint16_t lcd_y, uint16_t pen_x, uint16_t pen_y)
{
    char line[32];

    lcd_fill(0, 0, 320, 78, BLACK);
    lcd_set_back_color(BLACK);
    snprintf(line, sizeof(line), "RAW X:%4u Y:%4u", touch->raw_x, touch->raw_y);
    lcd_show_string(8, 6, 304, 20, 16, line, YELLOW);
    snprintf(line, sizeof(line), "LCD X:%4u Y:%4u", lcd_x, lcd_y);
    lcd_show_string(8, 30, 304, 20, 16, line, CYAN);
    snprintf(line, sizeof(line), "PEN X:%4u Y:%4u", pen_x, pen_y);
    lcd_show_string(8, 54, 304, 20, 16, line, GREEN);
}

static void draw_brush_hline(int x, int y, int len, uint16_t color)
{
    if (len <= 0 || y < 0 || y >= (int)lcddev.height) {
        return;
    }

    int x0 = x;
    int x1 = x + len;

    if (x0 < 0) {
        x0 = 0;
    }
    if (x1 > (int)lcddev.width) {
        x1 = (int)lcddev.width;
    }

    if (x0 < x1) {
        lcd_fill((uint16_t)x0, (uint16_t)y, (uint16_t)x1, (uint16_t)(y + 1), color);
    }
}

static void draw_thick_point(int x, int y, uint16_t color, uint8_t radius)
{
    if (radius == 0) {
        draw_brush_hline(x, y, 1, color);
        return;
    }

    if ((x + radius) < 0 || (y + radius) < 0 ||
        (x - radius) >= (int)lcddev.width || (y - radius) >= (int)lcddev.height) {
        return;
    }

    uint32_t imax = ((uint32_t)radius * 724U) >> 10;
    uint32_t sqmax = (uint32_t)radius * (uint32_t)radius + (radius >> 1);
    uint32_t xr = radius;
    uint32_t i_squared = 1;

    draw_brush_hline(x - radius, y, (int)(2 * radius + 1), color);

    for (uint32_t i = 1; i <= imax; i++) {
        if ((i_squared + xr * xr) > sqmax) {
            if (xr > imax) {
                draw_brush_hline(x - (int)i + 1, y + (int)xr, (int)(2 * (i - 1) + 1), color);
                draw_brush_hline(x - (int)i + 1, y - (int)xr, (int)(2 * (i - 1) + 1), color);
            }
            xr--;
        }

        draw_brush_hline(x - (int)xr, y + (int)i, (int)(2 * xr + 1), color);
        draw_brush_hline(x - (int)xr, y - (int)i, (int)(2 * xr + 1), color);
        i_squared += (i << 1) + 1;
    }
}

static void draw_thick_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint8_t radius)
{
    uint16_t dx_abs = abs_diff_u16(x1, x2);
    uint16_t dy_abs = abs_diff_u16(y1, y2);

    if (dx_abs > TOUCH_JUMP_LIMIT || dy_abs > TOUCH_JUMP_LIMIT) {
        return;
    }

    int delta_x = (int)x2 - (int)x1;
    int delta_y = (int)y2 - (int)y1;
    int row = x1;
    int col = y1;
    int incx;
    int incy;

    if (delta_x > 0) {
        incx = 1;
    } else if (delta_x == 0) {
        incx = 0;
    } else {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0) {
        incy = 1;
    } else if (delta_y == 0) {
        incy = 0;
    } else {
        incy = -1;
        delta_y = -delta_y;
    }

    int distance = (delta_x > delta_y) ? delta_x : delta_y;
    int xerr = 0;
    int yerr = 0;

    for (int t = 0; t <= distance; t++) {
        draw_thick_point(row, col, color, radius);
        xerr += delta_x;
        yerr += delta_y;

        if (xerr > distance) {
            xerr -= distance;
            row += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            col += incy;
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

    lcd_fill(x0, y0, (uint16_t)lcddev.width, (uint16_t)lcddev.height, GRAY);
    lcd_draw_rectangle(x0, y0, (uint16_t)(lcddev.width - 1), (uint16_t)(lcddev.height - 1), WHITE);
    lcd_set_back_color(GRAY);
    lcd_show_string((uint16_t)(x0 + 34), (uint16_t)(y0 + 20), BTN_WIDTH - 34, 20, 16, "Clear", BLACK);
    lcd_set_back_color(BLACK);
}

static bool touch_in_clear_button(uint16_t x, uint16_t y)
{
    uint16_t x0 = (uint16_t)(lcddev.width - BTN_WIDTH);
    uint16_t y0 = (uint16_t)(lcddev.height - BTN_HEIGHT);

    return x >= x0 && x < lcddev.width && y >= y0 && y < lcddev.height;
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
    ESP_ERROR_CHECK(ft6336_init());

    ESP_LOGI(TAG, "FT6336U chip id: 0x%02X, vendor id: 0x%02X", ft6336_get_chip_id(), ft6336_get_vendor_id());

    demo_state_t state = STATE_LOGO;
    bool state_drawn = false;
    bool last_valid = false;
    uint16_t last_x = 0;
    uint16_t last_y = 0;
    int64_t last_info_us = 0;
    int64_t clear_pressed_since_us = 0;
    bool clear_latched = false;

    s_state_start_us = esp_timer_get_time();

    while (1) {
        int64_t elapsed_ms = (esp_timer_get_time() - s_state_start_us) / 1000;

        switch (state) {
        case STATE_LOGO:
            if (!state_drawn) {
                draw_logo_page();
                state_drawn = true;
            }
            if (elapsed_ms >= 3000) {
                transition_to(&state, STATE_TEXT);
                state_drawn = false;
            }
            break;

        case STATE_TEXT:
            if (!state_drawn) {
                lcd_clear(BLACK);
                lcd_set_back_color(BLACK);
                lcd_show_string(28, 60, 420, 40, 32, "ESP32-P4 MIPI", WHITE);
                lcd_show_string(28, 118, 420, 40, 32, "Display Demo", GREEN);
                lcd_show_string(28, 190, 420, 32, 24, "480 x 854 RGB888", CYAN);
                lcd_show_string(28, 230, 420, 32, 24, "Touch IC: FT6336", YELLOW);
                state_drawn = true;
            }
            if (elapsed_ms >= 2200) {
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
            ft6336_state_t touch = {0};

            if (!state_drawn) {
                lcd_clear(BLACK);
                draw_clear_button();
                lcd_set_back_color(BLACK);
                lcd_show_string(8, 6, 300, 24, 24, "Touch coords", WHITE);
                last_info_us = 0;
                clear_pressed_since_us = 0;
                clear_latched = false;
                last_valid = false;
                state_drawn = true;
            }

            if (ft6336_read(&touch)) {
                int64_t now_us = esp_timer_get_time();
                uint16_t x = touch.x;
                uint16_t y = touch.y;

                /* Do not clamp invalid samples to the bottom-right Clear button. */
                if (x >= lcddev.width || y >= lcddev.height) {
                    last_valid = false;
                    clear_pressed_since_us = 0;
                    clear_latched = false;
                    break;
                }

                uint16_t pen_x = x;
                uint16_t pen_y = y;

                if (((now_us - last_info_us) / 1000) >= 100) {
                    draw_touch_info(&touch, x, y, pen_x, pen_y);
                    last_info_us = now_us;
                }

                if (touch_in_clear_button(x, y)) {
                    last_valid = false;
                    if (clear_pressed_since_us == 0) {
                        clear_pressed_since_us = now_us;
                    }
                    if (!clear_latched && ((now_us - clear_pressed_since_us) / 1000) >= TOUCH_CLEAR_HOLD_MS) {
                        lcd_clear(BLACK);
                        draw_clear_button();
                        lcd_set_back_color(BLACK);
                        lcd_show_string(8, 6, 300, 24, 24, "Touch coords", WHITE);
                        last_info_us = 0;
                        clear_latched = true;
                    }
                } else {
                    clear_pressed_since_us = 0;
                    clear_latched = false;
                    if (!last_valid) {
                        draw_thick_point(pen_x, pen_y, WHITE, LINE_THICKNESS);
                    } else {
                        draw_thick_line(last_x, last_y, pen_x, pen_y, WHITE, LINE_THICKNESS);
                    }
                    last_x = pen_x;
                    last_y = pen_y;
                    last_valid = true;
                }
            } else {
                last_valid = false;
                last_info_us = 0;
                clear_pressed_since_us = 0;
                clear_latched = false;
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
