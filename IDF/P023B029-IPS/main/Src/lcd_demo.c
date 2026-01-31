#include "Inc/lcd_demo.h"
#include "../Inc/lcd.h"
#include "stdlib.h"
#include "Inc/pic.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"

typedef enum
{
    STATE_LOGO,
    STATE_TEXT,
    STATE_IMAGE,
    STATE_COLOR_FULL,
    STATE_COLOR_BAR,
    STATE_GRAYSCALE,
    STATE_COUNTDOWN,
    STATE_HANDWRITING
} AppState;

AppState g_state = STATE_LOGO;
uint8_t g_img_index = 0;
uint8_t color_full_index = 0;
uint8_t g_countdown = 3;
extern const uint8_t gImage_logo[];

void LCD_DEMO(void)
{

    lcd_init();
    lcd_clear(BLACK);
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1)
    {
        // 更新触摸坐标
        switch (g_state)
        {
        case STATE_LOGO:
            lcd_show_picture(0, 29, 240, 220, gImage_logo);
            vTaskDelay(pdMS_TO_TICKS(LOGO_DURATION));
            lcd_clear(BLACK);
            g_state = STATE_TEXT; //

            break;
        case STATE_TEXT:
            lcd_show_string(20, 50, (const uint8_t *)"ESP32 Display", WHITE, BLACK, 24, 0);
            lcd_show_string(30, 100, (const uint8_t *)"Multi-Size Text", BLUE, BLACK, 16, 0);
            lcd_show_chinese(80, 150, (uint8_t *)"浦洋液晶", RED, BLACK, 32, 0);
            vTaskDelay(pdMS_TO_TICKS(TEXT_DURATION));

            lcd_clear(BLACK);
            g_state = STATE_COLOR_FULL;

            break;
        case STATE_COLOR_FULL:
            switch (color_full_index)
            {
            case 0:
                lcd_clear(RED);
                break;
            case 1:
                lcd_clear(GREEN);
                break;
            case 2:
                lcd_clear(BLUE);
                break;
            case 3:
                lcd_clear(WHITE);
                break;
            case 4:
                lcd_clear(BLACK);
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(COLOR_FULL_INTERVAL));
            if (++color_full_index > 4)
            {
                g_state = STATE_COLOR_BAR;
                color_full_index = 0;
            }
            break;
        case STATE_COLOR_BAR:
            lcd_draw_color_bars();
            vTaskDelay(pdMS_TO_TICKS(EFFECT_DURATION));
            lcd_clear(BLACK);
            g_state = STATE_GRAYSCALE;

            break;
        case STATE_GRAYSCALE:
            lcd_draw_grayscale();
            vTaskDelay(pdMS_TO_TICKS(EFFECT_DURATION));
            lcd_clear(BLACK);

            g_state = STATE_LOGO;
            // lcd_draw_clear_button();

            break;

        default:
            break;
        }
    }
}
