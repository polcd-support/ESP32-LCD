#include "Inc/lcd_demo.h"
#include "../Inc/lcd.h"
#include "../Inc/gt911.h"
#include "stdlib.h"
#include "Inc/pic.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

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

static uint8_t IsTouchInButton(uint16_t x, uint16_t y)
{
    return (x >= EXAMPLE_LCD_H_RES - BTN_WIDTH) &&
           (y >= EXAMPLE_LCD_V_RES - BTN_HEIGHT);
}

void LCD_DEMO(void)
{
    GT911_init(&alfredGt911, TOUCH_GT911_SDA, TOUCH_GT911_SCL, TOUCH_GT911_INT,
               TOUCH_GT911_RES, I2C_NUM_0, GT911_ADDR1,
               SCREEN_WIDTH, SCREEN_HEIGHT);
    // 初始化之后必须设置
    GT911_setRotation(&alfredGt911, ROTATION_INVERTED);
    lcd_init();
    lcd_clear(BLACK);
    vTaskDelay(pdMS_TO_TICKS(100));

    static int lastX, lastY,x,y;
    while (1)
    {
        // 更新触摸坐标
        // FT6236_Get_Touch_Data(); // 更新触摸坐标
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

            g_state = STATE_HANDWRITING;
            lcd_draw_clear_button();

            break;

        case STATE_HANDWRITING:
            //触摸绘图
            if (GT911_touched(&alfredGt911))
            {
                GT911_read_pos(&alfredGt911, &x, &y, 0);
                ESP_LOGI("TOUCH", "x:%d.y:%d\n",x,y);
                if (lastX != 0xFFFF && lastY != 0xFFFF)
                {
                    // 使用Bresenham算法画线
                    lcd_draw_thick_line(lastX, lastY, x, y, WHITE, 2);
                }
                lastX = x;
                lastY = y;
                if (IsTouchInButton(lastX, lastY))
                {
                    lcd_clear(BLACK);
                    lcd_draw_clear_button();
                }
            }
            else
            {
                lastX = lastY = 0xFFFF; // 手指抬起时重置
            }

            // 短延时，避免CPU占用过高
            vTaskDelay(pdMS_TO_TICKS(10));
            break;
        default:
            break;
        }
    }
}
