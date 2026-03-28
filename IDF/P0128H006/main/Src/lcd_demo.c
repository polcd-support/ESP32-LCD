#include "Inc/lcd_demo.h"
#include "Inc/lcd.h"
#include "Inc/ft6236.h"
#include "Inc/pic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

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

static AppState g_state = STATE_LOGO;
static AppState g_last_state = STATE_COUNTDOWN;
static uint32_t g_state_timer = 0;
static uint8_t g_img_index = 0;
static uint8_t color_full_index = 0;
static uint8_t g_countdown = 3;

extern const uint8_t gImage_logo[];

static inline uint32_t app_tick_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static uint8_t IsTouchInButton(uint16_t x, uint16_t y)
{
    return (x >= SCREEN_WIDTH - BTN_WIDTH) && (y >= SCREEN_HEIGHT - BTN_HEIGHT);
}

void LCD_DEMO(void)
{
    LCD_Init();
    FT6236_Init();
    LCD_Fill(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, BLACK);
    vTaskDelay(pdMS_TO_TICKS(100));
    LCD_BLK_Set();

    g_state = STATE_LOGO;
    g_last_state = STATE_COUNTDOWN;
    g_state_timer = app_tick_ms();

    uint16_t lastX = 0xFFFF;
    uint16_t lastY = 0xFFFF;

    while (1)
    {
        uint32_t now = app_tick_ms();
        FT6236_Get_Touch_Data();

        if (g_state != g_last_state)
        {
            g_last_state = g_state;
            g_state_timer = now;

            switch (g_state)
            {
            case STATE_LOGO:
                LCD_ShowPicture(29, 29, 199, 182, gImage_logo);
                break;
            case STATE_TEXT:
                LCD_Fill(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, BLACK);
                LCD_ShowString(20, 50, (const uint8_t *)"STM32 Display", WHITE, BLACK, 24, 0);
                LCD_ShowString(30, 100, (const uint8_t *)"Multi-Size Text", BLUE, BLACK, 16, 0);
                LCD_ShowChinese(80, 150, (uint8_t *)"ÆÖÑóÒº¾§", RED, BLACK, 32, 0);
                break;
            case STATE_COLOR_FULL:
                break;
            case STATE_COLOR_BAR:
                DrawColorBars();
                break;
            case STATE_GRAYSCALE:
                DrawGrayscale();
                break;
            case STATE_HANDWRITING:
                LCD_Fill(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, BLACK);
                DrawClearButton();
                break;
            case STATE_IMAGE:
            case STATE_COUNTDOWN:
            default:
                break;
            }
        }

        switch (g_state)
        {
        case STATE_LOGO:
            if (now - g_state_timer > LOGO_DURATION)
            {
                g_state = STATE_TEXT;
            }
            break;

        case STATE_TEXT:
            if (now - g_state_timer > TEXT_DURATION)
            {
                g_state = STATE_COLOR_FULL;
            }
            break;

        case STATE_IMAGE:
            if (now - g_state_timer > IMAGE_INTERVAL)
            {
                if (++g_img_index > 2)
                {
                    g_state = STATE_COLOR_BAR;
                    g_img_index = 0;
                }
            }
            break;

        case STATE_COLOR_FULL:
            switch (color_full_index)
            {
            case 0:
                LCD_Fill(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, RED);
                break;
            case 1:
                LCD_Fill(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, GREEN);
                break;
            case 2:
                LCD_Fill(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, BLUE);
                break;
            case 3:
                LCD_Fill(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, WHITE);
                break;
            case 4:
                LCD_Fill(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, BLACK);
                break;
            default:
                break;
            }

            if (now - g_state_timer > COLOR_FULL_INTERVAL)
            {
                if (++color_full_index > 4)
                {
                    color_full_index = 0;
                    g_state = STATE_COLOR_BAR;
                }
                else
                {
                    g_state_timer = now;
                }
            }
            break;

        case STATE_COLOR_BAR:
            if (now - g_state_timer > EFFECT_DURATION)
            {
                LCD_Fill(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, BLACK);
                g_state = STATE_GRAYSCALE;
            }
            break;

        case STATE_GRAYSCALE:
            if (now - g_state_timer > EFFECT_DURATION)
            {
                g_state = STATE_HANDWRITING;
            }
            break;

        case STATE_COUNTDOWN:
            LCD_ShowIntNum(100, 120, g_countdown, 1, RED, BLACK, 32);
            if (now - g_state_timer > 1000)
            {
                if (--g_countdown == 0)
                {
                    g_state = STATE_HANDWRITING;
                }
                else
                {
                    g_state_timer = now;
                }
            }
            break;

        case STATE_HANDWRITING:
            if (FT6236_Get_Touch_Count() > 0)
            {
                if (lastX != 0xFFFF && lastY != 0xFFFF)
                {
                    LCD_DrawThickLine(lastX, lastY, FT6236_Instance.X_Pos, FT6236_Instance.Y_Pos, WHITE, 2);
                }
                lastX = FT6236_Instance.X_Pos;
                lastY = FT6236_Instance.Y_Pos;

                if (IsTouchInButton(lastX, lastY))
                {
                    LCD_Fill(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, BLACK);
                    DrawClearButton();
                }
            }
            else
            {
                lastX = 0xFFFF;
                lastY = 0xFFFF;
            }
            break;

        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
