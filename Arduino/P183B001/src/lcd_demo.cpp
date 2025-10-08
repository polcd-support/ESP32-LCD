#include "../inc/lcd_demo.h"
#include "../inc/lcd.h"
#include "../inc/cst816.h"
#include "stdlib.h"
#include "../inc/pic.h"
#include "Arduino.h"
#include <stdio.h>

// 模拟get_tick()，返回系统启动后的毫秒数
static uint32_t get_tick(void)
{
    return millis();
}

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
uint32_t g_state_timer = 0;
uint8_t g_img_index = 0;
uint8_t color_full_index = 0;
uint8_t g_countdown = 3;
extern const uint8_t gImage_logo[];

uint8_t IsTouchInButton(uint16_t x, uint16_t y);

void LCD_DEMO(void)
{
    LCD_Init();
    CST816_Init();
    LCD_Fill(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, BLACK);
    delay(100);
    LCD_BLK_Set(); // 打开背光

    static uint16_t lastX, lastY;
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        CST816_Get_XY_AXIS(); // 更新触摸坐标
        switch (g_state)
        {
        case STATE_LOGO:
            LCD_ShowPicture(0, 29, 239, 219, gImage_logo);

            if (get_tick() - g_state_timer > LOGO_DURATION)
            {
                LCD_Fill(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, BLACK);
                g_state = STATE_TEXT;
                g_state_timer = get_tick();
            }
            break;

        case STATE_TEXT:
            LCD_ShowString(20, 50, (const uint8_t *)"STM32 Display", WHITE, BLACK, 24, 0);
            LCD_ShowString(30, 100, (const uint8_t *)"Multi-Size Text", BLUE, BLACK, 16, 0);
            LCD_ShowChinese(80, 150, (uint8_t *)"浦洋液晶", RED, BLACK, 32, 0);

            if (get_tick() - g_state_timer > TEXT_DURATION)
            {
                LCD_Fill(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, BLACK);
                g_state = STATE_COLOR_FULL;
                g_state_timer = get_tick();
            }
            break;

        case STATE_IMAGE:
            switch (g_img_index)
            {
                //					case 0: LCD_ShowPicture(0, 0, 239, 279, gImage_img1); break;
                //					case 1: LCD_ShowPicture(0, 0, 239, 279, gImage_img2); break;
                //					case 2: LCD_ShowPicture(0, 0, 239, 171, gImage_img3); break;
            }

            if (get_tick() - g_state_timer > IMAGE_INTERVAL)
            {
                if (++g_img_index > 2)
                {
                    g_state = STATE_COLOR_BAR;
                    g_img_index = 0;
                }
                LCD_Fill(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, BLACK);
                g_state_timer = get_tick();
            }
            break;
        case STATE_COLOR_FULL:
            switch (color_full_index)
            {
            case 0:
                LCD_Fill(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, RED);
                break;
            case 1:
                LCD_Fill(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, GREEN);
                break;
            case 2:
                LCD_Fill(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, BLUE);
                break;
            case 3:
                LCD_Fill(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
                break;
            case 4:
                LCD_Fill(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, BLACK);
                break;
            }

            if (get_tick() - g_state_timer > COLOR_FULL_INTERVAL)
            {
                if (++color_full_index > 4)
                {
                    g_state = STATE_COLOR_BAR;
                    color_full_index = 0;
                }
                g_state_timer = get_tick();
            }
            break;
        case STATE_COLOR_BAR:
            DrawColorBars();
            if (get_tick() - g_state_timer > EFFECT_DURATION)
            {
                LCD_Fill(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, BLACK);
                g_state = STATE_GRAYSCALE;
                g_state_timer = get_tick();
            }
            break;

        case STATE_GRAYSCALE:
            DrawGrayscale();
            if (get_tick() - g_state_timer > EFFECT_DURATION)
            {
                LCD_Fill(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, BLACK);
                g_state = STATE_HANDWRITING;
                g_state_timer = get_tick();
                DrawClearButton();
            }
            break;

        case STATE_COUNTDOWN:
            LCD_ShowIntNum(100, 120, g_countdown, 1, RED, BLACK, 32);
            if (get_tick() - g_state_timer > 1000)
            {
                if (--g_countdown == 0)
                {
                    LCD_Fill(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, BLACK);
                    DrawClearButton();
                    g_state = STATE_HANDWRITING;
                }
                g_state_timer = get_tick();
            }
            break;

        case STATE_HANDWRITING:
            // 触摸绘图
            if (CST816_Get_FingerNum() > 0)
            {
                if (lastX != 0xFFFF && lastY != 0xFFFF)
                {
                    // 使用Bresenham算法画线
                    LCD_DrawThickLine(lastX, lastY, CST816_Instance.X_Pos, CST816_Instance.Y_Pos, WHITE, 2);
                }
                lastX = CST816_Instance.X_Pos;
                lastY = CST816_Instance.Y_Pos;
                if (IsTouchInButton(lastX, lastY))
                {
                    LCD_Fill(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, BLACK);
                    DrawClearButton();
                }
            }
            else
            {
                lastX = lastY = 0xFFFF; // 手指抬起时重置
            }
            break;
        }
    }
}

uint8_t IsTouchInButton(uint16_t x, uint16_t y)
{
    return (x >= SCREEN_WIDTH - BTN_WIDTH) &&
           (y >= SCREEN_HEIGHT - BTN_HEIGHT);
}