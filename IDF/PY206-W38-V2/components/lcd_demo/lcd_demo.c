/*
 * lcd_demo.c
 *
 *  Created on: 2026年1月24日
 *      Author: betwowt
 */


#include "lcd_demo.h"
#include "font/lv_font.h"
#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include <stdio.h>
#include <stdlib.h>

// 状态枚举定义
typedef enum {
    STATE_LOGO,
    STATE_TEXT,
    STATE_COLOR_FULL,
    STATE_COLOR_BAR,
    STATE_GRAYSCALE,
    STATE_HANDWRITING
} AppState;

// 全局变量
AppState g_state = STATE_LOGO;
uint8_t g_color_full_index = 0;

// 定义各个状态的持续时间
#define LOGO_DURATION_MS     2000
#define TEXT_DURATION_MS     2000
#define COLOR_INTERVAL_MS    2000
#define EFFECT_DURATION_MS   2000
#define BAR_DURATION_MS      1500
LV_IMG_DECLARE(img_logo);
LV_FONT_DECLARE(alibaba_32); 

// 定义屏幕尺寸 (根据你的实际屏幕修改)
#define SCREEN_WIDTH  410
#define SCREEN_HEIGHT 502

// 画布缓冲区 (使用 PSRAM 动态分配)
static lv_color_t *cbuf = NULL;
static lv_obj_t *canvas;

// 前向声明
static void canvas_draw_event_cb(lv_event_t *e);
static void clear_canvas_event_cb(lv_event_t *e);

// 辅助函数：清除屏幕上的所有对象并释放 PSRAM
static void clear_screen(void) {
    lv_obj_clean(lv_scr_act());

    // 释放之前分配的画布缓冲区
    if (cbuf != NULL) {
        heap_caps_free(cbuf);
        cbuf = NULL;
        printf("Canvas buffer freed from PSRAM\n");
    }
    canvas = NULL;
}

// 状态机处理函数 (定时器回调)
static void state_machine_handler(lv_timer_t *timer) {
    static uint32_t last_tick = 0;
    uint32_t now = lv_tick_get();
    
    // 状态逻辑处理
    switch (g_state) {
        case STATE_LOGO:
            // 创建 Logo 图片对象
			lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
            lv_obj_t *img = lv_img_create(lv_scr_act());
            lv_img_set_src(img, &img_logo);
            lv_obj_center(img); // 居中显示

            // 停留一段时间后切换
            lv_timer_reset(timer);
            timer->period = LOGO_DURATION_MS; // 设置下一次触发的延时
            g_state = STATE_TEXT;
            // 注意：这里我们不return，让 timer 继续跑，或者直接设置 timer 为一次性
            // 为了简单演示，我们让定时器一直跑，但利用 timer->period 控制节奏
            break;

        case STATE_TEXT:
            clear_screen();
			lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
            // 显示文本 - "STM32 Display"
            lv_obj_t *label1 = lv_label_create(lv_scr_act());
            lv_label_set_text(label1, "STM32 Display");
            lv_obj_set_style_text_color(label1, lv_color_white(), 0);
            lv_obj_set_style_text_font(label1, &lv_font_montserrat_24, 0);
            lv_obj_align(label1, LV_ALIGN_OUT_TOP_MID, 0, 20);

            // 显示文本 - "Multi-Size Text"
            lv_obj_t *label2 = lv_label_create(lv_scr_act());
            lv_label_set_text(label2, "Multi-Size Text");
            lv_obj_set_style_text_color(label2, lv_palette_main(LV_PALETTE_BLUE), 0);
            lv_obj_set_style_text_font(label2, &lv_font_montserrat_16, 0);
            lv_obj_align(label2, LV_ALIGN_OUT_TOP_MID, 0, 60);
			
            // 显示中文 - "浦洋液晶"
            lv_obj_t *label3 = lv_label_create(lv_scr_act());
            lv_label_set_text(label3, "浦洋液晶");
            lv_obj_set_style_text_color(label3, lv_palette_main(LV_PALETTE_RED), 0);
            // 注意：中文字体需要你自己生成 .c 文件并加载，这里假设有个字体叫 my_font_chinese_32
             lv_obj_set_style_text_font(label3, &alibaba_32, 0); 
            lv_obj_align(label3, LV_ALIGN_CENTER, 0, 100);

            timer->period = TEXT_DURATION_MS;
            g_state = STATE_COLOR_FULL;
            break;

        case STATE_COLOR_FULL:
            clear_screen();
			lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
            lv_obj_t *scr = lv_scr_act();
            
            switch (g_color_full_index) {
                case 0: lv_obj_set_style_bg_color(scr, lv_palette_main(LV_PALETTE_RED), 0); break;
                case 1: lv_obj_set_style_bg_color(scr, lv_palette_main(LV_PALETTE_GREEN), 0); break;
                case 2: lv_obj_set_style_bg_color(scr, lv_palette_main(LV_PALETTE_BLUE), 0); break;
                case 3: lv_obj_set_style_bg_color(scr, lv_color_white(), 0); break;
                case 4: lv_obj_set_style_bg_color(scr, lv_color_black(), 0); break;
            }
            
            // 设置全屏颜色刷新标志（如果需要，LVGL通常自动处理）
            lv_obj_invalidate(scr);

            if (++g_color_full_index > 4) {
                g_color_full_index = 0;
                g_state = STATE_COLOR_BAR;
                timer->period = EFFECT_DURATION_MS;
            } else {
                timer->period = COLOR_INTERVAL_MS;
            }
            break;

        case STATE_COLOR_BAR:
            clear_screen();
			lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
            // 绘制彩条 (简单实现：创建几个不同颜色的矩形)
            uint16_t bar_w = SCREEN_WIDTH / 8;
            for(int i=0; i<8; i++) {
                lv_obj_t *bar = lv_obj_create(lv_scr_act());
                lv_obj_set_size(bar, bar_w, SCREEN_HEIGHT);
                lv_obj_set_pos(bar, i * bar_w, 0);
                lv_obj_set_style_border_width(bar, 0, 0);
                
                // 简单的彩条颜色循环
                lv_color_t c = lv_color_hsv_to_rgb(i * 30, 100, 100);
                lv_obj_set_style_bg_color(bar, c, 0);
            }

            timer->period = BAR_DURATION_MS;
            g_state = STATE_GRAYSCALE;
            break;

        case STATE_GRAYSCALE:
            clear_screen();
            // 绘制灰度条 (类似彩条，颜色为灰度)
            uint16_t gray_w = SCREEN_WIDTH / 8;
            for(int i=0; i<8; i++) {
                lv_obj_t *bar = lv_obj_create(lv_scr_act());
                lv_obj_set_size(bar, gray_w, SCREEN_HEIGHT);
                lv_obj_set_pos(bar, i * gray_w, 0);
                lv_obj_set_style_border_width(bar, 0, 0);
                
                lv_color_t c = lv_color_make(i * 30, i * 30, i * 30); // 近似灰度
                lv_obj_set_style_bg_color(bar, c, 0);
            }
            
            timer->period = EFFECT_DURATION_MS;
            g_state = STATE_HANDWRITING;
            break;

		case STATE_HANDWRITING:
		    clear_screen();
		    // 设置背景为黑色
		    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);

		    // --- 从 PSRAM 分配画布缓冲区 ---
		    size_t cbuf_size = LV_CANVAS_BUF_SIZE_TRUE_COLOR(SCREEN_WIDTH, SCREEN_HEIGHT);
		    cbuf = (lv_color_t *)heap_caps_malloc(cbuf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
		    if (cbuf == NULL) {
		        printf("Failed to allocate PSRAM for canvas buffer!\n");
		        break;
		    }
		    printf("Canvas buffer allocated in PSRAM: %d bytes\n", cbuf_size);

		    // --- Canvas 创建 ---
		    canvas = lv_canvas_create(lv_scr_act());
		    lv_canvas_set_buffer(canvas, cbuf, SCREEN_WIDTH, SCREEN_HEIGHT, LV_IMG_CF_TRUE_COLOR);
		    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);
		    lv_obj_set_size(canvas, SCREEN_WIDTH, SCREEN_HEIGHT);
		    lv_obj_set_pos(canvas, 0, 0);
		    // 确保 Canvas 可以接收触摸事件
		    lv_obj_add_flag(canvas, LV_OBJ_FLAG_CLICKABLE);
		    // --------------------

		    // 创建清除按钮
		    lv_obj_t *clear_btn = lv_btn_create(lv_scr_act());
		    lv_obj_set_size(clear_btn, 80, 40);
		    lv_obj_align(clear_btn, LV_ALIGN_BOTTOM_RIGHT, -5, -5);

		    lv_obj_t *btn_label = lv_label_create(clear_btn);
		    lv_label_set_text(btn_label, "Clear");
		    lv_obj_center(btn_label);

		    // 按钮事件回调
		    lv_obj_add_event_cb(clear_btn, clear_canvas_event_cb, LV_EVENT_CLICKED, NULL);

		    // 添加 Canvas 绘图事件回调
		    lv_obj_add_event_cb(canvas, canvas_draw_event_cb, LV_EVENT_ALL, NULL);

		    // 进入手写状态，暂停状态机自动流转
		    lv_timer_pause(timer);
		    break;
            
        default:
            break;
    }
}

// Canvas 绘图事件回调函数 (黑色背景，白色画笔)
static void canvas_draw_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);

    static lv_point_t last_point;
    static bool is_pressed = false;

    if (code == LV_EVENT_PRESSED) {
        lv_indev_get_point(lv_indev_get_act(), &last_point);
        is_pressed = true;
        printf("Pressed at (%d, %d)\n", last_point.x, last_point.y);

        // 在按下位置画一个点
        lv_draw_rect_dsc_t rect_dsc;
        lv_draw_rect_dsc_init(&rect_dsc);
        rect_dsc.bg_color = lv_color_white();
        rect_dsc.border_width = 0;
        lv_canvas_draw_rect(obj, last_point.x - 2, last_point.y - 2, 5, 5, &rect_dsc);
    } else if (code == LV_EVENT_PRESSING) {
        if (is_pressed) {
            lv_point_t cur_point;
            lv_indev_get_point(lv_indev_get_act(), &cur_point);

            // 检查是否有移动
            if (cur_point.x != last_point.x || cur_point.y != last_point.y) {
                // 边界检查
                if (cur_point.x >= 0 && cur_point.x < SCREEN_WIDTH &&
                    cur_point.y >= 0 && cur_point.y < SCREEN_HEIGHT) {

                    // 构建线条坐标点
                    lv_point_t line_points[2];
                    line_points[0] = last_point;
                    line_points[1] = cur_point;

                    // 白色画笔，黑色背景
                    lv_draw_line_dsc_t line_dsc;
                    lv_draw_line_dsc_init(&line_dsc);
                    line_dsc.color = lv_color_white();
                    line_dsc.width = 4;
                    line_dsc.round_end = 1;
                    line_dsc.round_start = 1;

                    lv_canvas_draw_line(obj, line_points, 2, &line_dsc);

                    last_point = cur_point;
                }
            }
        }
    } else if (code == LV_EVENT_RELEASED) {
        is_pressed = false;
        printf("Released\n");
    }
}

// 清除画板事件回调函数（不释放内存，只重置内容）
static void clear_canvas_event_cb(lv_event_t *e) {
    (void)e;  // 未使用的参数
    if (canvas && cbuf) {
        lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);
    }
}

// 主任务入口
void lvgl_demo_task(void *pvParameters) {
    // 状态机定时器，初始周期 100ms，进入 loop 后会根据状态调整
    lv_timer_create(state_machine_handler, 100, NULL);

    while (1) {
        // LVGL 任务处理
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// 初始化调用 (在 main 函数或者 app_main 中调用)
void start_lvgl_demo() {
    xTaskCreate(lvgl_demo_task, "lvgl_demo", 4096, NULL, 1, NULL);
}