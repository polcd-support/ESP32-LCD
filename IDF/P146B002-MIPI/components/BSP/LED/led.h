/**
 ****************************************************************************************************
 * @file        led.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-01
 * @brief       LED驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32-P4 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 * 
 ****************************************************************************************************
 */

#ifndef __LED_H
#define __LED_H

#include "driver/gpio.h"


/* 引脚定义 */
#define LED0_GPIO_PIN    GPIO_NUM_51    /* LED0连接的GPIO端口 */

/* LED0端口定义 */
#define LED0(x)          do { x ?                                \
                              gpio_set_level(LED0_GPIO_PIN, 1):  \
                              gpio_set_level(LED0_GPIO_PIN, 0);  \
                            } while(0)  /* LED0翻转 */

/* LED取反定义 */
#define LED0_TOGGLE()    do { gpio_set_level(LED0_GPIO_PIN, !gpio_get_level(LED0_GPIO_PIN)); } while(0)  /* LED0翻转 */

/* 函数声明*/
void led_init(void);     /* 初始化LED */

#endif
