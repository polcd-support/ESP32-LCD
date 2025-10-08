#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "driver/spi_master.h"
#include "driver/gpio.h"

#define USE_HORIZONTAL 0  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏

#define LCD_SPI_PORT SPI2_HOST
#define LCD_MISO_PIN -1
#define LCD_SCK_PIN 2
#define LCD_MOSI_PIN 3
#define LCD_RES_PIN 19
#define LCD_DC_PIN 18
#define LCD_CS_PIN 7
#define LCD_BLK_PIN 1

#define LCD_W 320
#define LCD_H 480

extern spi_device_handle_t spi;
//-----------------LCD端口定义----------------

#define LCD_RES_Clr()  gpio_set_level(LCD_RES_PIN,0)  // RES
#define LCD_RES_Set()  gpio_set_level(LCD_RES_PIN,1)

#define LCD_DC_Clr()   gpio_set_level(LCD_DC_PIN,0)  // DC
#define LCD_DC_Set()   gpio_set_level(LCD_DC_PIN,1)

#define LCD_BLK_Clr()  gpio_set_level(LCD_BLK_PIN,0)  // BLK
#define LCD_BLK_Set()  gpio_set_level(LCD_BLK_PIN,1)

// void LCD_GPIO_Init(void);//初始化GPIO
// void LCD_Writ_Bus(uint8_t dat);//模拟SPI时序
// void LCD_WR_DATA8(uint8_t dat);//写入一个字节
void LCD_WR_DATA(uint16_t dat);//写入两个字节
// void LCD_WR_REG(uint8_t dat);//写入一个指令
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);//设置坐标函数
void LCD_Init(void);//LCD初始化
#endif




