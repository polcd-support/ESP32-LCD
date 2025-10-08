#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "Arduino.h"
#include <SPI.h>

#define USE_HORIZONTAL 0  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏

#define LCD_W 320
#define LCD_H 480

#define LCD_MISO_PIN -1
#define LCD_SCK_PIN  2
#define LCD_MOSI_PIN 3
#define LCD_RES_PIN 19
#define LCD_DC_PIN 18
#define LCD_CS_PIN 7
#define LCD_BLK_PIN 1

extern SPISettings lcdSPISettings;

//-----------------LCD端口定义---------------- 
// 复位引脚操作
#define LCD_RES_Clr()  digitalWrite(LCD_RES_PIN, LOW)  // 拉低RES引脚
#define LCD_RES_Set()  digitalWrite(LCD_RES_PIN, HIGH) // 拉高RES引脚

// 数据/命令引脚操作
#define LCD_DC_Clr()   digitalWrite(LCD_DC_PIN, LOW)   // 拉低DC引脚（通常表示命令模式）
#define LCD_DC_Set()   digitalWrite(LCD_DC_PIN, HIGH)  // 拉高DC引脚（通常表示数据模式）

// 背光引脚操作
#define LCD_BLK_Clr()  digitalWrite(LCD_BLK_PIN, LOW)  // 拉低BLK引脚（关闭背光）
#define LCD_BLK_Set()  digitalWrite(LCD_BLK_PIN, HIGH) // 拉高BLK引脚（打开背光）

// 片选引脚操作（选中/取消LCD）
#define LCD_CS_Clr()   digitalWrite(LCD_CS_PIN, LOW)   // 拉低CS引脚（选中LCD）
#define LCD_CS_Set()   digitalWrite(LCD_CS_PIN, HIGH)  // 拉高CS引脚（取消选中）

// void LCD_GPIO_Init(void);//初始化GPIO
// void LCD_Writ_Bus(uint8_t dat);//模拟SPI时序
void LCD_WR_DATA8(uint8_t dat);//写入一个字节
void LCD_WR_DATA(uint16_t dat);//写入两个字节
// void LCD_WR_REG(uint8_t dat);//写入一个指令
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);//设置坐标函数
void LCD_Init(void);//LCD初始化
#endif




