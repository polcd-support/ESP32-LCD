#include "../inc/lcd_init.h"

SPISettings lcdSPISettings(15000000, MSBFIRST, SPI_MODE0);

static void LCD_GPIO_Init(void) 
{
  pinMode(LCD_RES_PIN, OUTPUT);
  pinMode(LCD_DC_PIN, OUTPUT);
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(LCD_BLK_PIN, OUTPUT);
  pinMode(12, OUTPUT);//两个板载LED，会亮，拉低关闭led
  pinMode(13, OUTPUT);

  // 初始化默认电平（可选，根据硬件要求设置）
  LCD_RES_Set();   // 复位引脚默认高电平（未复位）
  LCD_DC_Set();    // 默认数据模式
  LCD_CS_Set();    // 未选中LCD
  LCD_BLK_Set();   // 打开背光
	digitalWrite(12, LOW);
	digitalWrite(13, LOW);

	SPI.setHwCs(false);
	SPI.begin(LCD_SCK_PIN,LCD_MISO_PIN,LCD_MOSI_PIN,LCD_CS_PIN);
	SPI.beginTransaction(lcdSPISettings);
}


/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
static inline void LCD_Writ_Bus(uint8_t dat) 
{	
	LCD_CS_Clr();
	SPI.transfer(dat);
  LCD_CS_Set();	
}

/******************************************************************************
	  函数说明：LCD写入数据
	  入口数据：dat 写入的数据
	  返回值：  无
******************************************************************************/
void LCD_WR_DATA8(uint8_t dat) {
	LCD_Writ_Bus(dat);
}

/******************************************************************************
	  函数说明：LCD写入数据
	  入口数据：dat 写入的数据
	  返回值：  无
******************************************************************************/
void LCD_WR_DATA(uint16_t dat) {
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}

/******************************************************************************
	  函数说明：LCD写入命令
	  入口数据：dat 写入的命令
	  返回值：  无
******************************************************************************/
static inline void LCD_WR_REG(uint8_t dat) {
	LCD_DC_Clr();  // 写命令
	LCD_Writ_Bus(dat);
	LCD_DC_Set();
}

/******************************************************************************
	  函数说明：设置起始和结束地址
	  入口数据：x1,x2 设置列的起始和结束地址
				y1,y2 设置行的起始和结束地址
	  返回值：  无
******************************************************************************/
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
	if (USE_HORIZONTAL == 0)
	{
		LCD_WR_REG(0x2a); // 列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b); // 行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c); // 储存器写
	}
	else if (USE_HORIZONTAL == 1)
	{
		LCD_WR_REG(0x2a); // 列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b); // 行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c); // 储存器写
	}
	else if (USE_HORIZONTAL == 2)
	{
		LCD_WR_REG(0x2a); // 列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b); // 行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c); // 储存器写
	}
	else
	{
		LCD_WR_REG(0x2a); // 列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b); // 行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c); // 储存器写
	}
}

void LCD_Init(void) {
	LCD_GPIO_Init();  // 初始化GPIO

	LCD_RES_Clr();  // 复位
	delay(100);
	LCD_RES_Set();
	delay(100);

	//************* Start Initial Sequence **********//
	LCD_WR_REG(0x11); //Sleep out 
	delay(120);              //Delay 120ms 
	//************* Start Initial Sequence **********// 
	LCD_WR_REG(0x36);
	if (USE_HORIZONTAL == 0)
		LCD_WR_DATA8(0x00);
	else if (USE_HORIZONTAL == 1)
		LCD_WR_DATA8(0xC0);
	else if (USE_HORIZONTAL == 2)
		LCD_WR_DATA8(0x70);
	else
		LCD_WR_DATA8(0xA0);

	LCD_WR_REG(0x3A);
	LCD_WR_DATA8(0x05);

	LCD_WR_REG(0xB0);     
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0xE0);   

	LCD_WR_REG(0xB2);     
	LCD_WR_DATA8(0x0C);   
	LCD_WR_DATA8(0x0C);   
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0x33);   
	LCD_WR_DATA8(0x33);   

	LCD_WR_REG(0xB7);     
	LCD_WR_DATA8(0x02);   

	LCD_WR_REG(0xBB);     
	LCD_WR_DATA8(0x19);   

	LCD_WR_REG(0xC0);     
	LCD_WR_DATA8(0x2C);   

	LCD_WR_REG(0xC2);     
	LCD_WR_DATA8(0x01);   

	LCD_WR_REG(0xC3);     
	LCD_WR_DATA8(0x13);   

	LCD_WR_REG(0xC6);     
	LCD_WR_DATA8(0x0F);   

	LCD_WR_REG(0xD0);     
	LCD_WR_DATA8(0xA7);   

	LCD_WR_REG(0xD0);     
	LCD_WR_DATA8(0xA4);   
	LCD_WR_DATA8(0xA1);   

	LCD_WR_REG(0xD6);     
	LCD_WR_DATA8(0xA1);   

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0xF0);
	LCD_WR_DATA8(0x0F);
	LCD_WR_DATA8(0x16);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x08);
	LCD_WR_DATA8(0x25);
	LCD_WR_DATA8(0x36);
	LCD_WR_DATA8(0x54);
	LCD_WR_DATA8(0x4B);
	LCD_WR_DATA8(0x37);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x2E);
	LCD_WR_DATA8(0x35);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0xF0);
	LCD_WR_DATA8(0x10);
	LCD_WR_DATA8(0x14);
	LCD_WR_DATA8(0x08);
	LCD_WR_DATA8(0x06);
	LCD_WR_DATA8(0x23);
	LCD_WR_DATA8(0x35);
	LCD_WR_DATA8(0x44);
	LCD_WR_DATA8(0x4A);
	LCD_WR_DATA8(0x38);
	LCD_WR_DATA8(0x10);
	LCD_WR_DATA8(0x10);
	LCD_WR_DATA8(0x2C);
	LCD_WR_DATA8(0x33);

	LCD_WR_REG(0x21);     

	LCD_WR_REG(0x29);
}
