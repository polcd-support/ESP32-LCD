#include "../Inc/lcd_init.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

spi_device_handle_t spi = NULL;

static const spi_bus_config_t buscfg = {
	.miso_io_num = LCD_MISO_PIN,
	.mosi_io_num = LCD_MOSI_PIN,
	.sclk_io_num = LCD_SCK_PIN,
	.quadwp_io_num = -1,
	.quadhd_io_num = -1,
	.max_transfer_sz = 4096,
};

static const spi_device_interface_config_t devcfg = {
	.clock_speed_hz = SPI_MASTER_FREQ_10M,
	.mode = 0,					// SPI模式0
	.spics_io_num = LCD_CS_PIN, // CS引脚
	.queue_size = 7,			// 我们希望能够一次排队7个事务
	.flags = SPI_DEVICE_HALFDUPLEX,
};

static void LCD_GPIO_Init(void)
{
	esp_err_t ret;

	gpio_config_t io_conf = {
		.pin_bit_mask = ((1ULL << LCD_RES_PIN) | (1ULL << LCD_DC_PIN) | (1ULL << LCD_BLK_PIN)),
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.pull_up_en = GPIO_PULLUP_DISABLE,
	};

	gpio_config(&io_conf);
	// 初始化SPI总线
	ret = spi_bus_initialize(LCD_SPI_PORT, &buscfg, SPI_DMA_CH_AUTO);
	assert(ret == ESP_OK);
	ret = spi_bus_add_device(LCD_SPI_PORT, &devcfg, &spi);
	assert(ret == ESP_OK);

	LCD_RES_Set();
	LCD_DC_Set();
	LCD_BLK_Set();
}
/******************************************************************************
	  函数说明：LCD串行数据写入函数
	  入口数据：dat  要写入的串行数据
	  返回值：  无
******************************************************************************/
static inline void LCD_Writ_Bus(uint8_t dat)
{
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8;
	t.tx_buffer = &dat;

	ret = spi_device_polling_transmit(spi, &t); // Transmit!
	assert(ret == ESP_OK);
}

/******************************************************************************
	  函数说明：LCD写入数据
	  入口数据：dat 写入的数据
	  返回值：  无
******************************************************************************/
static inline void LCD_WR_DATA8(uint8_t dat)
{
	LCD_Writ_Bus(dat);
}

/******************************************************************************
	  函数说明：LCD写入数据
	  入口数据：dat 写入的数据
	  返回值：  无
******************************************************************************/
inline void LCD_WR_DATA(uint16_t dat)
{
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	uint8_t tx_buf[2] = {
		(dat >> 8) & 0xFF, // 高字节
		dat & 0xFF		   // 低字节
	};
	t.length = 16;

	t.tx_buffer = tx_buf;

	ret = spi_device_polling_transmit(spi, &t); // Transmit!
	assert(ret == ESP_OK);
}

/******************************************************************************
	  函数说明：LCD写入命令
	  入口数据：dat 写入的命令
	  返回值：  无
******************************************************************************/
static inline void LCD_WR_REG(uint8_t dat)
{
	LCD_DC_Clr(); // 写命令
	LCD_Writ_Bus(dat);
	LCD_DC_Set();
}

/******************************************************************************
	  函数说明：设置起始和结束地址
	  入口数据：x1,x2 设置列的起始和结束地址
				y1,y2 设置行的起始和结束地址
	  返回值：  无
******************************************************************************/
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	if (USE_HORIZONTAL == 0)
	{
		LCD_WR_REG(0x2a); //列地址设置
		LCD_WR_DATA(x1 );
		LCD_WR_DATA(x2 );
		LCD_WR_REG(0x2b); //行地址设置
		LCD_WR_DATA(y1 + 20);
		LCD_WR_DATA(y2 + 20);
		LCD_WR_REG(0x2c); //储存器写
	}
	else if (USE_HORIZONTAL == 1)
	{
		LCD_WR_REG(0x2a); //列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b); //行地址设置
		LCD_WR_DATA(y1 + 80);
		LCD_WR_DATA(y2 + 80);
		LCD_WR_REG(0x2c); //储存器写
	}
	else if (USE_HORIZONTAL == 2)
	{
		LCD_WR_REG(0x2a); //列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b); //行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c); //储存器写
	}
	else
	{
		LCD_WR_REG(0x2a); //列地址设置
		LCD_WR_DATA(x1 + 80);
		LCD_WR_DATA(x2 + 80);
		LCD_WR_REG(0x2b); //行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c); //储存器写
	}
}

void LCD_Init(void)
{
	LCD_GPIO_Init(); // 初始化GPIO

	LCD_RES_Clr(); // 复位
	vTaskDelay(pdMS_TO_TICKS(100));
	LCD_RES_Set();
	vTaskDelay(pdMS_TO_TICKS(100));

	//************* Start Initial Sequence **********//
	LCD_WR_REG(0x11);				// Sleep out
	vTaskDelay(pdMS_TO_TICKS(120)); // Delay 120ms
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

	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x33);
	LCD_WR_DATA8(0x33);

	LCD_WR_REG(0xB7);
	LCD_WR_DATA8(0x35);

	LCD_WR_REG(0xBB);
	LCD_WR_DATA8(0x32); // Vcom=1.35V

	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x01);

	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x15); // GVDD=4.8V  颜色深度

	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x20); // VDV, 0x20:0v

	LCD_WR_REG(0xC6);
	LCD_WR_DATA8(0x0F); // 0x0F:60Hz

	LCD_WR_REG(0xD0);
	LCD_WR_DATA8(0xA4);
	LCD_WR_DATA8(0xA1);

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0xD0);
	LCD_WR_DATA8(0x08);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x09);
	LCD_WR_DATA8(0x09);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x31);
	LCD_WR_DATA8(0x33);
	LCD_WR_DATA8(0x48);
	LCD_WR_DATA8(0x17);
	LCD_WR_DATA8(0x14);
	LCD_WR_DATA8(0x15);
	LCD_WR_DATA8(0x31);
	LCD_WR_DATA8(0x34);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0xD0);
	LCD_WR_DATA8(0x08);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x09);
	LCD_WR_DATA8(0x09);
	LCD_WR_DATA8(0x15);
	LCD_WR_DATA8(0x31);
	LCD_WR_DATA8(0x33);
	LCD_WR_DATA8(0x48);
	LCD_WR_DATA8(0x17);
	LCD_WR_DATA8(0x14);
	LCD_WR_DATA8(0x15);
	LCD_WR_DATA8(0x31);
	LCD_WR_DATA8(0x34);
	LCD_WR_REG(0x21);

	LCD_WR_REG(0x29);
}
