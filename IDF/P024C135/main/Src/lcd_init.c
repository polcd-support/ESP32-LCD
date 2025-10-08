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
	LCD_WR_REG(0x2a); // 列地址设置
	LCD_WR_DATA(x1);
	LCD_WR_DATA(x2);
	LCD_WR_REG(0x2b); // 行地址设置
	LCD_WR_DATA(y1);
	LCD_WR_DATA(y2);
	LCD_WR_REG(0x2c); // 储存器写
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
	LCD_WR_REG(0xCF);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0xD9);
	LCD_WR_DATA8(0X30);

	LCD_WR_REG(0xED);
	LCD_WR_DATA8(0x64);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0X12);
	LCD_WR_DATA8(0X81);

	LCD_WR_REG(0xE8);
	LCD_WR_DATA8(0x85);
	LCD_WR_DATA8(0x10);
	LCD_WR_DATA8(0x78);

	LCD_WR_REG(0xCB);
	LCD_WR_DATA8(0x39);
	LCD_WR_DATA8(0x2C);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x34);
	LCD_WR_DATA8(0x02);

	LCD_WR_REG(0xF7);
	LCD_WR_DATA8(0x20);

	LCD_WR_REG(0xEA);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x00);

	LCD_WR_REG(0xC0);	// Power control
	LCD_WR_DATA8(0x21); // VRH[5:0]

	LCD_WR_REG(0xC1);	// Power control
	LCD_WR_DATA8(0x12); // SAP[2:0];BT[3:0]

	LCD_WR_REG(0xC5); // VCM control
	LCD_WR_DATA8(0x32);
	LCD_WR_DATA8(0x3C);

	LCD_WR_REG(0xC7); // VCM control2
	LCD_WR_DATA8(0XC1);

	LCD_WR_REG(0x36); // Memory Access Control
	if (USE_HORIZONTAL == 0)
		LCD_WR_DATA8(0x08);
	else if (USE_HORIZONTAL == 1)
		LCD_WR_DATA8(0xC8);
	else if (USE_HORIZONTAL == 2)
		LCD_WR_DATA8(0x78);
	else
		LCD_WR_DATA8(0xA8);

	LCD_WR_REG(0x3A);
	LCD_WR_DATA8(0x55);

	LCD_WR_REG(0xB1);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x18);

	LCD_WR_REG(0xB6); // Display Function Control
	LCD_WR_DATA8(0x0A);
	LCD_WR_DATA8(0xA2);

	LCD_WR_REG(0xF2); // 3Gamma Function Disable
	LCD_WR_DATA8(0x00);

	LCD_WR_REG(0x26); // Gamma curve selected
	LCD_WR_DATA8(0x01);

	LCD_WR_REG(0xE0); // Set Gamma
	LCD_WR_DATA8(0x0F);
	LCD_WR_DATA8(0x20);
	LCD_WR_DATA8(0x1E);
	LCD_WR_DATA8(0x09);
	LCD_WR_DATA8(0x12);
	LCD_WR_DATA8(0x0B);
	LCD_WR_DATA8(0x50);
	LCD_WR_DATA8(0XBA);
	LCD_WR_DATA8(0x44);
	LCD_WR_DATA8(0x09);
	LCD_WR_DATA8(0x14);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x23);
	LCD_WR_DATA8(0x21);
	LCD_WR_DATA8(0x00);

	LCD_WR_REG(0XE1); // Set Gamma
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x19);
	LCD_WR_DATA8(0x19);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x12);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x2D);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x3F);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x0A);
	LCD_WR_DATA8(0x08);
	LCD_WR_DATA8(0x25);
	LCD_WR_DATA8(0x2D);
	LCD_WR_DATA8(0x0F);
	LCD_WR_REG(0x29); // Display on
}
