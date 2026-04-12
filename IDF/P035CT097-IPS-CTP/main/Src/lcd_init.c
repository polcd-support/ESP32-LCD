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
	.clock_speed_hz = SPI_MASTER_FREQ_40M,
	.mode = 0,					// SPIģʽ0
	.spics_io_num = LCD_CS_PIN, // CS����
	.queue_size = 7,			// ����ϣ���ܹ�һ���Ŷ�7������
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
	// ��ʼ��SPI����
	ret = spi_bus_initialize(LCD_SPI_PORT, &buscfg, SPI_DMA_CH_AUTO);
	assert(ret == ESP_OK);
	ret = spi_bus_add_device(LCD_SPI_PORT, &devcfg, &spi);
	assert(ret == ESP_OK);

	LCD_RES_Set();
	LCD_DC_Set();
	LCD_BLK_Set();
}
/******************************************************************************
	  ����˵����LCD��������д�뺯��
	  ������ݣ�dat  Ҫд��Ĵ�������
	  ����ֵ��  ��
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
	  ����˵����LCDд������
	  ������ݣ�dat д�������
	  ����ֵ��  ��
******************************************************************************/
static inline void LCD_WR_DATA8(uint8_t dat)
{
	LCD_Writ_Bus(dat);
}

/******************************************************************************
	  ����˵����LCDд������
	  ������ݣ�dat д�������
	  ����ֵ��  ��
******************************************************************************/
inline void LCD_WR_DATA(uint16_t dat)
{
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	uint8_t tx_buf[3] = {
		(dat >> 8) & 0xF8, // ���ֽ�
		(dat >> 3) & 0xFC, // ���ֽ�
		dat << 3,
	};
	t.length = 24;

	t.tx_buffer = tx_buf;

	ret = spi_device_polling_transmit(spi, &t); // Transmit!
	assert(ret == ESP_OK);
}

/******************************************************************************
	  ����˵����LCDд������
	  ������ݣ�dat д�������
	  ����ֵ��  ��
******************************************************************************/
static inline void LCD_WR_REG(uint8_t dat)
{
	LCD_DC_Clr(); // д����
	LCD_Writ_Bus(dat);
	LCD_DC_Set();
}

/******************************************************************************
	  ����˵����������ʼ�ͽ�����ַ
	  ������ݣ�x1,x2 �����е���ʼ�ͽ�����ַ
				y1,y2 �����е���ʼ�ͽ�����ַ
	  ����ֵ��  ��
******************************************************************************/
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	LCD_WR_REG(0x2a); // �е�ַ����
	LCD_WR_DATA8(x1 >> 8);
	LCD_WR_DATA8(x1);
	LCD_WR_DATA8(x2 >> 8);
	LCD_WR_DATA8(x2);
	LCD_WR_REG(0x2b); // �е�ַ����
	LCD_WR_DATA8(y1 >> 8);
	LCD_WR_DATA8(y1);
	LCD_WR_DATA8(y2 >> 8);
	LCD_WR_DATA8(y2);
	LCD_WR_REG(0x2c); // ������д
}

void LCD_Init(void)
{
	LCD_GPIO_Init(); // ��ʼ��GPIO

	LCD_RES_Clr(); // ��λ
	vTaskDelay(pdMS_TO_TICKS(100));
	LCD_RES_Set();
	vTaskDelay(pdMS_TO_TICKS(100));

	//************* Start Initial Sequence **********//
	LCD_WR_REG(0x11);				// Sleep out
	vTaskDelay(pdMS_TO_TICKS(120)); // Delay 120ms
	LCD_WR_REG(0xf0);
	LCD_WR_DATA8(0xc3);
	LCD_WR_REG(0xf0);
	LCD_WR_DATA8(0x96);
	LCD_WR_REG(0x36);
	LCD_WR_DATA8(0x48);
	LCD_WR_REG(0xB4);
	LCD_WR_DATA8(0x01);
	LCD_WR_REG(0x3A);
	LCD_WR_DATA8(0x77);
	LCD_WR_REG(0xB7);
	LCD_WR_DATA8(0xC6);
	LCD_WR_REG(0xe8);
	LCD_WR_DATA8(0x40);
	LCD_WR_DATA8(0x8a);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x29);
	LCD_WR_DATA8(0x19);
	LCD_WR_DATA8(0xa5);
	LCD_WR_DATA8(0x33);
	LCD_WR_REG(0xc1);
	LCD_WR_DATA8(0x06);
	LCD_WR_REG(0xc2);
	LCD_WR_DATA8(0xa5);
	LCD_WR_REG(0xc5);
	LCD_WR_DATA8(0x25);
	LCD_WR_REG(0xe0);
	LCD_WR_DATA8(0xf0);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x0b);
	LCD_WR_DATA8(0x0c);
	LCD_WR_DATA8(0x29);
	LCD_WR_DATA8(0x2e);
	LCD_WR_DATA8(0x44);
	LCD_WR_DATA8(0x41);
	LCD_WR_DATA8(0x17);
	LCD_WR_DATA8(0x11);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x16);
	LCD_WR_DATA8(0x1b);
	LCD_WR_REG(0xe1);
	LCD_WR_DATA8(0xf0);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x06);
	LCD_WR_DATA8(0x06);
	LCD_WR_DATA8(0x24);
	LCD_WR_DATA8(0x2a);
	LCD_WR_DATA8(0x43);
	LCD_WR_DATA8(0x3e);
	LCD_WR_DATA8(0x2d);
	LCD_WR_DATA8(0x1a);
	LCD_WR_DATA8(0x16);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x17);
	LCD_WR_REG(0xB9);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0xC0);
	LCD_WR_REG(0xf0);
	LCD_WR_DATA8(0x3c);
	LCD_WR_REG(0xf0);
	LCD_WR_DATA8(0x69);
	vTaskDelay(pdMS_TO_TICKS(120)); 
	LCD_WR_REG(0x21);
	LCD_WR_REG(0x29);
}
