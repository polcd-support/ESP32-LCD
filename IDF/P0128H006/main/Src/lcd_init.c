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
    .mode = 0,
    .spics_io_num = LCD_CS_PIN,
    .queue_size = 7,
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

    ret = spi_bus_initialize(LCD_SPI_PORT, &buscfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);
    ret = spi_bus_add_device(LCD_SPI_PORT, &devcfg, &spi);
    assert(ret == ESP_OK);

    LCD_RES_Set();
    LCD_DC_Set();
    LCD_BLK_Set();
}

static inline void LCD_Writ_Bus(uint8_t dat)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &dat;

    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static inline void LCD_WR_DATA8(uint8_t dat)
{
    LCD_Writ_Bus(dat);
}

inline void LCD_WR_DATA(uint16_t dat)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    uint8_t tx_buf[2] = {
        (uint8_t)((dat >> 8) & 0xFF),
        (uint8_t)(dat & 0xFF),
    };

    t.length = 16;
    t.tx_buffer = tx_buf;

    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static inline void LCD_WR_REG(uint8_t dat)
{
    LCD_DC_Clr();
    LCD_Writ_Bus(dat);
    LCD_DC_Set();
}

void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_WR_REG(0x2A);
    LCD_WR_DATA(x1);
    LCD_WR_DATA(x2);
    LCD_WR_REG(0x2B);
    LCD_WR_DATA(y1);
    LCD_WR_DATA(y2);
    LCD_WR_REG(0x2C);
}

void LCD_Init(void)
{
    LCD_GPIO_Init();

    LCD_RES_Clr();
    vTaskDelay(pdMS_TO_TICKS(100));
    LCD_RES_Set();
    vTaskDelay(pdMS_TO_TICKS(100));

    LCD_WR_REG(0xEF);
    LCD_WR_REG(0xEB);
    LCD_WR_DATA8(0x14);

    LCD_WR_REG(0xFE);
    LCD_WR_REG(0xEF);

    LCD_WR_REG(0xEB);
    LCD_WR_DATA8(0x14);

    LCD_WR_REG(0x84);
    LCD_WR_DATA8(0x40);

    LCD_WR_REG(0x85);
    LCD_WR_DATA8(0xFF);

    LCD_WR_REG(0x86);
    LCD_WR_DATA8(0xFF);

    LCD_WR_REG(0x87);
    LCD_WR_DATA8(0xFF);

    LCD_WR_REG(0x88);
    LCD_WR_DATA8(0x0A);

    LCD_WR_REG(0x89);
    LCD_WR_DATA8(0x21);

    LCD_WR_REG(0x8A);
    LCD_WR_DATA8(0x00);

    LCD_WR_REG(0x8B);
    LCD_WR_DATA8(0x80);

    LCD_WR_REG(0x8C);
    LCD_WR_DATA8(0x01);

    LCD_WR_REG(0x8D);
    LCD_WR_DATA8(0x01);

    LCD_WR_REG(0x8E);
    LCD_WR_DATA8(0xFF);

    LCD_WR_REG(0x8F);
    LCD_WR_DATA8(0xFF);

    LCD_WR_REG(0xB6);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x20);

    LCD_WR_REG(0x36);
#if (USE_HORIZONTAL == 0)
    LCD_WR_DATA8(0x08);
#elif (USE_HORIZONTAL == 1)
    LCD_WR_DATA8(0xC8);
#elif (USE_HORIZONTAL == 2)
    LCD_WR_DATA8(0x68);
#else
    LCD_WR_DATA8(0xA8);
#endif

    LCD_WR_REG(0x3A);
    LCD_WR_DATA8(0x05);

    LCD_WR_REG(0x90);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x08);

    LCD_WR_REG(0xBD);
    LCD_WR_DATA8(0x06);

    LCD_WR_REG(0xBC);
    LCD_WR_DATA8(0x00);

    LCD_WR_REG(0xFF);
    LCD_WR_DATA8(0x60);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x04);

    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x13);
    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x13);

    LCD_WR_REG(0xC9);
    LCD_WR_DATA8(0x22);

    LCD_WR_REG(0xBE);
    LCD_WR_DATA8(0x11);

    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0x10);
    LCD_WR_DATA8(0x0E);

    LCD_WR_REG(0xDF);
    LCD_WR_DATA8(0x21);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x02);

    LCD_WR_REG(0xF0);
    LCD_WR_DATA8(0x45);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x26);
    LCD_WR_DATA8(0x2A);

    LCD_WR_REG(0xF1);
    LCD_WR_DATA8(0x43);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x72);
    LCD_WR_DATA8(0x36);
    LCD_WR_DATA8(0x37);
    LCD_WR_DATA8(0x6F);

    LCD_WR_REG(0xF2);
    LCD_WR_DATA8(0x45);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x26);
    LCD_WR_DATA8(0x2A);

    LCD_WR_REG(0xF3);
    LCD_WR_DATA8(0x43);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x72);
    LCD_WR_DATA8(0x36);
    LCD_WR_DATA8(0x37);
    LCD_WR_DATA8(0x6F);

    LCD_WR_REG(0xED);
    LCD_WR_DATA8(0x1B);
    LCD_WR_DATA8(0x0B);

    LCD_WR_REG(0xAE);
    LCD_WR_DATA8(0x77);

    LCD_WR_REG(0xCD);
    LCD_WR_DATA8(0x63);

    LCD_WR_REG(0x70);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x0E);
    LCD_WR_DATA8(0x0F);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x03);

    LCD_WR_REG(0xE8);
    LCD_WR_DATA8(0x34);

    LCD_WR_REG(0x62);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x71);
    LCD_WR_DATA8(0xED);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x0F);
    LCD_WR_DATA8(0x71);
    LCD_WR_DATA8(0xEF);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x70);

    LCD_WR_REG(0x63);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x11);
    LCD_WR_DATA8(0x71);
    LCD_WR_DATA8(0xF1);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x13);
    LCD_WR_DATA8(0x71);
    LCD_WR_DATA8(0xF3);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x70);

    LCD_WR_REG(0x64);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x29);
    LCD_WR_DATA8(0xF1);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0xF1);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x07);

    LCD_WR_REG(0x66);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0xCD);
    LCD_WR_DATA8(0x67);
    LCD_WR_DATA8(0x45);
    LCD_WR_DATA8(0x45);
    LCD_WR_DATA8(0x10);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);

    LCD_WR_REG(0x67);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x54);
    LCD_WR_DATA8(0x10);
    LCD_WR_DATA8(0x32);
    LCD_WR_DATA8(0x98);

    LCD_WR_REG(0x74);
    LCD_WR_DATA8(0x10);
    LCD_WR_DATA8(0x85);
    LCD_WR_DATA8(0x80);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x4E);
    LCD_WR_DATA8(0x00);

    LCD_WR_REG(0x98);
    LCD_WR_DATA8(0x3E);
    LCD_WR_DATA8(0x07);

    LCD_WR_REG(0x35);
    LCD_WR_REG(0x21);

    LCD_WR_REG(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));
    LCD_WR_REG(0x29);
    vTaskDelay(pdMS_TO_TICKS(20));
}
