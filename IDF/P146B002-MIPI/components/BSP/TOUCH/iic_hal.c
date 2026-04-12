#include "iic_hal.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

#define IIC_DELAY_US 2

static void iic_start(iic_bus_t *bus)
{
    gpio_set_level(bus->sda_pin, 1);
    gpio_set_level(bus->scl_pin, 1);
    esp_rom_delay_us(IIC_DELAY_US);
    gpio_set_level(bus->sda_pin, 0);
    esp_rom_delay_us(IIC_DELAY_US);
    gpio_set_level(bus->scl_pin, 0);
}

static void iic_stop(iic_bus_t *bus)
{
    gpio_set_level(bus->scl_pin, 0);
    gpio_set_level(bus->sda_pin, 0);
    esp_rom_delay_us(IIC_DELAY_US);
    gpio_set_level(bus->scl_pin, 1);
    esp_rom_delay_us(IIC_DELAY_US);
    gpio_set_level(bus->sda_pin, 1);
    esp_rom_delay_us(IIC_DELAY_US);
}

static uint8_t iic_wait_ack(iic_bus_t *bus)
{
    uint16_t timeout = 0x0FFF;

    gpio_set_level(bus->scl_pin, 0);
    gpio_set_direction(bus->sda_pin, GPIO_MODE_INPUT);
    esp_rom_delay_us(IIC_DELAY_US);
    gpio_set_level(bus->scl_pin, 1);

    while (gpio_get_level(bus->sda_pin)) {
        if ((timeout--) == 0) {
            gpio_set_direction(bus->sda_pin, GPIO_MODE_OUTPUT);
            iic_stop(bus);
            return 1;
        }
    }

    esp_rom_delay_us(IIC_DELAY_US);
    gpio_set_level(bus->scl_pin, 0);
    gpio_set_direction(bus->sda_pin, GPIO_MODE_OUTPUT);
    return 0;
}

static void iic_ack(iic_bus_t *bus)
{
    gpio_set_level(bus->scl_pin, 0);
    gpio_set_direction(bus->sda_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(bus->sda_pin, 0);
    esp_rom_delay_us(IIC_DELAY_US);
    gpio_set_level(bus->scl_pin, 1);
    esp_rom_delay_us(IIC_DELAY_US);
    gpio_set_level(bus->scl_pin, 0);
}

static void iic_nack(iic_bus_t *bus)
{
    gpio_set_level(bus->scl_pin, 0);
    gpio_set_direction(bus->sda_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(bus->sda_pin, 1);
    esp_rom_delay_us(IIC_DELAY_US);
    gpio_set_level(bus->scl_pin, 1);
    esp_rom_delay_us(IIC_DELAY_US);
    gpio_set_level(bus->scl_pin, 0);
}

static void iic_send_byte(iic_bus_t *bus, uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++) {
        gpio_set_level(bus->scl_pin, 0);
        esp_rom_delay_us(IIC_DELAY_US);
        gpio_set_level(bus->sda_pin, (data & 0x80) ? 1 : 0);
        gpio_set_level(bus->scl_pin, 1);
        esp_rom_delay_us(IIC_DELAY_US);
        data <<= 1;
    }
}

static uint8_t iic_recv_byte(iic_bus_t *bus)
{
    uint8_t receive = 0;
    gpio_set_direction(bus->sda_pin, GPIO_MODE_INPUT);

    for (uint8_t i = 0; i < 8; i++) {
        receive <<= 1;
        gpio_set_level(bus->scl_pin, 0);
        esp_rom_delay_us(IIC_DELAY_US);
        gpio_set_level(bus->scl_pin, 1);
        if (gpio_get_level(bus->sda_pin)) {
            receive |= 0x01;
        }
        esp_rom_delay_us(IIC_DELAY_US);
    }

    return receive;
}

void iic_init(iic_bus_t *bus)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << bus->sda_pin) | (1ULL << bus->scl_pin),
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    gpio_config(&io_conf);
    gpio_set_level(bus->sda_pin, 1);
    gpio_set_level(bus->scl_pin, 1);
}

uint8_t iic_read_one_byte(iic_bus_t *bus, uint8_t dev_addr, uint8_t reg_addr)
{
    uint8_t res;

    iic_start(bus);
    iic_send_byte(bus, dev_addr << 1);
    iic_wait_ack(bus);
    iic_send_byte(bus, reg_addr);
    iic_wait_ack(bus);

    iic_start(bus);
    iic_send_byte(bus, (dev_addr << 1) | 1);
    iic_wait_ack(bus);
    res = iic_recv_byte(bus);
    iic_nack(bus);
    iic_stop(bus);

    return res;
}

void iic_write_one_byte(iic_bus_t *bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t dat)
{
    iic_start(bus);
    iic_send_byte(bus, dev_addr << 1);
    iic_wait_ack(bus);
    iic_send_byte(bus, reg_addr);
    iic_wait_ack(bus);
    iic_send_byte(bus, dat);
    iic_wait_ack(bus);
    iic_stop(bus);
}

void iic_read_multi_byte(iic_bus_t *bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t *buf)
{
    iic_start(bus);
    iic_send_byte(bus, dev_addr << 1);
    iic_wait_ack(bus);
    iic_send_byte(bus, reg_addr);
    iic_wait_ack(bus);

    iic_start(bus);
    iic_send_byte(bus, (dev_addr << 1) | 1);
    iic_wait_ack(bus);

    while (len) {
        *buf = iic_recv_byte(bus);
        if (len == 1) {
            iic_nack(bus);
        } else {
            iic_ack(bus);
        }
        buf++;
        len--;
    }

    iic_stop(bus);
}
