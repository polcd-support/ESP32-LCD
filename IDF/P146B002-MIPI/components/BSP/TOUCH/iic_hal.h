#ifndef __IIC_HAL_H
#define __IIC_HAL_H

#include <stdint.h>

typedef struct {
    uint8_t sda_pin;
    uint8_t scl_pin;
} iic_bus_t;

void iic_init(iic_bus_t *bus);
uint8_t iic_read_one_byte(iic_bus_t *bus, uint8_t dev_addr, uint8_t reg_addr);
void iic_write_one_byte(iic_bus_t *bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t dat);
void iic_read_multi_byte(iic_bus_t *bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t *buf);

#endif
