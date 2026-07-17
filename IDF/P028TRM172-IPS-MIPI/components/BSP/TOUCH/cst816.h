#ifndef __CST816_H
#define __CST816_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "iic_hal.h"

#define CST816_ADDR        0x15

/* Default pins, update these for your board if needed */
#define TOUCH_INT_PIN      21
#define TOUCH_I2C_SCL_PIN  32
#define TOUCH_I2C_SDA_PIN  33
#define TOUCH_RST_PIN      45

#define CST816_REG_FINGER_NUM 0x02
#define CST816_REG_XPOS_H     0x03
#define CST816_REG_CHIP_ID    0xA7
#define CST816_REG_AUTO_SLEEP 0xF9

typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t finger_num;
} cst816_state_t;

esp_err_t cst816_init(void);
uint8_t cst816_get_chip_id(void);
bool cst816_read(cst816_state_t *state);

#endif
