#ifndef __FT6336_H
#define __FT6336_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "iic_hal.h"

#define FT6336_ADDR        0x38

/* Default pins, update these for your board if needed */
#define TOUCH_INT_PIN      21
#define TOUCH_I2C_SCL_PIN  32
#define TOUCH_I2C_SDA_PIN  33
#define TOUCH_RST_PIN      45

#define FT6336_REG_MODE      0x00
#define FT6336_REG_TD_STATUS 0x02
#define FT6336_REG_P1_XH     0x03
#define FT6336_REG_THRESHOLD 0x80
#define FT6336_REG_FILTER_COE 0x85
#define FT6336_REG_CTRL      0x86
#define FT6336_REG_PERIODACTIVE 0x88
#define FT6336_REG_CHIP_ID   0xA3
#define FT6336_REG_G_MODE    0xA4
#define FT6336_REG_POWER_MODE 0xA5
#define FT6336_REG_VENDOR_ID 0xA8

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t raw_x;
    uint16_t raw_y;
    uint8_t finger_num;
} ft6336_state_t;

esp_err_t ft6336_init(void);
uint8_t ft6336_get_chip_id(void);
uint8_t ft6336_get_vendor_id(void);
bool ft6336_read(ft6336_state_t *state);

#endif
