#include "ft6336.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static iic_bus_t s_touch_i2c = {
    .sda_pin = TOUCH_I2C_SDA_PIN,
    .scl_pin = TOUCH_I2C_SCL_PIN,
};

static inline uint8_t ft6336_read_reg(uint8_t reg)
{
    return iic_read_one_byte(&s_touch_i2c, FT6336_ADDR, reg);
}

static inline void ft6336_write_reg(uint8_t reg, uint8_t value)
{
    iic_write_one_byte(&s_touch_i2c, FT6336_ADDR, reg, value);
}

static void ft6336_reset(void)
{
    gpio_set_level(TOUCH_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(TOUCH_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
}

esp_err_t ft6336_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TOUCH_RST_PIN) | (1ULL << TOUCH_INT_PIN),
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_direction(TOUCH_INT_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(TOUCH_RST_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(TOUCH_RST_PIN, 1);
    iic_init(&s_touch_i2c);
    ft6336_reset();

    return ESP_OK;
}

uint8_t ft6336_get_chip_id(void)
{
    return ft6336_read_reg(FT6336_REG_CHIP_ID);
}

uint8_t ft6336_get_vendor_id(void)
{
    return ft6336_read_reg(FT6336_REG_VENDOR_ID);
}

bool ft6336_read(ft6336_state_t *state)
{
    if (state == NULL) {
        return false;
    }

    state->finger_num = ft6336_read_reg(FT6336_REG_TD_STATUS) & 0x0F;

    if (state->finger_num == 0) {
        return false;
    }

    uint8_t xh = ft6336_read_reg(FT6336_REG_P1_XH);
    uint8_t xl = ft6336_read_reg(FT6336_REG_P1_XH + 1);
    uint8_t yh = ft6336_read_reg(FT6336_REG_P1_XH + 2);
    uint8_t yl = ft6336_read_reg(FT6336_REG_P1_XH + 3);
    uint16_t raw_x = ((xh & 0x0F) << 8) | xl;
    uint16_t raw_y = ((yh & 0x0F) << 8) | yl;

    state->raw_x = raw_x;
    state->raw_y = raw_y;
    state->x = raw_x;
    state->y = raw_y;

    return true;
}
