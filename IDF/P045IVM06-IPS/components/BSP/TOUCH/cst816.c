#include "cst816.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TOUCH_OFFSET_Y 5

static iic_bus_t s_touch_i2c = {
    .sda_pin = TOUCH_I2C_SDA_PIN,
    .scl_pin = TOUCH_I2C_SCL_PIN,
};

static inline void cst816_write_reg(uint8_t reg, uint8_t val)
{
    iic_write_one_byte(&s_touch_i2c, CST816_ADDR, reg, val);
}

static inline uint8_t cst816_read_reg(uint8_t reg)
{
    return iic_read_one_byte(&s_touch_i2c, CST816_ADDR, reg);
}

static void cst816_reset(void)
{
    gpio_set_level(TOUCH_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(TOUCH_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

esp_err_t cst816_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TOUCH_RST_PIN),
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(TOUCH_RST_PIN, 1);
    iic_init(&s_touch_i2c);
    cst816_reset();

    /* Auto sleep after 5s idle, same as reference project */
    cst816_write_reg(CST816_REG_AUTO_SLEEP, 5);

    return ESP_OK;
}

uint8_t cst816_get_chip_id(void)
{
    return cst816_read_reg(CST816_REG_CHIP_ID);
}

bool cst816_read(cst816_state_t *state)
{
    if (state == NULL) {
        return false;
    }

    uint8_t raw[4] = {0};
    state->finger_num = cst816_read_reg(CST816_REG_FINGER_NUM);

    if (state->finger_num == 0 || state->finger_num == 0xFF) {
        return false;
    }

    iic_read_multi_byte(&s_touch_i2c, CST816_ADDR, CST816_REG_XPOS_H, 4, raw);
    state->x = ((raw[0] & 0x0F) << 8) | raw[1];
    state->y = (((raw[2] & 0x0F) << 8) | raw[3]) + TOUCH_OFFSET_Y;

    return true;
}
