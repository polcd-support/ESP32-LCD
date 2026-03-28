#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Inc/ft6236.h"

#define TOUCH_OFFSET_X 0
#define TOUCH_OFFSET_Y 0
#define REVERSE_X 0
#define REVERSE_Y 0

#define CST816_ADDR 0x15
#define CST816_REG_FINGER_NUM 0x02
#define CST816_REG_XH 0x03
#define CST816_REG_CHIP_ID 0xA7
#define CST816_REG_AUTO_SLEEP 0xF9

typedef enum
{
    TOUCH_IC_UNKNOWN = 0,
    TOUCH_IC_FT6236,
    TOUCH_IC_CST816,
} touch_ic_t;

static const char *TAG = "touch";
static touch_ic_t s_touch_ic = TOUCH_IC_UNKNOWN;
static uint8_t s_touch_addr = FT6236_ADDR;

FT6236_Info FT6236_Instance;

iic_bus_t FT6236_dev = {
    .sda_pin = I2C0_SDA_PIN,
    .scl_pin = I2C0_SCL_PIN,
};

static uint8_t touch_read_reg(uint8_t dev_addr, uint8_t reg)
{
    return IIC_Read_One_Byte(&FT6236_dev, dev_addr, reg);
}

static void touch_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val)
{
    IIC_Write_One_Byte(&FT6236_dev, dev_addr, reg, val);
}

static uint8_t touch_read_reg_active(uint8_t reg)
{
    return touch_read_reg(s_touch_addr, reg);
}

static void touch_write_reg_active(uint8_t reg, uint8_t val)
{
    touch_write_reg(s_touch_addr, reg, val);
}

void FT6236_GPIO_Init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TOUCH_RST_PIN),
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    gpio_config(&io_conf);
    TOUCH_RST_1;

    IICInit(&FT6236_dev);
}

void FT6236_RESET(void)
{
    TOUCH_RST_0;
    vTaskDelay(pdMS_TO_TICKS(10));
    TOUCH_RST_1;
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void touch_detect_ic(void)
{
    uint8_t cst_id = touch_read_reg(CST816_ADDR, CST816_REG_CHIP_ID);
    uint8_t ft_id = touch_read_reg(FT6236_ADDR, FT6236_REG_CHIP_ID);

    if (cst_id != 0x00 && cst_id != 0xFF)
    {
        s_touch_ic = TOUCH_IC_CST816;
        s_touch_addr = CST816_ADDR;
        ESP_LOGI(TAG, "Detected CST816, chip id=0x%02X", cst_id);
        return;
    }

    if (ft_id != 0x00 && ft_id != 0xFF)
    {
        s_touch_ic = TOUCH_IC_FT6236;
        s_touch_addr = FT6236_ADDR;
        ESP_LOGI(TAG, "Detected FT6236, chip id=0x%02X", ft_id);
        return;
    }

    s_touch_ic = TOUCH_IC_FT6236;
    s_touch_addr = FT6236_ADDR;
    ESP_LOGW(TAG, "Touch IC detect failed, fallback FT6236 (ft=0x%02X cst=0x%02X)", ft_id, cst_id);
}

void FT6236_Init(void)
{
    FT6236_GPIO_Init();
    FT6236_RESET();
    touch_detect_ic();

    if (s_touch_ic == TOUCH_IC_CST816)
    {
        touch_write_reg_active(CST816_REG_AUTO_SLEEP, 5);
        return;
    }

    FT6236_Set_Threshold(128);
    FT6236_Set_InterruptMode(FT6236_INT_MODE_TRIGGER);
    FT6236_Set_ActiveRate(10);
}

uint8_t FT6236_IIC_ReadREG(uint8_t addr)
{
    return touch_read_reg_active(addr);
}

void FT6236_IIC_WriteREG(uint8_t addr, uint8_t dat)
{
    touch_write_reg_active(addr, dat);
}

void FT6236_Get_Touch_Data(void)
{
    uint8_t data[4] = {0};

    if (s_touch_ic == TOUCH_IC_CST816)
    {
        uint8_t finger_num = touch_read_reg(CST816_ADDR, CST816_REG_FINGER_NUM);
        FT6236_Instance.Touch_Count = (finger_num == 0xFF) ? 0 : (finger_num & 0x0F);

        if (FT6236_Instance.Touch_Count > 0)
        {
            IIC_Read_Multi_Byte(&FT6236_dev, CST816_ADDR, CST816_REG_XH, 4, data);
            FT6236_Instance.Touch_Event = FT6236_TOUCH_EVENT_CONTACT;
            FT6236_Instance.X_Pos = ((data[0] & 0x0F) << 8) | data[1];
            FT6236_Instance.Y_Pos = ((data[2] & 0x0F) << 8) | data[3];
        }
        return;
    }

    uint8_t touch_status = FT6236_IIC_ReadREG(FT6236_REG_TD_STATUS);
    FT6236_Instance.Touch_Count = touch_status & 0x0F;

    if (FT6236_Instance.Touch_Count > 0)
    {
        IIC_Read_Multi_Byte(&FT6236_dev, FT6236_ADDR, FT6236_REG_TOUCH1_XH, 4, data);
        FT6236_Instance.Touch_Event = (data[0] >> 6) & 0x03;
        FT6236_Instance.X_Pos = ((data[0] & 0x0F) << 8) | data[1];
        FT6236_Instance.Y_Pos = ((data[2] & 0x0F) << 8) | data[3];
    }

#if REVERSE_X
    FT6236_Instance.X_Pos = 239 - FT6236_Instance.X_Pos;
#endif

#if REVERSE_Y
    FT6236_Instance.Y_Pos = 239 - FT6236_Instance.Y_Pos;
#endif

    FT6236_Instance.X_Pos += TOUCH_OFFSET_X;
    FT6236_Instance.Y_Pos += TOUCH_OFFSET_Y;
}

uint8_t FT6236_Get_Touch_Count(void)
{
    if (s_touch_ic == TOUCH_IC_CST816)
    {
        uint8_t finger_num = touch_read_reg(CST816_ADDR, CST816_REG_FINGER_NUM);
        return (finger_num == 0xFF) ? 0 : (finger_num & 0x0F);
    }

    uint8_t status = FT6236_IIC_ReadREG(FT6236_REG_TD_STATUS);
    return status & 0x0F;
}

uint8_t FT6236_Get_ChipID(void)
{
    if (s_touch_ic == TOUCH_IC_CST816)
    {
        return touch_read_reg(CST816_ADDR, CST816_REG_CHIP_ID);
    }

    return FT6236_IIC_ReadREG(FT6236_REG_CHIP_ID);
}

uint16_t FT6236_Get_LibVersion(void)
{
    if (s_touch_ic == TOUCH_IC_CST816)
    {
        return 0;
    }

    uint8_t ver_h = FT6236_IIC_ReadREG(FT6236_REG_LIB_VERSION_H);
    uint8_t ver_l = FT6236_IIC_ReadREG(FT6236_REG_LIB_VERSION_L);
    return (uint16_t)((ver_h << 8) | ver_l);
}

void FT6236_Set_Threshold(uint8_t threshold)
{
    if (s_touch_ic != TOUCH_IC_FT6236)
    {
        return;
    }

    FT6236_IIC_WriteREG(FT6236_REG_THRESHOLD, threshold);
}

void FT6236_Set_PowerMode(FT6236_PowerMode_TypeDef mode)
{
    if (s_touch_ic != TOUCH_IC_FT6236)
    {
        return;
    }

    FT6236_IIC_WriteREG(FT6236_REG_POWER_MODE, mode);
}

void FT6236_Set_InterruptMode(FT6236_IntMode_TypeDef mode)
{
    if (s_touch_ic != TOUCH_IC_FT6236)
    {
        return;
    }

    FT6236_IIC_WriteREG(FT6236_REG_G_MODE, mode);
}

void FT6236_Set_ActiveRate(uint8_t rate)
{
    if (s_touch_ic != TOUCH_IC_FT6236)
    {
        return;
    }

    if (rate > 14)
    {
        rate = 14;
    }
    FT6236_IIC_WriteREG(FT6236_REG_PERIODACTIVE, rate);
}

void FT6236_Set_MonitorRate(uint8_t rate)
{
    if (s_touch_ic != TOUCH_IC_FT6236)
    {
        return;
    }

    if (rate < 10)
    {
        rate = 10;
    }
    if (rate > 100)
    {
        rate = 100;
    }
    FT6236_IIC_WriteREG(FT6236_REG_PERIODMONITOR, rate);
}

void FT6236_Sleep(void)
{
    if (s_touch_ic == TOUCH_IC_CST816)
    {
        touch_write_reg(CST816_ADDR, 0xE5, 0x03);
        return;
    }

    FT6236_Set_PowerMode(FT6236_POWER_MODE_HIBERNATE);
}

void FT6236_Wakeup(void)
{
    FT6236_RESET();
}
