#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Inc/ft6236.h"

#define TOUCH_OFFSET_X 0
#define TOUCH_OFFSET_Y 0
#define REVERSE_X 0
#define REVERSE_Y 0

FT6236_Info FT6236_Instance;

iic_bus_t FT6236_dev = {
    .sda_pin = I2C0_SDA_PIN,
    .scl_pin = I2C0_SCL_PIN,
};
/*
*********************************************************************************************************
*   Function: FT6236_GPIO_Init
*   Description: Initialize GPIO pins for FT6236
*   Parameters: none
*   Return: none
*********************************************************************************************************
*/
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

    /* Initialize I2C pins */
    IICInit(&FT6236_dev);
}

/*
*********************************************************************************************************
*   Function: FT6236_Init
*   Description: Initialize FT6236 touch controller
*   Parameters: none
*   Return: none
*********************************************************************************************************
*/
void FT6236_Init(void)
{
    FT6236_GPIO_Init();
    FT6236_RESET();

    // Set default threshold
    FT6236_Set_Threshold(128);

    // Set interrupt mode to trigger
    FT6236_Set_InterruptMode(FT6236_INT_MODE_TRIGGER);

    // Set active mode report rate to 10ms
    FT6236_Set_ActiveRate(10);
}

/*
*********************************************************************************************************
*   Function: FT6236_IIC_ReadREG
*   Description: Read single register from FT6236
*   Parameters: addr - register address
*   Return: register value
*********************************************************************************************************
*/
uint8_t FT6236_IIC_ReadREG(uint8_t addr)
{
    return IIC_Read_One_Byte(&FT6236_dev, FT6236_ADDR, addr);
}

/*
*********************************************************************************************************
*   Function: FT6236_IIC_WriteREG
*   Description: Write to FT6236 register
*   Parameters: addr - register address
*               dat - data to write
*   Return: none
*********************************************************************************************************
*/
void FT6236_IIC_WriteREG(uint8_t addr, uint8_t dat)
{
    IIC_Write_One_Byte(&FT6236_dev, FT6236_ADDR, addr, dat);
}

/*
*********************************************************************************************************
*   Function: FT6236_RESET
*   Description: Reset FT6236
*   Parameters: none
*   Return: none
*********************************************************************************************************
*/
void FT6236_RESET(void)
{
    TOUCH_RST_0;
    vTaskDelay(pdMS_TO_TICKS(10));
    TOUCH_RST_1;
    vTaskDelay(pdMS_TO_TICKS(100));
}

/*
*********************************************************************************************************
*   Function: FT6236_Get_Touch_Data
*   Description: Read touch data from FT6236
*   Parameters: none
*   Return: none (data stored in FT6236_Instance structure)
*********************************************************************************************************
*/
void FT6236_Get_Touch_Data(void)
{
    uint8_t data[4];
    uint8_t touch_status = FT6236_IIC_ReadREG(FT6236_REG_TD_STATUS);

    FT6236_Instance.Touch_Count = touch_status & 0x0F;

    if (FT6236_Instance.Touch_Count > 0)
    {
        IIC_Read_Multi_Byte(&FT6236_dev, FT6236_ADDR, FT6236_REG_TOUCH1_XH, 4, data);

        FT6236_Instance.Touch_Event = (data[0] >> 6) & 0x03;
        FT6236_Instance.X_Pos = ((data[0] & 0x0F) << 8) | data[1];
        FT6236_Instance.Y_Pos = ((data[2] & 0x0F) << 8) | data[3];

#if REVERSE_X
        FT6236_Instance.X_Pos = 239 - FT6236_Instance.X_Pos;
#endif

#if REVERSE_Y
        FT6236_Instance.Y_Pos = 279 - FT6236_Instance.Y_Pos;
#endif

        FT6236_Instance.X_Pos += TOUCH_OFFSET_X;
        FT6236_Instance.Y_Pos += TOUCH_OFFSET_Y;
    }
}

/*
*********************************************************************************************************
*   Function: FT6236_Get_Touch_Count
*   Description: Get number of touch points
*   Parameters: none
*   Return: number of touch points (0-2)
*********************************************************************************************************
*/
uint8_t FT6236_Get_Touch_Count(void)
{
    uint8_t status = FT6236_IIC_ReadREG(FT6236_REG_TD_STATUS);
    return status & 0x0F;
}

/*
*********************************************************************************************************
*   Function: FT6236_Get_ChipID
*   Description: Get chip ID
*   Parameters: none
*   Return: chip ID
*********************************************************************************************************
*/
uint8_t FT6236_Get_ChipID(void)
{
    return FT6236_IIC_ReadREG(FT6236_REG_CHIP_ID);
}

/*
*********************************************************************************************************
*   Function: FT6236_Get_LibVersion
*   Description: Get library version
*   Parameters: none
*   Return: library version
*********************************************************************************************************
*/
uint16_t FT6236_Get_LibVersion(void)
{
    uint8_t ver_h = FT6236_IIC_ReadREG(FT6236_REG_LIB_VERSION_H);
    uint8_t ver_l = FT6236_IIC_ReadREG(FT6236_REG_LIB_VERSION_L);
    return (ver_h << 8) | ver_l;
}

/*
*********************************************************************************************************
*   Function: FT6236_Set_Threshold
*   Description: Set touch threshold
*   Parameters: threshold - threshold value (0-255)
*   Return: none
*********************************************************************************************************
*/
void FT6236_Set_Threshold(uint8_t threshold)
{
    FT6236_IIC_WriteREG(FT6236_REG_THRESHOLD, threshold);
}

/*
*********************************************************************************************************
*   Function: FT6236_Set_PowerMode
*   Description: Set power mode
*   Parameters: mode - power mode (active, monitor, hibernate)
*   Return: none
*********************************************************************************************************
*/
void FT6236_Set_PowerMode(FT6236_PowerMode_TypeDef mode)
{
    FT6236_IIC_WriteREG(FT6236_REG_POWER_MODE, mode);
}

/*
*********************************************************************************************************
*   Function: FT6236_Set_InterruptMode
*   Description: Set interrupt mode
*   Parameters: mode - interrupt mode (polling, trigger, continuous)
*   Return: none
*********************************************************************************************************
*/
void FT6236_Set_InterruptMode(FT6236_IntMode_TypeDef mode)
{
    FT6236_IIC_WriteREG(FT6236_REG_G_MODE, mode);
}

/*
*********************************************************************************************************
*   Function: FT6236_Set_ActiveRate
*   Description: Set active mode report rate
*   Parameters: rate - report rate in ms (1-14)
*   Return: none
*********************************************************************************************************
*/
void FT6236_Set_ActiveRate(uint8_t rate)
{
    if (rate > 14)
        rate = 14;
    FT6236_IIC_WriteREG(FT6236_REG_PERIODACTIVE, rate);
}

/*
*********************************************************************************************************
*   Function: FT6236_Set_MonitorRate
*   Description: Set monitor mode report rate
*   Parameters: rate - report rate in ms (10-100)
*   Return: none
*********************************************************************************************************
*/
void FT6236_Set_MonitorRate(uint8_t rate)
{
    if (rate < 10)
        rate = 10;
    if (rate > 100)
        rate = 100;
    FT6236_IIC_WriteREG(FT6236_REG_PERIODMONITOR, rate);
}

/*
*********************************************************************************************************
*   Function: FT6236_Sleep
*   Description: Put FT6236 into sleep mode
*   Parameters: none
*   Return: none
*********************************************************************************************************
*/
void FT6236_Sleep(void)
{
    FT6236_Set_PowerMode(FT6236_POWER_MODE_HIBERNATE);
}

/*
*********************************************************************************************************
*   Function: FT6236_Wakeup
*   Description: Wake up FT6236 from sleep
*   Parameters: none
*   Return: none
*********************************************************************************************************
*/
void FT6236_Wakeup(void)
{
    FT6236_RESET();
}