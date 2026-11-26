/**
 ****************************************************************************************************
 * @file        mipi_lcd.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2026-11-26
 * @brief       P040B019-MIPI-CTP(3.97寸 BOE, ST7701P, 480x800) MIPI屏驱动
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32-P4 开发板 + P024BM057-IPS-CTP MIPI转接板(P040B019 引脚兼容, 直接插)
 *
 ****************************************************************************************************
 */

#include "mipi_lcd.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "soc/mipi_dsi_host_reg.h"

/* DPI源(PLL_F240M)整数分频规则:
 * mipi_dsi_hal_host_dpi_calculate_divider(): div = 240/请求值, hal->dpi_clock_freq_mhz = 240/div(整数截断);
 * mipi_dsi_hal_host_dpi_set_horizontal_timing(): host侧 hsa/hbp/hline/hfp 按 lane_rate/dpi_clock/8 换算成
 * "lane字节时钟"周期, 而 bridge 侧同一组参数直接按像素计。
 * 若 240/div 不是整数MHz(请求26 -> div 9 -> 实际26.67, 截断成26), host认为的行长比 bridge 实际行长 2.6%,
 * bridge 的像素流填不下 -> FIFO 溢出丢像素 -> 花屏/横竖条纹。所以请求值必须让 240/div 严格等于实际像素时钟。 */
#define MIPI_LCD_DPI_SRC_MHZ    240
_Static_assert((MIPI_LCD_DPI_SRC_MHZ % (MIPI_LCD_DPI_SRC_MHZ / MIPI_LCD_PCLK_MHZ)) == 0,
               "MIPI_LCD_PCLK_MHZ must divide 240MHz into an integer MHz clock (e.g. 30 -> 240/8)");

static const char *mipi_lcd_tag = "mipi_lcd";

/* DSI链路就绪等待上限: IDF的HAL在命令FIFO满时是死等, 这里先看链路再发, 避免整机卡死 */
#define MIPI_LCD_LINK_TIMEOUT_MS    50

volatile uint32_t g_lcd_stage;   /* 调试用: 初始化进度 */

/**
 * @brief       判断DSI链路的时钟/数据通道是否都在LP-11(stop state)
 * @param       status: 返回PHY_STATUS原始寄存器值, 可为NULL
 * @retval      true:链路空闲, 可以发下一包
 */
static bool mipi_lcd_link_idle(uint32_t *status)
{
    uint32_t st = REG_READ(DSI_HOST_PHY_STATUS_REG);

    if (status != NULL)
    {
        *status = st;
    }

    return (st & DSI_HOST_PHY_STOPSTATECLKLANE) && (st & DSI_HOST_PHY_STOPSTATE0LANE) && (st & DSI_HOST_PHY_STOPSTATE1LANE);
}

/**
 * @brief       打印当前DSI链路状态(时钟/两条数据通道是否都在LP-11)
 * @param       where: 打印用的位置说明
 * @retval      true:链路空闲
 */
static bool mipi_lcd_report_link(const char *where)
{
    uint32_t st = 0;
    bool idle = mipi_lcd_link_idle(&st);

    ESP_LOGI(mipi_lcd_tag, "link %s @%s: PHY_STATUS=0x%08X (clk_stop=%d d0_stop=%d d1_stop=%d)",
             idle ? "idle" : "BUSY", where, (unsigned)st,
             !!(st & DSI_HOST_PHY_STOPSTATECLKLANE),
             !!(st & DSI_HOST_PHY_STOPSTATE0LANE),
             !!(st & DSI_HOST_PHY_STOPSTATE1LANE));

    return idle;
}

/**
 * @brief       等待DSI链路空闲, 超时则打印诊断信息(面板/转接板把某条差分对钳住时走这里)
 * @param       cmd: 即将发送的命令
 * @retval      ESP_OK:链路空闲; ESP_ERR_TIMEOUT:链路一直不空闲
 */
static esp_err_t mipi_lcd_wait_link_idle(uint8_t cmd)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)MIPI_LCD_LINK_TIMEOUT_MS * 1000;
    uint32_t st = 0;

    while (!mipi_lcd_link_idle(&st))
    {
        if (esp_timer_get_time() > deadline)
        {
            ESP_LOGE(mipi_lcd_tag, "link stuck before cmd 0x%02X: PHY_STATUS=0x%08X (clk_stop=%d d0_stop=%d d1_stop=%d)",
                     cmd, (unsigned)st,
                     !!(st & DSI_HOST_PHY_STOPSTATECLKLANE),
                     !!(st & DSI_HOST_PHY_STOPSTATE0LANE),
                     !!(st & DSI_HOST_PHY_STOPSTATE1LANE));
            return ESP_ERR_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ESP_OK;
}

/* 初始化命令表项: 数据长度由编译器算, delay 缺省为0(数值全部取自面板厂初始化序列) */
#define MIPI_INIT_CMD(cmd, ...)                 {cmd, (const uint8_t[]){__VA_ARGS__}, sizeof((const uint8_t[]){__VA_ARGS__}), 0}
#define MIPI_INIT_CMD_DELAY(cmd, delay, ...)    {cmd, (const uint8_t[]){__VA_ARGS__}, sizeof((const uint8_t[]){__VA_ARGS__}), delay}

/* ST7701P 初始化序列(前半部分): 选择CMD2页 -> 电源/GIP/伽马参数, 结束时回到CMD1页
 * 数据来源: 屏厂文本 ST7701P+BOE3.97_IPS MIPI 20260110.txt(43条含延时),
 *           与 rk3568-kickpi-mipi0-P040B019-IPS-CTP.dtsi 的 panel-init-sequence 逐字节一致 */
static const mipi_lcd_init_cmd_t st7701p_init_cmd_head[] = {
    MIPI_INIT_CMD(0xFF, 0x77, 0x01, 0x00, 0x00, 0x13),
    MIPI_INIT_CMD(0xEF, 0x08),
    MIPI_INIT_CMD(0xFF, 0x77, 0x01, 0x00, 0x00, 0x10),
    MIPI_INIT_CMD(0xC0, 0x63, 0x00),
    MIPI_INIT_CMD(0xC1, 0x09, 0x02),
    MIPI_INIT_CMD(0xC2, 0x21, 0x02),
    MIPI_INIT_CMD(0xCC, 0x18),
    MIPI_INIT_CMD(0xB0, 0x40, 0x0E, 0x51, 0x0F, 0x11, 0x07, 0x00, 0x09, 0x06, 0x1E, 0x04, 0x12, 0x11, 0x64, 0x29, 0xDF),
    MIPI_INIT_CMD(0xB1, 0x40, 0x07, 0x4C, 0x0A, 0x0E, 0x04, 0x00, 0x08, 0x09, 0x1D, 0x01, 0x0E, 0x0C, 0x6A, 0x34, 0xDF),
    MIPI_INIT_CMD(0xFF, 0x77, 0x01, 0x00, 0x00, 0x11),
    MIPI_INIT_CMD(0xB0, 0x30),
    MIPI_INIT_CMD(0xB1, 0x40),
    MIPI_INIT_CMD(0xB2, 0x80),
    MIPI_INIT_CMD(0xB3, 0x80),
    MIPI_INIT_CMD(0xB5, 0x4F),
    MIPI_INIT_CMD(0xB7, 0x85),
    MIPI_INIT_CMD(0xB8, 0x23),
    MIPI_INIT_CMD(0xB9, 0x22, 0x13),
    MIPI_INIT_CMD(0xBB, 0x03),
    MIPI_INIT_CMD(0xBC, 0x10),
    MIPI_INIT_CMD(0xC0, 0x89),
    MIPI_INIT_CMD(0xC1, 0x78),
    MIPI_INIT_CMD(0xC2, 0x78),
    MIPI_INIT_CMD(0xD0, 0x88),
    MIPI_INIT_CMD(0xE0, 0x00, 0x00, 0x02),
    MIPI_INIT_CMD(0xE1, 0x04, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10),
    MIPI_INIT_CMD(0xE2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    MIPI_INIT_CMD(0xE3, 0x00, 0x00, 0x33, 0x00),
    MIPI_INIT_CMD(0xE4, 0x22, 0x00),
    MIPI_INIT_CMD(0xE5, 0x03, 0x34, 0xAF, 0xB3, 0x05, 0x34, 0xAF, 0xB3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    MIPI_INIT_CMD(0xE6, 0x00, 0x00, 0x33, 0x00),
    MIPI_INIT_CMD(0xE7, 0x22, 0x00),
    MIPI_INIT_CMD(0xE8, 0x04, 0x34, 0xAF, 0xB3, 0x06, 0x34, 0xAF, 0xB3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
    MIPI_INIT_CMD(0xEB, 0x02, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00),
    MIPI_INIT_CMD(0xEC, 0x00, 0x00),
    MIPI_INIT_CMD(0xED, 0xFA, 0x45, 0x0B, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xB0, 0x54, 0xAF),
    MIPI_INIT_CMD(0xEF, 0x08, 0x08, 0x08, 0x45, 0x3F, 0x54),
    MIPI_INIT_CMD(0xFF, 0x77, 0x01, 0x00, 0x00, 0x13),
    MIPI_INIT_CMD(0xE6, 0x16, 0x7C),
    MIPI_INIT_CMD(0xE8, 0x00, 0x0E),
    MIPI_INIT_CMD(0xFF, 0x77, 0x01, 0x00, 0x00, 0x00),
};

/* ST7701P 初始化序列(后半部分): 退出睡眠后必须补发的E8配置, 再TEON/开显示 */
static const mipi_lcd_init_cmd_t st7701p_init_cmd_tail[] = {
    MIPI_INIT_CMD(0xFF, 0x77, 0x01, 0x00, 0x00, 0x13),
    MIPI_INIT_CMD_DELAY(0xE8, 255, 0x00, 0x0C),
    MIPI_INIT_CMD(0xE8, 0x00, 0x00),
    MIPI_INIT_CMD(0xFF, 0x77, 0x01, 0x00, 0x00, 0x00),
    MIPI_INIT_CMD(0x35, 0x00),              /* TEON */
    {0x29, NULL, 0, 255},                   /* DISPON */
};

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))

/**
 * @brief       发送一组合初始化命令
 * @param       io      : MIPILCD IO句柄
 * @param       cmds    : 初始化命令表
 * @param       size    : 命令条数
 * @retval      ESP_OK:发送成功
 */
static esp_err_t mipi_lcd_send_cmds(esp_lcd_panel_io_handle_t io, const mipi_lcd_init_cmd_t *cmds, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        ESP_RETURN_ON_ERROR(mipi_lcd_wait_link_idle((uint8_t)cmds[i].cmd), mipi_lcd_tag, "dsi link stalled");
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, cmds[i].cmd, cmds[i].data, cmds[i].data_bytes),
                            mipi_lcd_tag, "send command failed");
        if (cmds[i].delay_ms > 0)
        {
            vTaskDelay(pdMS_TO_TICKS(cmds[i].delay_ms));
        }
    }

    return ESP_OK;
}

/**
 * @brief       配置MIPI PHY供电(LDO_VO3 -> VDD_MIPI_DPHY)
 * @param       无
 * @retval      无
 */
void mipi_dev_bsp_enable_dsi_phy_power(void)
{
#ifdef MIPI_DSI_PHY_PWR_LDO_CHAN
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id    = MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));
    LCD_STAGE(1);
    ESP_LOGI(mipi_lcd_tag, "MIPI DSI PHY power on: LDO_VO%d = %dmV",
             MIPI_DSI_PHY_PWR_LDO_CHAN, MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV);
#endif
}

/**
 * @brief       删除LCD面板
 * @param       panel:LCD接口句柄
 * @retval      ESP_OK:删除成功
 */
static esp_err_t mipi_lcd_paneldel(esp_lcd_panel_t *panel)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);

    if (mipi_lcd->reset_gpio_num >= 0)
    {
        gpio_reset_pin(mipi_lcd->reset_gpio_num);
    }

    free(mipi_lcd);

    return ESP_OK;
}

/**
 * @brief       复位LCD面板
 * @param       panel:LCD接口句柄
 * @retval      ESP_OK:复位成功
 */
static esp_err_t mipi_lcd_panelreset(esp_lcd_panel_t *panel)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);

    if (mipi_lcd->reset_gpio_num >= 0)      /* 硬件复位 */
    {
        gpio_set_level(mipi_lcd->reset_gpio_num, mipi_lcd->reset_level);
        vTaskDelay(pdMS_TO_TICKS(100));                     /* 复位脉冲: ST7701 TRW >= 10us */
        gpio_set_level(mipi_lcd->reset_gpio_num, !mipi_lcd->reset_level);
        vTaskDelay(pdMS_TO_TICKS(200));                     /* 复位释放到可下命令: ST7701S 规格 7.5.5 TRT >= 120ms。
                                                             * 旧值100ms 低于规格, 冷启动偶发首条命令不回ACK
                                                             * (PHY_STATUS d0_stop=0, init 卡死 -> 黑屏/条纹),
                                                             * 重上电又正常。RK dtsi 的 reset-delay-ms 也是 120。*/
    }
    else                                    /* 软件复位 */
    {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(mipi_lcd->io, LCD_CMD_SWRESET, NULL, 0),
                            mipi_lcd_tag, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return ESP_OK;
}

/**
 * @brief       初始化LCD面板
 * @param       panel:LCD接口句柄
 * @retval      ESP_OK:初始化成功
 */
static esp_err_t mipi_lcd_panelinit(esp_lcd_panel_t *panel)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);
    esp_lcd_panel_io_handle_t io = mipi_lcd->io;

    /* 显示方向/像素格式: 放在厂序列之前写, 保证下面整段与面板厂初始化文本逐条一致 */
    uint8_t madctl = mipi_lcd->madctl_val;
    uint8_t colmod = mipi_lcd->colmod_val;
    ESP_RETURN_ON_ERROR(mipi_lcd_wait_link_idle(LCD_CMD_MADCTL), mipi_lcd_tag, "dsi link stalled");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, &madctl, 1), mipi_lcd_tag, "send command failed");
    ESP_RETURN_ON_ERROR(mipi_lcd_wait_link_idle(LCD_CMD_COLMOD), mipi_lcd_tag, "dsi link stalled");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, &colmod, 1), mipi_lcd_tag, "send command failed");

    /* CMD2页初始化: 电源/GIP/伽马 */
    LCD_STAGE(100);
    ESP_RETURN_ON_ERROR(mipi_lcd_send_cmds(io, st7701p_init_cmd_head, ARRAY_SIZE(st7701p_init_cmd_head)),
                        mipi_lcd_tag, "send init command failed");
    LCD_STAGE(101);
    ESP_LOGI(mipi_lcd_tag, "init: CMD2 page done");

    /* 退出睡眠 */
    ESP_RETURN_ON_ERROR(mipi_lcd_wait_link_idle(LCD_CMD_SLPOUT), mipi_lcd_tag, "dsi link stalled");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0), mipi_lcd_tag, "io tx param failed");
    LCD_STAGE(102);
    ESP_LOGI(mipi_lcd_tag, "init: sleep out");
    vTaskDelay(pdMS_TO_TICKS(255));          /* dtsi: 39 FF 02 11 00, 与RK一致(厂文本120ms, 规格>=120ms) */

    /* 睡眠退出后补发E8配置, 最后DISPON(与面板厂文本顺序一致) */
    ESP_RETURN_ON_ERROR(mipi_lcd_send_cmds(io, st7701p_init_cmd_tail, ARRAY_SIZE(st7701p_init_cmd_tail)),
                        mipi_lcd_tag, "send init command failed");
    LCD_STAGE(104);
    ESP_LOGI(mipi_lcd_tag, "init: display on (MADCTL=0x%02X, COLMOD=0x%02X)", madctl, colmod);

    return ESP_OK;
}

/**
 * @brief       反转显示
 * @param       panel             :LCD接口句柄
 * @param       invert_color_data :true反转显示;false正常显示
 * @retval      ESP_OK:设置成功
 */
static esp_err_t mipi_lcd_panelinvert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);
    esp_lcd_panel_io_handle_t io = mipi_lcd->io;
    int command = invert_color_data ? LCD_CMD_INVON : LCD_CMD_INVOFF;

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), mipi_lcd_tag, "send command failed");

    return ESP_OK;
}

/**
 * @brief       在特定轴上镜像LCD面板
 * @param       panel       :LCD接口句柄
 * @param       mirror_x    :true对x轴进行镜像,false不操作镜像
 * @param       mirror_y    :true对y轴进行镜像,false不进行镜像
 * @retval      ESP_OK:设置成功
 */
static esp_err_t mipi_lcd_panelmirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);
    esp_lcd_panel_io_handle_t io = mipi_lcd->io;

    if (mirror_x)
    {
        mipi_lcd->madctl_val |= LCD_CMD_MX_BIT;     /* 扫描方向水平翻转(镜像x) */
    }
    else
    {
        mipi_lcd->madctl_val &= ~LCD_CMD_MX_BIT;    /* 扫描方向水平不翻转 */
    }

    if (mirror_y)
    {
        mipi_lcd->madctl_val |= LCD_CMD_MY_BIT;     /* 扫描方向垂直翻转(镜像y) */
    }
    else
    {
        mipi_lcd->madctl_val &= ~LCD_CMD_MY_BIT;    /* 扫描方向垂直不翻转 */
    }

    uint8_t madctl = mipi_lcd->madctl_val;
    ESP_RETURN_ON_ERROR(mipi_lcd_wait_link_idle(LCD_CMD_MADCTL), mipi_lcd_tag, "dsi link stalled");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, &madctl, 1), mipi_lcd_tag, "send command failed");

    return ESP_OK;
}

/**
 * @brief       交换x和y轴
 * @param       panel     :LCD接口句柄
 * @param       swap_axes :是否对x和y轴进行方向互换
 * @retval      ESP_ERR_NOT_SUPPORTED:MIPI屏不支持
 */
static esp_err_t mipi_lcd_panelswap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    ESP_LOGW(mipi_lcd_tag, "Swap XY is not supported in MIPI driver. Please use SW rotation.");
    return ESP_ERR_NOT_SUPPORTED;
}

/**
 * @brief       设置x轴和y轴的偏移
 * @param       panel :LCD接口句柄
 * @param       x_gap :X方向偏移
 * @param       y_gap :Y方向偏移
 * @retval      ESP_OK:设置成功
 */
static esp_err_t mipi_lcd_panelset_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);
    mipi_lcd->x_gap = x_gap;
    mipi_lcd->y_gap = y_gap;
    return ESP_OK;
}

/**
 * @brief       打开或者关闭显示
 * @param       panel :LCD接口句柄
 * @param       on_off:true打开显示,false关闭显示
 * @retval      ESP_OK:设置成功
 */
static esp_err_t mipi_lcd_paneldisp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);
    esp_lcd_panel_io_handle_t io = mipi_lcd->io;
    int command = on_off ? LCD_CMD_DISPON : LCD_CMD_DISPOFF;

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), mipi_lcd_tag, "send command failed");

    return ESP_OK;
}

/**
 * @brief       进入或退出睡眠模式
 * @param       panel :LCD接口句柄
 * @param       sleep :true进入睡眠模式,false退出睡眠模式
 * @retval      ESP_OK:设置成功
 */
static esp_err_t mipi_lcd_panelsleep(esp_lcd_panel_t *panel, bool sleep)
{
    mipi_panel_t *mipi_lcd = __containerof(panel, mipi_panel_t, base);
    esp_lcd_panel_io_handle_t io = mipi_lcd->io;
    int command = sleep ? LCD_CMD_SLPIN : LCD_CMD_SLPOUT;

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), mipi_lcd_tag, "io tx param failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

/**
 * @brief       新建mipilcd句柄，并配置函数
 * @param       io              :MIPI IO句柄
 * @param       panel_dev_config:MIPI 设备配置结构体
 * @param       ret_panel       :MIPI屏句柄
 * @retval      esp_err_t 返回值
 */
static esp_err_t mipi_lcd_new_panel(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, mipi_lcd_tag, "invalid argument");

    mipi_panel_t *mipi_lcd = (mipi_panel_t *)calloc(1, sizeof(mipi_panel_t));
    ESP_RETURN_ON_FALSE(mipi_lcd, ESP_ERR_NO_MEM, mipi_lcd_tag, "no mem for mipi_lcd panel");

    if (panel_dev_config->reset_gpio_num >= 0)      /* 配置LCD复位引脚 */
    {
        gpio_config_t io_conf = {
            .mode         = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, mipi_lcd_tag, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->rgb_ele_order)        /* 颜色顺序RGB/BGR */
    {
        case LCD_RGB_ELEMENT_ORDER_RGB:
            mipi_lcd->madctl_val = 0;
            break;
        case LCD_RGB_ELEMENT_ORDER_BGR:
            mipi_lcd->madctl_val = LCD_CMD_BGR_BIT;
            break;
        default:
            ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, mipi_lcd_tag, "unsupported rgb element order");
            break;
    }

    switch (panel_dev_config->bits_per_pixel)       /* 像素格式 */
    {
        case 16:    /* RGB565 */
            mipi_lcd->colmod_val = 0x55;
            break;
        case 18:    /* RGB666 */
            mipi_lcd->colmod_val = 0x66;
            break;
        case 24:    /* RGB888 */
            mipi_lcd->colmod_val = 0x77;
            break;
        default:
            ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, mipi_lcd_tag, "unsupported pixel width");
            break;
    }

    mipi_lcd->io                = io;
    mipi_lcd->reset_gpio_num    = panel_dev_config->reset_gpio_num;         /* LCD的复位引脚 */
    mipi_lcd->reset_level       = panel_dev_config->flags.reset_active_high;/* 复位引脚的有效电平 */
    mipi_lcd->base.reset        = mipi_lcd_panelreset;                      /* 复位LCD面板 */
    mipi_lcd->base.init         = mipi_lcd_panelinit;                       /* 初始化LCD面板 */
    mipi_lcd->base.del          = mipi_lcd_paneldel;                        /* 删除LCD面板 */
    mipi_lcd->base.mirror       = mipi_lcd_panelmirror;                     /* 在特定轴上镜像LCD面板 */
    mipi_lcd->base.swap_xy      = mipi_lcd_panelswap_xy;                    /* 交换x轴和y轴 */
    mipi_lcd->base.set_gap      = mipi_lcd_panelset_gap;                    /* 设置x轴和y轴的偏移 */
    mipi_lcd->base.invert_color = mipi_lcd_panelinvert_color;               /* 反转显示 */
    mipi_lcd->base.disp_on_off  = mipi_lcd_paneldisp_on_off;                /* 打开或关闭显示 */
    mipi_lcd->base.disp_sleep   = mipi_lcd_panelsleep;                      /* 进入或退出睡眠模式 */

    *ret_panel = &mipi_lcd->base;

    return ESP_OK;

err:
    if (mipi_lcd)
    {
        mipi_lcd_paneldel(&mipi_lcd->base);
    }

    return ret;
}

/**
 * @brief       mipi_lcd初始化
 * @param       无
 * @retval      LCD控制句柄
 */
esp_lcd_panel_handle_t mipi_lcd_init(void)
{
    mipi_dev_bsp_enable_dsi_phy_power();                    /* 配置DSI接口电压 */

    /* 创建DSI总线 */
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id             = 0,                            /* 总线ID */
        .num_data_lanes     = MIPI_DSI_LANE_NUM,            /* DSI数据通道数 */
        .phy_clk_src        = MIPI_DSI_PHY_CLK_SRC_DEFAULT, /* DPHY时钟源 */
        .lane_bit_rate_mbps = MIPI_DSI_LANE_BITRATE_MBPS,   /* 数据通道比特率(Mbps) */
    };
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));
    LCD_STAGE(2);
    ESP_LOGI(mipi_lcd_tag, "DSI bus ready: %u lane @ %u Mbps", (unsigned)MIPI_DSI_LANE_NUM, (unsigned)MIPI_DSI_LANE_BITRATE_MBPS);
    mipi_lcd_report_link("bus init");

    /* 配置DSI总线的DBI接口(用于发送初始化命令) */
    esp_lcd_panel_io_handle_t mipi_dbi_io;
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,                               /* 虚拟通道(只有一个LCD连接,设置0即可) */
        .lcd_cmd_bits    = 8,                               /* 命令位宽度 */
        .lcd_param_bits  = 8,                               /* 参数位宽度 */
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &mipi_dbi_io));
    LCD_STAGE(3);

    ESP_LOGI(mipi_lcd_tag, "cmd_mode_cfg: 0x%08X", (unsigned)REG_READ(DSI_HOST_CMD_MODE_CFG_REG));
    ESP_LOGI(mipi_lcd_tag, "DBI io ready");

    /* 创建LCD控制器驱动 */
    esp_lcd_panel_handle_t mipi_lcd_ctrl_panel;
    esp_lcd_panel_dev_config_t lcd_dev_config = {
        .bits_per_pixel = 24,                               /* RGB888 */
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_BGR,        /* 本屏实际是 BGR 顺序: 纯红(FF0000)显示成蓝、纯蓝显示成红,
                                                             * 绿色正常; P4的DSI只能按标准RGB888发(没有字节交换选项),
                                                             * 所以用 MADCTL(0x36)=0x08 让面板按BGR解释 */
        .reset_gpio_num = lcddev.ctrl.lcd_rst,              /* MIPILCD屏的复位引脚 */
    };
    ESP_ERROR_CHECK(mipi_lcd_new_panel(mipi_dbi_io, &lcd_dev_config, &mipi_lcd_ctrl_panel));
    LCD_STAGE(4);

    ESP_ERROR_CHECK(esp_lcd_panel_reset(mipi_lcd_ctrl_panel));              /* 复位MIPILCD屏 */
    LCD_STAGE(5);
    ESP_LOGI(mipi_lcd_tag, "panel reset done");

    /* 命令通道不通(某条差分对被钳住)时 IDF 的 HAL 会死等, 这里先做链路自检并带超时 */
    mipi_lcd_report_link("after reset");

    esp_err_t err = esp_lcd_panel_init(mipi_lcd_ctrl_panel);                /* 初始化MIPILCD屏 */
    if (err != ESP_OK)
    {
        ESP_LOGE(mipi_lcd_tag, "panel init failed: %s, skip DPI setup", esp_err_to_name(err));
        return NULL;
    }
    LCD_STAGE(6);
    LCD_STAGE(7);
    ESP_LOGI(mipi_lcd_tag, "panel command init done");

    lcddev.id     = 0x7701;                                                 /* ST7701P */
    lcddev.width  = MIPI_LCD_WIDTH;
    lcddev.height = MIPI_LCD_HEIGHT;
    lcddev.lcd_dbi_io = mipi_dbi_io;

    ESP_LOGI(mipi_lcd_tag, "panel: ST7701P %ux%u, timing hsw=%u hbp=%u hfp=%u vsw=%u vbp=%u vfp=%u pclk=%uMHz",
             MIPI_LCD_WIDTH, MIPI_LCD_HEIGHT,
             MIPI_LCD_HSW, MIPI_LCD_HBP, MIPI_LCD_HFP,
             MIPI_LCD_VSW, MIPI_LCD_VBP, MIPI_LCD_VFP, MIPI_LCD_PCLK_MHZ);

    /* 创建DPI面板: 视频模式下按上面的RGB时序输出帧 */
    esp_lcd_panel_handle_t mipi_dpi_panel;
    esp_lcd_dpi_panel_config_t dpi_config = {
        .virtual_channel    = 0,                                /* 虚拟通道 */
        .dpi_clk_src        = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     /* 时钟源 */
        .dpi_clock_freq_mhz = MIPI_LCD_PCLK_MHZ,                /* 像素时钟频率 */
        .pixel_format       = LCD_COLOR_PIXEL_FORMAT_RGB888,    /* 颜色格式 */
        .num_fbs            = 2,                                /* 帧缓冲区数量 */
        .video_timing       = {
            .h_size            = MIPI_LCD_WIDTH,
            .v_size            = MIPI_LCD_HEIGHT,
            .hsync_back_porch  = MIPI_LCD_HBP,
            .hsync_pulse_width = MIPI_LCD_HSW,
            .hsync_front_porch = MIPI_LCD_HFP,
            .vsync_back_porch  = MIPI_LCD_VBP,
            .vsync_pulse_width = MIPI_LCD_VSW,
            .vsync_front_porch = MIPI_LCD_VFP,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_dpi(mipi_dsi_bus, &dpi_config, &mipi_dpi_panel));
    LCD_STAGE(8);
    ESP_LOGI(mipi_lcd_tag, "DPI panel created");
    ESP_ERROR_CHECK(esp_lcd_panel_init(mipi_dpi_panel));
    LCD_STAGE(9);
    ESP_LOGI(mipi_lcd_tag, "DPI panel started, video streaming");

    return mipi_dpi_panel;
}
