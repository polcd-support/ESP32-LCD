#include "esp_bsp.h"
#include "display.h"

//#include "lv_demos.h"
#include "lcd_demo.h"

void app_main(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = EXAMPLE_LCD_QSPI_H_RES * EXAMPLE_LCD_QSPI_V_RES,
        .rotate = LV_DISP_ROT_NONE,
    };

    bsp_display_start_with_config(&cfg);

    bsp_display_lock(0);

//    lv_demo_widgets();

	start_lvgl_demo();

    bsp_display_unlock();
}
