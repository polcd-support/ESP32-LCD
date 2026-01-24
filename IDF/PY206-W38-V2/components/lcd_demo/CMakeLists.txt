idf_component_register(SRCS "lcd_demo.c" "img_logo.c" "alibaba_32.c" INCLUDE_DIRS "include" REQUIRES "driver"
    PRIV_REQUIRES esp_lcd esp_lcd_touch_cst9217 esp_lv_port
)
