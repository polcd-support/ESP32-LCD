set(src_dirs
            LED
            LCD
            TOUCH)

set(include_dirs
            LED
            LCD
            TOUCH)

set(requires
            driver
            esp_lcd
            esp_common
            esp_timer)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-error=format=-Wno-format)
