# P040B019-MIPI-CTP (3.97寸 BOE, ST7701P, 480x800) MIPI屏显示实验
# 平台: 正点原子 ESP32-P4 开发板 + P024BM057-IPS-CTP MIPI转接板(P040B019 引脚兼容)
cmake_minimum_required(VERSION 3.16)
add_compile_options(-fdiagnostics-color=always)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(P040B019_IPS_MIPI)
