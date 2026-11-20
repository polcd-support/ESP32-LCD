# P024BM057-IPS-CTP (2.4寸 BOE, ST7701P, 480x640) MIPI屏显示实验
# 平台: 正点原子 ESP32-P4 开发板 + P024BM057-IPS-CTP MIPI转接板
cmake_minimum_required(VERSION 3.16)
add_compile_options(-fdiagnostics-color=always)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(P024BM057_IPS_MIPI)
