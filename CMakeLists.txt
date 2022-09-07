# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(phyphox_satellite)

target_sources(app PRIVATE src/adc.c)
target_sources(app PRIVATE src/mprls.c)
target_sources(app PRIVATE src/sensors.h)
target_sources(app PRIVATE src/led.c)
#target_sources(app PRIVATE src/waermelehre/ds18b20.c)
#target_sources(app PRIVATE src/waermelehre/ds18b20_sensor.c)
target_sources(app PRIVATE src/shtc3.c)
target_sources(app PRIVATE src/icm42605.c)
target_sources(app PRIVATE src/ble.c)
target_sources(app PRIVATE src/mlxZephyr.c)
target_sources(app PRIVATE src/mlx90393.c)
target_sources(app PRIVATE src/bmp3.c)
target_sources(app PRIVATE src/bmpZephyr.c)
target_sources(app PRIVATE src/ds18b20.c)
target_sources(app PRIVATE src/main.c)