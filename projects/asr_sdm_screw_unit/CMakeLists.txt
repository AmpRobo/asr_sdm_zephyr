# SPDX-License-Identifier: Apache-2.0
# ICM-42688 sensor demo application

cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/led)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(icm42688_demo)

target_sources(app PRIVATE src/main.c)
