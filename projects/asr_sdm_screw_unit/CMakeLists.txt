# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES
	${CMAKE_CURRENT_SOURCE_DIR}/../../modules/common
	${CMAKE_CURRENT_SOURCE_DIR}/../../modules/led
	${CMAKE_CURRENT_SOURCE_DIR}/../../modules/imu
	${CMAKE_CURRENT_SOURCE_DIR}/../../modules/comm
	${CMAKE_CURRENT_SOURCE_DIR}/../../modules/drivers
	${CMAKE_CURRENT_SOURCE_DIR}/../../modules/interface
	${CMAKE_CURRENT_SOURCE_DIR}/../../modules/monitor
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unit_system)

target_sources(app PRIVATE src/main.c)
