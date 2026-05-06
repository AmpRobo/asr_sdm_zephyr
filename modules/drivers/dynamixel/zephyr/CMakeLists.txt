# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/dynamixel/include)
zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/dynamixel/src)

zephyr_library_sources_ifdef(CONFIG_ASR_DYNAMIXEL
	${ZEPHYR_CURRENT_MODULE_DIR}/dynamixel/src/dynamixel_packet.c
	${ZEPHYR_CURRENT_MODULE_DIR}/dynamixel/src/dynamixel_bus.c
)
