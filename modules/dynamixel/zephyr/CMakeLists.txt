# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)
zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/src)

zephyr_library_sources_ifdef(CONFIG_ASR_DYNAMIXEL
	${ZEPHYR_CURRENT_MODULE_DIR}/src/dynamixel_packet.c
	${ZEPHYR_CURRENT_MODULE_DIR}/src/dynamixel_bus.c
	${ZEPHYR_CURRENT_MODULE_DIR}/src/dynamixel.c
)

zephyr_library_sources_ifdef(CONFIG_ASR_DYNAMIXEL_THREAD
	${ZEPHYR_CURRENT_MODULE_DIR}/src/dynamixel_thread.c
)
