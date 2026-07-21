# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)

zephyr_library_sources_ifdef(CONFIG_ASR_BAROMETER
	${ZEPHYR_CURRENT_MODULE_DIR}/src/barometer.c
)

zephyr_library_sources_ifdef(CONFIG_ASR_BAROMETER_THREAD
	${ZEPHYR_CURRENT_MODULE_DIR}/src/barometer_thread.c
)
