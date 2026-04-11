# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ASR_LED ${ZEPHYR_CURRENT_MODULE_DIR}/src/led.c)
zephyr_include_directories_ifdef(CONFIG_ASR_LED ${ZEPHYR_CURRENT_MODULE_DIR}/include)
