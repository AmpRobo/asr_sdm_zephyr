# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)

zephyr_library_sources_ifdef(CONFIG_ASR_PROTOCOL_CORE
	${ZEPHYR_CURRENT_MODULE_DIR}/src/protocol_core.c
)

zephyr_library_sources_ifdef(CONFIG_ASR_TRANSPORT_UART_AA55
	${ZEPHYR_CURRENT_MODULE_DIR}/src/comm_thread.c
	${ZEPHYR_CURRENT_MODULE_DIR}/src/transport_uart_aa55.c
)

zephyr_library_sources_ifdef(CONFIG_ASR_TRANSPORT_BCAN_PROTOL
	${ZEPHYR_CURRENT_MODULE_DIR}/src/bcan_protol.c
	${ZEPHYR_CURRENT_MODULE_DIR}/src/bcan_protol_codec.c
	${ZEPHYR_CURRENT_MODULE_DIR}/src/can_protocol_transport.c
	${ZEPHYR_CURRENT_MODULE_DIR}/src/can_protocol_thread.c
)