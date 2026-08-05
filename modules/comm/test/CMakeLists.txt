# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(asr_comm_test)

# Add the comm module (parent of test/) so protocol_core is available.
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_sources(app PRIVATE
	src/main.c
	src/bcan_protol_test.c
	src/can_protocol_test.c
	src/bcan_protol_stubs.c
	${CMAKE_CURRENT_SOURCE_DIR}/../src/bcan_protol_codec.c
	${CMAKE_CURRENT_SOURCE_DIR}/../src/protocol_core.c
	${CMAKE_CURRENT_SOURCE_DIR}/../src/can_protocol_transport.c
)

# Add the comm module include path for internal headers.
target_include_directories(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Define CAN ID config symbols for the transport layer test
# (these are normally defined by ASR_TRANSPORT_BCAN_PROTOL Kconfig).
target_compile_definitions(app PRIVATE
	CONFIG_ASR_CAN_PROTOCOL_REQUEST_CAN_ID=0x201
	CONFIG_ASR_CAN_PROTOCOL_RESPONSE_CAN_ID=0x281
)