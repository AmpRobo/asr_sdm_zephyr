/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ASR_DYNAMIXEL_PACKET_H_
#define ASR_DYNAMIXEL_PACKET_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ASR_DYNAMIXEL_HEADER_SIZE     4U
#define ASR_DYNAMIXEL_MAX_PARAMS      32U
#define ASR_DYNAMIXEL_MAX_PACKET_SIZE 64U

#define ASR_DYNAMIXEL_INST_PING  0x01U
#define ASR_DYNAMIXEL_INST_READ  0x02U
#define ASR_DYNAMIXEL_INST_WRITE 0x03U

#define ASR_DYNAMIXEL_STATUS_INST 0x55U

struct asr_dynamixel_status_packet {
	uint8_t id;
	uint8_t error;
	bool alert;
	uint8_t params[ASR_DYNAMIXEL_MAX_PARAMS];
	size_t param_len;
};

size_t asr_dynamixel_build_instruction_packet(uint8_t id, uint8_t instruction,
					      const uint8_t *params,
					      size_t param_len, uint8_t *out,
					      size_t out_size);

int asr_dynamixel_parse_status_packet(const uint8_t *packet, size_t packet_len,
				      struct asr_dynamixel_status_packet *status);

int asr_dynamixel_map_status_error(uint8_t error);

#endif /* ASR_DYNAMIXEL_PACKET_H_ */
