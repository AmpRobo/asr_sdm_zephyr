/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dynamixel_packet.h"

#include <string.h>

#include <errno.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#define DXL_HEADER_0 0xFFU
#define DXL_HEADER_1 0xFFU
#define DXL_HEADER_2 0xFDU
#define DXL_RESERVED 0x00U

static uint16_t asr_dynamixel_update_crc(uint16_t crc,
					 const uint8_t *data,
					 size_t len)
{
	for (size_t i = 0; i < len; ++i) {
		crc ^= (uint16_t)data[i] << 8;

		for (uint8_t bit = 0U; bit < 8U; ++bit) {
			if ((crc & BIT(15)) != 0U) {
				crc = (uint16_t)((crc << 1) ^ 0x8005U);
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

static size_t asr_dynamixel_add_stuffing(const uint8_t *input, size_t input_len,
					 uint8_t *output, size_t output_len)
{
	size_t out_len = 0U;

	for (size_t i = 0; i < input_len; ++i) {
		if (out_len >= output_len) {
			return 0U;
		}

		output[out_len++] = input[i];

		if (i >= 2U &&
		    input[i - 2U] == DXL_HEADER_0 &&
		    input[i - 1U] == DXL_HEADER_1 &&
		    input[i] == DXL_HEADER_2) {
			if (out_len >= output_len) {
				return 0U;
			}

			output[out_len++] = DXL_HEADER_2;
		}
	}

	return out_len;
}

static size_t asr_dynamixel_remove_stuffing(const uint8_t *input, size_t input_len,
					    uint8_t *output, size_t output_len)
{
	size_t out_len = 0U;

	for (size_t i = 0; i < input_len; ++i) {
		if (i + 3U < input_len &&
		    input[i] == DXL_HEADER_0 &&
		    input[i + 1U] == DXL_HEADER_1 &&
		    input[i + 2U] == DXL_HEADER_2 &&
		    input[i + 3U] == DXL_HEADER_2) {
			if ((output_len - out_len) < 3U) {
				return 0U;
			}

			output[out_len++] = DXL_HEADER_0;
			output[out_len++] = DXL_HEADER_1;
			output[out_len++] = DXL_HEADER_2;
			i += 3U;
			continue;
		}

		if (out_len >= output_len) {
			return 0U;
		}

		output[out_len++] = input[i];
	}

	return out_len;
}

size_t asr_dynamixel_build_instruction_packet(uint8_t id, uint8_t instruction,
					      const uint8_t *params,
					      size_t param_len, uint8_t *out,
					      size_t out_size)
{
	uint8_t body[1U + ASR_DYNAMIXEL_MAX_PARAMS];
	uint8_t stuffed[ASR_DYNAMIXEL_MAX_PACKET_SIZE - 7U - 2U];
	uint16_t length;
	uint16_t crc;
	size_t stuffed_len;
	size_t total_len;

	if ((param_len > ASR_DYNAMIXEL_MAX_PARAMS) || (out_size < 10U)) {
		return 0U;
	}

	body[0] = instruction;
	if ((params != NULL) && (param_len > 0U)) {
		memcpy(&body[1], params, param_len);
	}

	stuffed_len = asr_dynamixel_add_stuffing(body, 1U + param_len, stuffed,
						 ARRAY_SIZE(stuffed));
	if (stuffed_len == 0U) {
		return 0U;
	}

	length = (uint16_t)(stuffed_len + 2U);
	total_len = 7U + stuffed_len + 2U;
	if (total_len > out_size) {
		return 0U;
	}

	out[0] = DXL_HEADER_0;
	out[1] = DXL_HEADER_1;
	out[2] = DXL_HEADER_2;
	out[3] = DXL_RESERVED;
	out[4] = id;
	sys_put_le16(length, &out[5]);
	memcpy(&out[7], stuffed, stuffed_len);

	crc = asr_dynamixel_update_crc(0U, out, 7U + stuffed_len);
	sys_put_le16(crc, &out[7U + stuffed_len]);

	return total_len;
}

int asr_dynamixel_parse_status_packet(const uint8_t *packet, size_t packet_len,
				      struct asr_dynamixel_status_packet *status)
{
	uint8_t body[1U + 1U + ASR_DYNAMIXEL_MAX_PARAMS + 4U];
	uint16_t expected_crc;
	uint16_t received_crc;
	uint16_t length;
	size_t stuffed_len;
	size_t body_len;

	if ((packet == NULL) || (status == NULL) || (packet_len < 11U)) {
		return -EINVAL;
	}

	if (packet[0] != DXL_HEADER_0 ||
	    packet[1] != DXL_HEADER_1 ||
	    packet[2] != DXL_HEADER_2 ||
	    packet[3] != DXL_RESERVED) {
		return -EPROTO;
	}

	length = sys_get_le16(&packet[5]);
	if (packet_len != (size_t)(7U + length)) {
		return -EMSGSIZE;
	}

	expected_crc = asr_dynamixel_update_crc(0U, packet, packet_len - 2U);
	received_crc = sys_get_le16(&packet[packet_len - 2U]);
	if (expected_crc != received_crc) {
		return -EBADMSG;
	}

	stuffed_len = packet_len - 9U;
	body_len = asr_dynamixel_remove_stuffing(&packet[7], stuffed_len,
						 body, sizeof(body));
	if (body_len < 2U) {
		return -EMSGSIZE;
	}

	if ((uint16_t)(body_len + 2U) != length) {
		return -EMSGSIZE;
	}

	if (body[0] != ASR_DYNAMIXEL_STATUS_INST) {
		return -EPROTO;
	}

	status->id = packet[4];
	status->alert = (body[1] & BIT(7)) != 0U;
	status->error = (body[1] & 0x7FU);
	status->param_len = body_len - 2U;
	if (status->param_len > sizeof(status->params)) {
		return -EMSGSIZE;
	}

	if (status->param_len > 0U) {
		memcpy(status->params, &body[2], status->param_len);
	}

	return 0;
}

int asr_dynamixel_map_status_error(uint8_t error)
{
	switch (error) {
	case 0x00U:
		return 0;
	case 0x01U:
		return -EIO;
	case 0x02U:
		return -ENOTSUP;
	case 0x03U:
		return -EBADMSG;
	case 0x04U:
	case 0x06U:
		return -ERANGE;
	case 0x05U:
		return -EMSGSIZE;
	case 0x07U:
		return -EACCES;
	default:
		return -EIO;
	}
}
