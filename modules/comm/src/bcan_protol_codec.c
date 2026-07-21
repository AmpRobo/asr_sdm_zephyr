/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * BCAN PROTOL codec: pure encode/decode functions.
 *
 * No UART, kernel, or logging dependencies -- pure data transformation.
 * Suitable for unit testing without hardware.
 */

#include <asr/bcan_protol_internal.h>

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>

/* -------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */

#define BCAN_PROTOL_DLC_MASK  0x0FU
#define BCAN_PROTOL_FLAG_EXT  0x80U
#define BCAN_PROTOL_FLAG_RTR  0x40U
#define BCAN_STD_ID_MAX       0x7FFU

/* -------------------------------------------------------------------------
 * Encode
 * ------------------------------------------------------------------------- */

int asr_bcan_protol_encode(
	const struct asr_can_frame *frame,
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN])
{
	if ((frame == NULL) || (raw == NULL)) {
		return -EINVAL;
	}

	if (frame->dlc > ASR_CLASSIC_CAN_DATA_LEN) {
		return -EMSGSIZE;
	}

	if (frame->can_id > BCAN_STD_ID_MAX) {
		return -ERANGE;
	}

	raw[0] = frame->dlc & BCAN_PROTOL_DLC_MASK;
	sys_put_be32(frame->can_id, &raw[1]);
	memset(&raw[5], 0, ASR_CLASSIC_CAN_DATA_LEN);
	memcpy(&raw[5], frame->data, frame->dlc);
	return 0;
}

/* -------------------------------------------------------------------------
 * Decode
 * ------------------------------------------------------------------------- */

int asr_bcan_protol_decode(
	const uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN],
	struct asr_can_frame *frame)
{
	if ((raw == NULL) || (frame == NULL)) {
		return -EINVAL;
	}

	/* Check for unsupported frame types. */
	if ((raw[0] & BCAN_PROTOL_FLAG_EXT) != 0U) {
		return -ENOTSUP;
	}

	if ((raw[0] & BCAN_PROTOL_FLAG_RTR) != 0U) {
		return -ENOTSUP;
	}

	uint8_t dlc = raw[0] & BCAN_PROTOL_DLC_MASK;

	if (dlc > ASR_CLASSIC_CAN_DATA_LEN) {
		return -EMSGSIZE;
	}

	uint32_t can_id = sys_get_be32(&raw[1]);

	if (can_id > BCAN_STD_ID_MAX) {
		return -ERANGE;
	}

	frame->dlc = dlc;
	frame->can_id = can_id;
	memset(frame->data, 0, sizeof(frame->data));
	memcpy(frame->data, &raw[5], dlc);
	return 0;
}