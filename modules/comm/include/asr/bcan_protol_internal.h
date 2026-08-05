/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal BCAN PROTOL codec API.
 *
 * Pure encode/decode functions for the 13-byte BCAN-S01 PROTOL frame format.
 * No UART, kernel, or logging dependencies -- suitable for unit testing.
 *
 * Wire frame (13 bytes):
 *   Byte 0:     frame_info (bits 7=EXT, 6=RTR, 3-0=DLC)
 *   Byte 1..4:  CAN ID (big-endian)
 *   Byte 5..12: CAN DATA (8 bytes, zero-padded when DLC < 8)
 */

#ifndef ASR_BCAN_PROTOL_INTERNAL_H_
#define ASR_BCAN_PROTOL_INTERNAL_H_

#include <stdint.h>

#include <asr/bcan_protol.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Encode a CAN frame into a 13-byte PROTOL raw buffer.
 *
 * @param frame  CAN frame to encode (must not be NULL).
 * @param raw    Output buffer of exactly ASR_BCAN_PROTOL_FRAME_LEN bytes.
 * @return 0 on success, -EINVAL if frame or raw is NULL,
 *         -EMSGSIZE if DLC > 8, -ERANGE if CAN ID > 0x7FF.
 */
int asr_bcan_protol_encode(
	const struct asr_can_frame *frame,
	uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN]);

/**
 * Decode a 13-byte PROTOL raw buffer into a CAN frame.
 *
 * @param raw    Input buffer of exactly ASR_BCAN_PROTOL_FRAME_LEN bytes.
 * @param frame  Output CAN frame (must not be NULL).
 * @return 0 on success, -EINVAL if raw or frame is NULL,
 *         -ENOTSUP if EXT or RTR flag is set,
 *         -EMSGSIZE if DLC > 8, -ERANGE if CAN ID > 0x7FF.
 */
int asr_bcan_protol_decode(
	const uint8_t raw[ASR_BCAN_PROTOL_FRAME_LEN],
	struct asr_can_frame *frame);

#ifdef __cplusplus
}
#endif

#endif /* ASR_BCAN_PROTOL_INTERNAL_H_ */