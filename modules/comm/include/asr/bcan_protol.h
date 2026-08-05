/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * BCAN-S01 PROTOL transport layer for the ASR SDM protocol.
 *
 * Encodes and decodes the 13-byte BCAN-S01 PROTOL frames on UART0 into
 * generic struct asr_can_frame.  This layer is transport-only: it contains
 * no business logic, Dynamixel calls, or protocol command dispatch.
 *
 * Wire frame (13 bytes):
 *   Byte 0:     frame_info (bits 7=EXT, 6=RTR, 3-0=DLC)
 *   Byte 1..4:  CAN ID (big-endian)
 *   Byte 5..12: CAN DATA (8 bytes, zero-padded when DLC < 8)
 */

#ifndef ASR_BCAN_PROTOL_H_
#define ASR_BCAN_PROTOL_H_

#include <stdint.h>

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ASR_BCAN_PROTOL_FRAME_LEN  13U
#define ASR_CLASSIC_CAN_DATA_LEN   8U

/** Generic CAN frame (transport-independent, no business fields). */
struct asr_can_frame {
	uint32_t can_id;
	uint8_t  dlc;
	uint8_t  data[ASR_CLASSIC_CAN_DATA_LEN];
};

/** BCAN PROTOL transport statistics. */
struct asr_bcan_protol_stats {
	unsigned long frames_received;
	unsigned long frames_sent;
	unsigned long decode_errors;
	unsigned long uart_errors;
};

/**
 * Initialise the BCAN PROTOL transport (UART0, poll-based).
 *
 * @return 0 on success, negative errno on failure.
 */
int asr_bcan_protol_init(void);

/**
 * Block until a complete CAN frame is decoded from PROTOL.
 *
 * @param frame   Output decoded CAN frame.
 * @param timeout Maximum time to wait (K_FOREVER to block indefinitely).
 * @return 0 on success, -EAGAIN on timeout, negative errno on other errors.
 */
int asr_bcan_protol_receive(struct asr_can_frame *frame,
			    k_timeout_t timeout);

/**
 * Encode and send a CAN frame as a 13-byte PROTOL frame.
 *
 * @param frame CAN frame to send (dlc must be <= 8).
 * @return 0 on success, negative errno on failure.
 */
int asr_bcan_protol_send(const struct asr_can_frame *frame);

/**
 * Get a read-only snapshot of the transport statistics.
 */
const struct asr_bcan_protol_stats *asr_bcan_protol_get_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_BCAN_PROTOL_H_ */