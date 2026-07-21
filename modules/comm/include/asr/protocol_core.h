/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Transport-independent protocol message processing core.
 *
 * Processes fixed 8-byte ASR SDM protocol messages without any dependency on
 * UART, CAN, or other transport layers.  The result object (has_response +
 * response[8]) tells the caller whether to send a reply and what payload.
 */

#ifndef ASR_PROTOCOL_CORE_H_
#define ASR_PROTOCOL_CORE_H_

#include <stdbool.h>
#include <stdint.h>

#include <asr/comm_thread.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/* protocol_core always processes fixed 8-byte messages. */
BUILD_ASSERT(ASR_COMM_MSG_SIZE == 8,
	     "protocol_core requires ASR_COMM_MSG_SIZE == 8");

/**
 * Result of processing one protocol request.
 *
 * @param has_response  true if a reply should be sent
 * @param response      reply payload (only valid when has_response is true)
 */
struct asr_protocol_result {
	bool has_response;
	uint8_t response[ASR_COMM_MSG_SIZE];
};

/**
 * Initialise the protocol core state (clears unit_status).
 *
 * @return 0 on success, negative errno on failure.
 */
int asr_protocol_core_init(void);

/**
 * Register hardware-action callbacks invoked by the protocol handler.
 *
 * @param callbacks  Pointer to a callback structure (may be NULL to clear).
 */
void asr_protocol_core_register_callbacks(
	const struct asr_comm_callbacks *callbacks);

/**
 * Get a read-only snapshot of the current unit status.
 *
 * @return  Pointer to the internal unit status structure (never NULL).
 */
const asr_unit_status_t *asr_protocol_core_get_status(void);

/**
 * Process one 8-byte protocol request.
 *
 * The result object is zero-initialised on entry, so has_response defaults to
 * false.  Only commands that produce a reply (e.g., IMU READ) set
 * result->has_response = true and fill result->response.
 *
 * @param request  Incoming 8-byte protocol message.
 * @param result   Output result object (zeroed before processing).
 * @return 0 on success, -EINVAL for bad format, -ENOTSUP for unsupported
 *         command/param, -ENOSYS if a required callback is not registered,
 *         or a negative errno from the callback itself.
 */
int asr_protocol_core_process(
	const uint8_t request[ASR_COMM_MSG_SIZE],
	struct asr_protocol_result *result);

#ifdef __cplusplus
}
#endif

#endif /* ASR_PROTOCOL_CORE_H_ */