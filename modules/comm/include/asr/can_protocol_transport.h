/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CAN protocol transport adapter.
 *
 * Receives CAN frames from the BCAN PROTOL layer, filters by request CAN ID,
 * validates DLC, and dispatches the 8-byte payload through the transport-
 * independent protocol core.  When the core indicates a response is needed,
 * the adapter sets the response CAN ID and sends it back via PROTOL.
 *
 * This layer contains no Dynamixel, LED, or other business logic.
 */

#ifndef ASR_CAN_PROTOCOL_TRANSPORT_H_
#define ASR_CAN_PROTOCOL_TRANSPORT_H_

#include <asr/bcan_protol.h>
#include <stdbool.h>

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Process one CAN frame through the protocol core.
 *
 * Pure business mapping function: checks CAN ID, validates DLC, calls
 * protocol_core_process(), and fills the response frame if needed.
 * No UART or BCAN PROTOL dependency -- suitable for unit testing.
 *
 * @param request      Incoming CAN frame (must not be NULL).
 * @param response     Output response CAN frame (only valid when
 *                     *has_response is true; must not be NULL).
 * @param has_response Set to true if a response should be sent
 *                     (must not be NULL).
 * @return 0 on success (including non-matching CAN ID), -EMSGSIZE if
 *         DLC != ASR_COMM_MSG_SIZE, or negative errno from protocol_core.
 */
int asr_can_protocol_handle_frame(
	const struct asr_can_frame *request,
	struct asr_can_frame *response,
	bool *has_response);

/**
 * Initialise the CAN protocol transport.
 *
 * Initialises the underlying BCAN PROTOL layer and reads the request/
 * response CAN IDs from Kconfig.
 *
 * @return 0 on success, negative errno on failure.
 */
int asr_can_protocol_transport_init(void);

/**
 * Process one CAN frame: receive, filter, dispatch, and optionally reply.
 *
 * @param timeout Maximum time to wait for a frame.
 * @return 0 on success (including non-matching CAN ID), 1 if a reply was
 *         sent, negative errno on error.
 */
int asr_can_protocol_process_once(k_timeout_t timeout);

/**
 * Run the CAN protocol processing loop forever.
 *
 * Calls protocol_core_init and can_protocol_transport_init, then loops
 * calling process_once.  Errors other than -ETIMEDOUT are logged.
 *
 * @return 0 on graceful exit (never under normal operation).
 */
int asr_can_protocol_process_forever(void);

#ifdef CONFIG_ZTEST
/**
 * Set CAN request/response IDs for testing (bypasses UART init).
 */
void asr_can_protocol_test_set_ids(uint32_t req, uint32_t resp);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ASR_CAN_PROTOCOL_TRANSPORT_H_ */