/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CAN protocol transport adapter.
 *
 * Bridges the BCAN PROTOL transport layer and the transport-independent
 * protocol core.  CAN frames with the configured request CAN ID and DLC=8
 * are forwarded to protocol_core_process(); if a reply is needed, the
 * response CAN ID is set and the frame is sent back via PROTOL.
 */

#include <asr/can_protocol_transport.h>
#include <asr/bcan_protol.h>
#include <asr/protocol_core.h>

#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_can_protocol, LOG_LEVEL_INF);

static uint32_t request_can_id;
static uint32_t response_can_id;

#ifdef CONFIG_ZTEST
void asr_can_protocol_test_set_ids(uint32_t req, uint32_t resp)
{
	request_can_id = req;
	response_can_id = resp;
}
#endif

/* -------------------------------------------------------------------------
 * Pure business mapping function (testable without UART)
 * ------------------------------------------------------------------------- */

int asr_can_protocol_handle_frame(
	const struct asr_can_frame *request,
	struct asr_can_frame *response,
	bool *has_response)
{
	struct asr_protocol_result result;

	if ((request == NULL) || (response == NULL) || (has_response == NULL)) {
		return -EINVAL;
	}

	*has_response = false;

	/* Filter by request CAN ID.  Non-matching frames are silently ignored. */
	if (request->can_id != request_can_id) {
		return 0;
	}

	/* The protocol core requires exactly 8-byte payloads. */
	if (request->dlc != ASR_COMM_MSG_SIZE) {
		return -EMSGSIZE;
	}

	int ret = asr_protocol_core_process(request->data, &result);

	if (result.has_response) {
		memset(response, 0, sizeof(*response));
		response->can_id = response_can_id;
		response->dlc = ASR_COMM_MSG_SIZE;
		memcpy(response->data, result.response, ASR_COMM_MSG_SIZE);
		*has_response = true;
	}

	return ret;
}

/* -------------------------------------------------------------------------
 * Initialisation
 * ------------------------------------------------------------------------- */

int asr_can_protocol_transport_init(void)
{
	int ret;

	ret = asr_bcan_protol_init();
	if (ret < 0) {
		return ret;
	}

	request_can_id = CONFIG_ASR_CAN_PROTOCOL_REQUEST_CAN_ID;
	response_can_id = CONFIG_ASR_CAN_PROTOCOL_RESPONSE_CAN_ID;

	LOG_INF("CAN protocol transport: request=0x%03x response=0x%03x",
		request_can_id, response_can_id);
	return 0;
}

/* -------------------------------------------------------------------------
 * Processing loop
 * ------------------------------------------------------------------------- */

int asr_can_protocol_process_once(k_timeout_t timeout)
{
	struct asr_can_frame request_frame;
	struct asr_can_frame response_frame;
	bool has_response;
	int ret;

	ret = asr_bcan_protol_receive(&request_frame, timeout);
	if (ret < 0) {
		return ret;
	}

	ret = asr_can_protocol_handle_frame(&request_frame, &response_frame,
					    &has_response);
	if (ret < 0) {
		return ret;
	}

	if (has_response) {
		return asr_bcan_protol_send(&response_frame);
	}

	return ret;
}

int asr_can_protocol_process_forever(void)
{
	int ret;

	ret = asr_protocol_core_init();
	if (ret < 0) {
		return ret;
	}

	ret = asr_can_protocol_transport_init();
	if (ret < 0) {
		return ret;
	}

	while (true) {
		ret = asr_can_protocol_process_once(K_FOREVER);

		if ((ret < 0) &&
		    (ret != -ETIMEDOUT) &&
		    (ret != -ENOTSUP) &&
		    (ret != -EMSGSIZE)) {
			LOG_WRN("CAN protocol processing failed: %d", ret);
		}
	}

	return 0;
}