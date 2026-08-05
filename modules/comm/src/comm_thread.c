/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Communication processing thread for the ASR SDM protocol.
 *
 * Receives framed 8-byte messages from the UART-AA55 transport, dispatches
 * them through the transport-independent protocol core, and sends replies
 * back through the transport when the core indicates one is needed.
 */

#include <asr/comm_thread.h>
#include <asr/protocol_core.h>
#include <asr/transport_uart_aa55.h>

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(asr_comm, LOG_LEVEL_INF);

static K_THREAD_STACK_DEFINE(comm_stack, CONFIG_ASR_COMM_THREAD_STACK_SIZE);
static struct k_thread comm_thread_data;
static bool comm_started;

static void comm_thread_entry(void *p1, void *p2, void *p3)
{
	uint8_t msg[ASR_COMM_MSG_SIZE];

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/* Initialise the protocol core (transport-independent). */
	asr_protocol_core_init();

	/* Initialise the UART-AA55 transport. */
	if (asr_uart_aa55_init() < 0) {
		LOG_ERR("UART0 AA55 transport init failed, comm thread exiting");
		return;
	}

	for (;;) {
		struct asr_protocol_result result;

		/* Block until a complete 8-byte message arrives. */
		if (asr_uart_aa55_receive(msg, K_FOREVER) < 0) {
			continue;
		}

		/* Process the request (result is zeroed inside). */
		int ret = asr_protocol_core_process(msg, &result);

		if (ret < 0) {
			LOG_DBG("core process: cmd=0x%02x param=0x%02x -> %d",
				msg[2], msg[3], ret);
		}

		/* Send a reply only when the core explicitly requests it. */
		if (result.has_response) {
			asr_uart_aa55_send(result.response);
		}
	}
}

int asr_comm_thread_init(void)
{
	if (comm_started) {
		LOG_WRN("communication thread already initialised");
		return -EALREADY;
	}

	k_thread_create(&comm_thread_data, comm_stack,
			K_THREAD_STACK_SIZEOF(comm_stack),
			comm_thread_entry, NULL, NULL, NULL,
			CONFIG_ASR_COMM_THREAD_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&comm_thread_data, "comm_thread");
	comm_started = true;

	LOG_INF("communication thread initialised (suspended), prio %d",
		k_thread_priority_get(&comm_thread_data));
	return 0;
}

int asr_comm_thread_start(void)
{
	if (!comm_started) {
		LOG_ERR("communication thread not initialised");
		return -EINVAL;
	}
	LOG_INF("communication thread started");
	k_thread_start(&comm_thread_data);
	return 0;
}