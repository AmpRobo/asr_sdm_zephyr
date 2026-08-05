/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CAN protocol processing thread.
 *
 * Background thread that runs the CAN protocol transport loop.
 * Callbacks registered via asr_protocol_core_register_callbacks() must be
 * set up before this thread is started.
 */

#include <asr/can_protocol_thread.h>
#include <asr/can_protocol_transport.h>

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_can_protocol_thread, LOG_LEVEL_INF);

static K_THREAD_STACK_DEFINE(can_proto_stack,
			     CONFIG_ASR_CAN_PROTOCOL_THREAD_STACK_SIZE);
static struct k_thread can_proto_thread_data;
static bool can_proto_started;

static void can_proto_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int ret = asr_can_protocol_process_forever();

	LOG_ERR("CAN protocol thread exited unexpectedly: %d", ret);
}

int asr_can_protocol_thread_init(void)
{
	if (can_proto_started) {
		LOG_WRN("CAN protocol thread already initialised");
		return -EALREADY;
	}

	k_thread_create(&can_proto_thread_data, can_proto_stack,
			K_THREAD_STACK_SIZEOF(can_proto_stack),
			can_proto_thread_entry, NULL, NULL, NULL,
			CONFIG_ASR_CAN_PROTOCOL_THREAD_PRIORITY,
			0, K_FOREVER);
	k_thread_name_set(&can_proto_thread_data, "can_proto");
	can_proto_started = true;

	LOG_INF("CAN protocol thread initialised (suspended), prio %d",
		k_thread_priority_get(&can_proto_thread_data));
	return 0;
}

int asr_can_protocol_thread_start(void)
{
	if (!can_proto_started) {
		LOG_ERR("CAN protocol thread not initialised");
		return -EINVAL;
	}

	k_thread_start(&can_proto_thread_data);
	LOG_INF("CAN protocol thread started");
	return 0;
}