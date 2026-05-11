/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Background thread that runs the Dynamixel bring-up sequence so it does not
 * block main() while the servo PINGs and limit-readbacks are in flight. The
 * bring-up itself lives in dynamixel.c (asr_dynamixel_app_init); this file is
 * just the thread plumbing.
 */

#include <asr/dynamixel.h>
#include <asr/dynamixel_thread.h>

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(asr_dynamixel, LOG_LEVEL_INF);

#define DXL_THREAD_STACK_SIZE CONFIG_ASR_DYNAMIXEL_THREAD_STACK_SIZE
#define DXL_THREAD_PRIO       CONFIG_ASR_DYNAMIXEL_THREAD_PRIORITY
#define DXL_STARTUP_DELAY_MS  CONFIG_ASR_DYNAMIXEL_STARTUP_DELAY_MS

static K_THREAD_STACK_DEFINE(dxl_thread_stack, DXL_THREAD_STACK_SIZE);
static struct k_thread dxl_thread_data;
static bool dxl_thread_initialized;
static bool dxl_thread_started;

static void dxl_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int ret;

	if (DXL_STARTUP_DELAY_MS > 0) {
		k_sleep(K_MSEC(DXL_STARTUP_DELAY_MS));
	}

	ret = asr_dynamixel_app_init();
	if (ret < 0) {
		LOG_WRN("Dynamixel bring-up 失败: %d (继续运行)", ret);
	}

	/* No periodic work yet; park the thread so its stack stays reserved
	 * for future periodic state polling without terminating the thread. */
	k_sleep(K_FOREVER);
}

int asr_dynamixel_thread_init(void)
{
	if (dxl_thread_initialized) {
		LOG_WRN("Dynamixel thread already initialised");
		return -EALREADY;
	}

	k_thread_create(&dxl_thread_data, dxl_thread_stack,
			K_THREAD_STACK_SIZEOF(dxl_thread_stack),
			dxl_thread_entry, NULL, NULL, NULL,
			DXL_THREAD_PRIO, 0, K_FOREVER);
	k_thread_name_set(&dxl_thread_data, "dynamixel");
	dxl_thread_initialized = true;
	LOG_INF("Dynamixel thread initialised (suspended), prio %d",
		k_thread_priority_get(&dxl_thread_data));
	return 0;
}

int asr_dynamixel_thread_start(void)
{
	if (!dxl_thread_initialized) {
		LOG_ERR("Dynamixel thread not initialised");
		return -EINVAL;
	}

	if (dxl_thread_started) {
		LOG_WRN("Dynamixel thread already started");
		return -EALREADY;
	}

	dxl_thread_started = true;
	LOG_INF("Dynamixel thread started");
	k_thread_start(&dxl_thread_data);
	return 0;
}
