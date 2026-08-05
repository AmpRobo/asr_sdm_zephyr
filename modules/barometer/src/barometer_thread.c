/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <asr/barometer.h>
#include <asr/barometer_thread.h>

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(asr_barometer, LOG_LEVEL_INF);

#define BAROMETER_THREAD_STACK_SIZE CONFIG_ASR_BAROMETER_THREAD_STACK_SIZE
#define BAROMETER_THREAD_PRIORITY   CONFIG_ASR_BAROMETER_THREAD_PRIORITY
#define BAROMETER_STARTUP_DELAY_MS  CONFIG_ASR_BAROMETER_STARTUP_DELAY_MS
#define BAROMETER_PERIOD_MS         CONFIG_ASR_BAROMETER_PERIOD_MS
#define BAROMETER_LOG_INTERVAL_MS   CONFIG_ASR_BAROMETER_LOG_INTERVAL_MS
#define BAROMETER_OSR               CONFIG_ASR_BAROMETER_OSR

#define BAROMETER_OSR_VALID                                                    \
	((BAROMETER_OSR == 128) || (BAROMETER_OSR == 256) ||                    \
	 (BAROMETER_OSR == 512) || (BAROMETER_OSR == 1024) ||                   \
	 (BAROMETER_OSR == 2048) || (BAROMETER_OSR == 4096))

BUILD_ASSERT(BAROMETER_OSR_VALID,
	     "CONFIG_ASR_BAROMETER_OSR must be a supported HP206F value");
BUILD_ASSERT(BAROMETER_THREAD_STACK_SIZE > 0,
	     "CONFIG_ASR_BAROMETER_THREAD_STACK_SIZE must be > 0");
BUILD_ASSERT(BAROMETER_PERIOD_MS > 0,
	     "CONFIG_ASR_BAROMETER_PERIOD_MS must be > 0");
BUILD_ASSERT(BAROMETER_LOG_INTERVAL_MS > 0,
	     "CONFIG_ASR_BAROMETER_LOG_INTERVAL_MS must be > 0");

static K_THREAD_STACK_DEFINE(barometer_thread_stack,
			     BAROMETER_THREAD_STACK_SIZE);
static struct k_thread barometer_thread_data;
static bool barometer_thread_initialized;
static bool barometer_thread_started;

static K_SEM_DEFINE(barometer_tick_sem, 0, 1);
static struct k_timer barometer_timer;

static void barometer_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&barometer_tick_sem);
}

static void barometer_log_sample(const struct asr_barometer_sample *sample)
{
	LOG_INF("barometer: pressure=%.3f kPa, temperature=%.2f C",
		(double)sensor_value_to_double(&sample->pressure),
		(double)sensor_value_to_double(&sample->temperature));
}

static void barometer_thread_entry(void *p1, void *p2, void *p3)
{
	int64_t last_data_log_ms = 0;
	int64_t last_error_log_ms = 0;
	uint32_t consecutive_failures = 0;
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (BAROMETER_STARTUP_DELAY_MS > 0) {
		k_sleep(K_MSEC(BAROMETER_STARTUP_DELAY_MS));
	}

	ret = asr_barometer_init();
	if (ret < 0) {
		LOG_WRN("Barometer disabled: init failed (%d); other modules continue",
			ret);
		return;
	}

	LOG_INF("Barometer ready: HP206F compatible sensor, OSR=%d, period=%d ms",
		BAROMETER_OSR, BAROMETER_PERIOD_MS);

	k_timer_init(&barometer_timer, barometer_timer_expiry, NULL);
	k_timer_start(&barometer_timer, K_MSEC(BAROMETER_PERIOD_MS),
		      K_MSEC(BAROMETER_PERIOD_MS));
	k_sem_give(&barometer_tick_sem);

	for (;;) {
		struct asr_barometer_sample sample;
		int64_t now_ms;

		k_sem_take(&barometer_tick_sem, K_FOREVER);
		now_ms = k_uptime_get();

		ret = asr_barometer_update();
		if (ret < 0) {
			consecutive_failures++;
			if ((consecutive_failures == 1U) ||
			    ((now_ms - last_error_log_ms) >=
			     BAROMETER_LOG_INTERVAL_MS)) {
				last_error_log_ms = now_ms;
				LOG_ERR("Barometer read failed: %d; retrying every %d ms",
					ret, BAROMETER_PERIOD_MS);
			}
			continue;
		}

		if (consecutive_failures > 0U) {
			LOG_INF("Barometer sampling recovered after %u failures",
				consecutive_failures);
			consecutive_failures = 0U;
		}

		if ((last_data_log_ms == 0) ||
		    ((now_ms - last_data_log_ms) >= BAROMETER_LOG_INTERVAL_MS)) {
			last_data_log_ms = now_ms;
			if (asr_barometer_get_latest(&sample) == 0) {
				barometer_log_sample(&sample);
			}
		}
	}
}

int asr_barometer_thread_init(void)
{
	if (barometer_thread_initialized) {
		LOG_WRN("Barometer thread already initialised");
		return -EALREADY;
	}

	k_thread_create(&barometer_thread_data, barometer_thread_stack,
			K_THREAD_STACK_SIZEOF(barometer_thread_stack),
			barometer_thread_entry, NULL, NULL, NULL,
			BAROMETER_THREAD_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&barometer_thread_data, "barometer");
	barometer_thread_initialized = true;
	LOG_INF("Barometer thread initialised (suspended), prio %d",
		k_thread_priority_get(&barometer_thread_data));
	return 0;
}

int asr_barometer_thread_start(void)
{
	if (!barometer_thread_initialized) {
		LOG_ERR("Barometer thread not initialised");
		return -EINVAL;
	}

	if (barometer_thread_started) {
		LOG_WRN("Barometer thread already started");
		return -EALREADY;
	}

	barometer_thread_started = true;
	k_thread_start(&barometer_thread_data);
	LOG_INF("Barometer thread started");
	return 0;
}
