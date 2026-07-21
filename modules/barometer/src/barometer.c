/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <asr/barometer.h>

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_barometer, LOG_LEVEL_INF);

#define BAROMETER_NODE DT_ALIAS(asr_barometer)

static struct asr_barometer_sample cached_sample;
static bool cached_sample_valid;
static K_MUTEX_DEFINE(cached_sample_mutex);

int asr_barometer_get_latest(struct asr_barometer_sample *sample)
{
	int ret = -ENODATA;

	if (sample == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&cached_sample_mutex, K_FOREVER);
	if (cached_sample_valid) {
		*sample = cached_sample;
		ret = 0;
	}
	k_mutex_unlock(&cached_sample_mutex);

	return ret;
}

#if DT_NODE_EXISTS(BAROMETER_NODE) && DT_NODE_HAS_STATUS(BAROMETER_NODE, okay)

#define BAROMETER_DEV DEVICE_DT_GET(BAROMETER_NODE)

int asr_barometer_init(void)
{
	const struct sensor_value osr = {
		.val1 = CONFIG_ASR_BAROMETER_OSR,
		.val2 = 0,
	};
	int ret;

	if (!device_is_ready(BAROMETER_DEV)) {
		LOG_ERR("Barometer device not ready");
		return -ENODEV;
	}

	ret = sensor_attr_set(BAROMETER_DEV, SENSOR_CHAN_ALL,
			      SENSOR_ATTR_OVERSAMPLING, &osr);
	if (ret < 0) {
		LOG_ERR("Barometer OSR configuration failed: %d", ret);
		return ret;
	}

	return 0;
}

int asr_barometer_read(struct asr_barometer_sample *sample)
{
	int ret;

	if (sample == NULL) {
		return -EINVAL;
	}

	ret = sensor_sample_fetch(BAROMETER_DEV);
	if (ret < 0) {
		return ret;
	}

	ret = sensor_channel_get(BAROMETER_DEV, SENSOR_CHAN_PRESS,
				 &sample->pressure);
	if (ret < 0) {
		return ret;
	}

	return sensor_channel_get(BAROMETER_DEV, SENSOR_CHAN_AMBIENT_TEMP,
				  &sample->temperature);
}

int asr_barometer_update(void)
{
	struct asr_barometer_sample sample;
	int ret;

	ret = asr_barometer_read(&sample);
	if (ret < 0) {
		return ret;
	}

	k_mutex_lock(&cached_sample_mutex, K_FOREVER);
	cached_sample = sample;
	cached_sample_valid = true;
	k_mutex_unlock(&cached_sample_mutex);

	return 0;
}

#else

int asr_barometer_init(void)
{
	LOG_ERR("Missing enabled asr-barometer alias node");
	return -ENODEV;
}

int asr_barometer_read(struct asr_barometer_sample *sample)
{
	if (sample == NULL) {
		return -EINVAL;
	}

	return -ENODEV;
}

int asr_barometer_update(void)
{
	return -ENODEV;
}

#endif
