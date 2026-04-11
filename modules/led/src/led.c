/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <asr/led.h>

#include <errno.h>

#include <zephyr/sys/util.h>

int led_group_init(struct led_group *group, const struct gpio_dt_spec *pins, size_t count)
{
	if (group == NULL || pins == NULL || count == 0U) {
		return -EINVAL;
	}

	group->pins = pins;
	group->count = count;

	for (size_t i = 0; i < count; i++) {
		if (!gpio_is_ready_dt(&pins[i])) {
			return -ENODEV;
		}

		const int ret = gpio_pin_configure_dt(&pins[i], GPIO_OUTPUT_INACTIVE);

		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

void led_group_set(struct led_group *group, size_t index, bool on)
{
	if (group == NULL || group->pins == NULL || index >= group->count) {
		return;
	}

	(void)gpio_pin_set_dt(&group->pins[index], on ? 1 : 0);
}

void led_group_set_mask(struct led_group *group, uint32_t mask)
{
	if (group == NULL || group->pins == NULL) {
		return;
	}

	const size_t n = MIN(group->count, 32U);

	for (size_t i = 0; i < n; i++) {
		const bool on = ((mask >> i) & 1U) != 0U;

		(void)gpio_pin_set_dt(&group->pins[i], on ? 1 : 0);
	}
}

void led_group_set_all(struct led_group *group, bool on)
{
	if (group == NULL || group->pins == NULL) {
		return;
	}

	for (size_t i = 0; i < group->count; i++) {
		(void)gpio_pin_set_dt(&group->pins[i], on ? 1 : 0);
	}
}

int led_group_toggle(struct led_group *group, size_t index)
{
	if (group == NULL || group->pins == NULL || index >= group->count) {
		return -EINVAL;
	}

	return gpio_pin_toggle_dt(&group->pins[index]);
}
