/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * GPIO LED group — devicetree compatible: asr,gpio-led-group
 *
 * Example devicetree (e.g. board overlay for XIAO RP2350):
 *
 *   / {
 *     my_leds: gpio-led-group {
 *       compatible = "asr,gpio-led-group";
 *       gpios = <&xiao_d 0 GPIO_ACTIVE_HIGH>, <&xiao_d 1 GPIO_ACTIVE_HIGH>;
 *     };
 *   };
 *
 * Example use:
 *
 *   static struct gpio_dt_spec specs[LED_GROUP_DT_NUM_PINS(DT_NODELABEL(my_leds))];
 *   static struct led_group leds;
 *
 *   LED_GROUP_DT_SPECS_INIT(DT_NODELABEL(my_leds), specs);
 *   led_group_init(&leds, specs, ARRAY_SIZE(specs));
 */

#ifndef ASR_LED_H_
#define ASR_LED_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Logical LED group: indices are 0 .. count-1. */
struct led_group {
	const struct gpio_dt_spec *pins;
	size_t count;
};

/**
 * Number of GPIO entries under @p node_id (requires property "gpios").
 */
#define LED_GROUP_DT_NUM_PINS(node_id) DT_PROP_LEN(node_id, gpios)

/** @internal Per-index assignment; index @p n must be an integer literal (LISTIFY). */
#define LED_GROUP_DT_SPEC_SET_IDX(n, node_id, specs_array)                                         \
	(specs_array)[n] = (struct gpio_dt_spec)GPIO_DT_SPEC_GET_BY_IDX(node_id, gpios, n)

/**
 * Fill @p specs_array with GPIO specs from devicetree @p node_id.
 * Caller must ensure the array length is at least LED_GROUP_DT_NUM_PINS(node_id).
 */
#define LED_GROUP_DT_SPECS_INIT(node_id, specs_array)                                              \
	do {                                                                                       \
		LISTIFY(DT_PROP_LEN(node_id, gpios), LED_GROUP_DT_SPEC_SET_IDX, (;), node_id,      \
			specs_array);                                                              \
	} while (0)

/**
 * Configure all pins as outputs (inactive / LED off) and attach @p pins to @p group.
 *
 * @param group  Group state (storage provided by caller).
 * @param pins   Array of GPIO specs; must remain valid for the lifetime of @p group.
 * @param count  Number of LEDs; must match devicetree gpios length when using DT helpers.
 * @return 0 on success, negative errno on error (e.g. -ENODEV, -EINVAL).
 */
int led_group_init(struct led_group *group, const struct gpio_dt_spec *pins, size_t count);

/** Drive one LED; logical @p on follows devicetree active level. No-op if index is invalid. */
void led_group_set(struct led_group *group, size_t index, bool on);

/** Set up to 32 LEDs from bit @c 0 = first LED. Extra mask bits are ignored. */
void led_group_set_mask(struct led_group *group, uint32_t mask);

/** Turn every LED on or off. */
void led_group_set_all(struct led_group *group, bool on);

/** Toggle one LED. Returns 0 on success, negative errno on failure. */
int led_group_toggle(struct led_group *group, size_t index);

#ifdef __cplusplus
}
#endif

#endif /* ASR_LED_H_ */
