/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <asr/usr_led.h>

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_usr_led, LOG_LEVEL_INF);

/*
 * Prefer PWM (pwm-leds) for brightness control; fall back to GPIO (gpio-leds).
 * Board overlay selects which one is active via DT status.
 */

#if DT_HAS_COMPAT_STATUS_OKAY(pwm_leds)
#define USR_LED_PWM   1
#define PWM_LEDS_DEV  DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(pwm_leds))
#define PWM_LED_INDEX 0U
#else
#define USR_LED_PWM 0
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(gpio_leds) && DT_NODE_HAS_STATUS(DT_ALIAS(led0), okay)
#define USR_LED_GPIO 1
static const struct gpio_dt_spec usr_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#else
#define USR_LED_GPIO 0
#endif

#if !USR_LED_PWM && !USR_LED_GPIO
#error "No user LED: enable gpio-leds or pwm-leds in devicetree"
#endif

static bool usr_led_state;

int asr_usr_led_init(void)
{
#if USR_LED_PWM
	if (!device_is_ready(PWM_LEDS_DEV)) {
		LOG_ERR("PWM LED device not ready");
		return -ENODEV;
	}
	(void)led_off(PWM_LEDS_DEV, PWM_LED_INDEX);
#elif USR_LED_GPIO
	if (!gpio_is_ready_dt(&usr_gpio)) {
		LOG_ERR("User LED GPIO not ready");
		return -ENODEV;
	}
	int ret = gpio_pin_configure_dt(&usr_gpio, GPIO_OUTPUT_INACTIVE);

	if (ret < 0) {
		return ret;
	}
#endif
	usr_led_state = false;
	LOG_INF("User LED initialised");
	return 0;
}

void asr_usr_led_on(void)
{
#if USR_LED_PWM
	(void)led_set_brightness(PWM_LEDS_DEV, PWM_LED_INDEX, 100U);
#elif USR_LED_GPIO
	(void)gpio_pin_set_dt(&usr_gpio, 1);
#endif
	usr_led_state = true;
}

void asr_usr_led_off(void)
{
#if USR_LED_PWM
	(void)led_off(PWM_LEDS_DEV, PWM_LED_INDEX);
#elif USR_LED_GPIO
	(void)gpio_pin_set_dt(&usr_gpio, 0);
#endif
	usr_led_state = false;
}

void asr_usr_led_toggle(void)
{
	if (usr_led_state) {
		asr_usr_led_off();
	} else {
		asr_usr_led_on();
	}
}

int asr_usr_led_set_brightness(uint8_t brightness)
{
#if USR_LED_PWM
	if (brightness > 100U) {
		brightness = 100U;
	}
	usr_led_state = brightness > 0U;
	return led_set_brightness(PWM_LEDS_DEV, PWM_LED_INDEX, brightness);
#else
	ARG_UNUSED(brightness);
	return -ENOTSUP;
#endif
}

void asr_usr_led_app_handle_write(bool state)
{
	(void)asr_usr_led_set_brightness(state ? 100U : 0U);
}
