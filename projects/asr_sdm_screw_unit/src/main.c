/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * 应用入口负责装配模块回调并管理模块生命周期。
 * UART0 连接 BCAN-S01 并运行 PROTOL CAN 传输；
 * UART1 用作 Dynamixel RS-485 总线。
 */

#include <asr/barometer_thread.h>
#include <asr/can_protocol_thread.h>
#include <asr/cpu_monitor_thread.h>
#include <asr/dynamixel.h>
#include <asr/dynamixel_thread.h>
#include <asr/imu.h>
#include <asr/imu_thread.h>
#include <asr/led_thread.h>
#include <asr/protocol_core.h>
#include <asr/robot_base.h>
#include <asr/usr_led.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(unit_system, LOG_LEVEL_INF);

unit_status_t unit_status;
K_MUTEX_DEFINE(unit_status_mutex);

static void unit_system_log_banner(void)
{
	LOG_INF("==========================================");
	LOG_INF("Unit system application");
	LOG_INF("==========================================");
}

/* -------------------------------------------------------------------------
 * Protocol core callbacks
 *
 * These are registered with protocol_core so that CAN frames carrying
 * ASR protocol messages dispatch to the correct hardware module.
 * ------------------------------------------------------------------------- */

static const struct asr_comm_callbacks app_callbacks = {
	.on_led_write            = asr_usr_led_app_handle_write,
	.on_dynamixel_torque     = asr_dynamixel_app_handle_torque,
	.on_dynamixel_goal_position = asr_dynamixel_app_handle_goal_position,
	.on_dynamixel_position_read = asr_dynamixel_app_handle_position_read,
	.on_imu_read             = asr_imu_app_handle_read,
};

/* -------------------------------------------------------------------------
 * Module lifecycle
 * ------------------------------------------------------------------------- */

struct unit_module_state {
	bool dynamixel_ready;
	bool can_proto_ready;
	bool barometer_ready;
};

static struct unit_module_state unit_system_init_modules(void)
{
	struct unit_module_state state = {0};
	int ret;

	ret = asr_led_thread_init();
	if (ret < 0) {
		LOG_WRN("LED thread init failed: %d (continuing without LED)", ret);
	}

	ret = asr_cpu_monitor_thread_init();
	if (ret < 0) {
		LOG_WRN("CPU monitor thread init failed: %d (continuing without CPU monitor)", ret);
	}

	ret = asr_dynamixel_thread_init();
	if (ret < 0) {
		LOG_WRN("Dynamixel thread init failed: %d (continuing without Dynamixel)", ret);
	} else {
		state.dynamixel_ready = true;
	}

	/* Register protocol core callbacks before starting the CAN thread. */
	asr_protocol_core_register_callbacks(&app_callbacks);

	ret = asr_can_protocol_thread_init();
	if (ret < 0) {
		LOG_WRN("CAN protocol thread init failed: %d (continuing without CAN)", ret);
	} else {
		state.can_proto_ready = true;
	}

	ret = asr_imu_thread_init();
	if (ret < 0) {
		LOG_WRN("IMU thread init failed: %d (continuing without IMU)", ret);
	}

	ret = asr_barometer_thread_init();
	if (ret < 0) {
		LOG_WRN("Barometer thread init failed: %d (continuing without barometer)",
			ret);
	} else {
		state.barometer_ready = true;
	}

	return state;
}

static void unit_system_start_modules(const struct unit_module_state *state)
{
	asr_led_thread_start();
	asr_cpu_monitor_thread_start();
	if (state->dynamixel_ready) {
		asr_dynamixel_thread_start();
	}
	asr_imu_thread_start();
	if (state->can_proto_ready) {
		asr_can_protocol_thread_start();
	}
	if (state->barometer_ready) {
		int ret = asr_barometer_thread_start();

		if (ret < 0) {
			LOG_WRN("Barometer thread start failed: %d", ret);
		}
	}
}

int main(void)
{
	struct unit_module_state state;

	unit_system_log_banner();
	state = unit_system_init_modules();

	unit_system_start_modules(&state);
	LOG_INF("all modules initialised and background threads started");
	return 0;
}