/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * The application delegates LED, IMU, Dynamixel servo, and UART communication
 * activity to the ASR helper modules. Dynamixel bring-up and command handling
 * live in the dynamixel module (see asr/dynamixel.h and
 * asr/dynamixel_thread.h); main only wires the comm callbacks used by the
 * UART0 protocol path.
 */

#include <asr/comm_thread.h>
#include <asr/cpu_monitor_thread.h>
#include <asr/dynamixel.h>
#include <asr/dynamixel_thread.h>
#include <asr/imu.h>
#include <asr/imu_thread.h>
#include <asr/led_thread.h>
#include <asr/robot_base.h>

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(unit_system, LOG_LEVEL_INF);

#define UNIT_SYSTEM_THREAD_START_DELAY_MS 10U

unit_status_t unit_status;
K_MUTEX_DEFINE(unit_status_mutex);

static int handle_imu_read(uint8_t buf[ASR_COMM_MSG_SIZE])
{
	ARG_UNUSED(buf);
	return -ENOTSUP;
}

static const struct asr_comm_callbacks comm_callbacks = {
	.on_dynamixel_torque = asr_dynamixel_app_handle_torque,
	.on_dynamixel_goal_position = asr_dynamixel_app_handle_goal_position,
	.on_imu_read = handle_imu_read,
};

static void unit_system_log_banner(void)
{
	LOG_INF("==========================================");
	LOG_INF("Unit system application");
	LOG_INF("==========================================");
}

static bool unit_system_init_modules(void)
{
	int ret;
	bool comm_ready = false;

	ret = asr_led_thread_init();
	if (ret < 0) {
		LOG_WRN("LED thread init failed: %d (continuing without LED)", ret);
	}

	ret = asr_cpu_monitor_thread_init();
	if (ret < 0) {
		LOG_WRN("CPU monitor thread init failed: %d (continuing without CPU monitor)", ret);
	}

	asr_comm_register_callbacks(&comm_callbacks);
	ret = asr_comm_thread_init();
	if (ret < 0) {
		LOG_WRN("Comm thread init failed: %d (continuing without comm)", ret);
	} else {
		comm_ready = true;
	}

	ret = asr_dynamixel_thread_init();
	if (ret < 0) {
		LOG_WRN("Dynamixel thread init failed: %d (continuing without Dynamixel)", ret);
	}

	ret = asr_imu_thread_init();
	if (ret < 0) {
		LOG_WRN("IMU thread init failed: %d (continuing without IMU)", ret);
	}

	return comm_ready;
}

static void unit_system_start_modules(bool comm_ready)
{
	asr_led_thread_start();
	asr_cpu_monitor_thread_start();
	if (comm_ready) {
		asr_comm_thread_start();
	}
	asr_dynamixel_thread_start();
	asr_imu_thread_start();
}

int main(void)
{
	bool comm_ready;

	unit_system_log_banner();
	comm_ready = unit_system_init_modules();

	LOG_INF("all modules initialised, starting threads");
	k_sleep(K_MSEC(UNIT_SYSTEM_THREAD_START_DELAY_MS));

	unit_system_start_modules(comm_ready);

	LOG_INF("all background threads started");
	return 0;
}
