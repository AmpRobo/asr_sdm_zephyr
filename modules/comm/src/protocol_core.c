/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Transport-independent ASR SDM application protocol core.
 *
 * Handles fixed 8-byte protocol messages: parses the command/param fields,
 * updates unit_status, dispatches hardware callbacks, and returns a result
 * object that tells the transport layer whether to send a reply.
 *
 * This file MUST NOT include or reference any UART, CAN, DeviceTree, ISR,
 * message queue, semaphore, or transport-specific code.
 */

#include <asr/protocol_core.h>

#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(asr_protocol_core, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------
 * Internal state
 * ------------------------------------------------------------------------- */

static asr_unit_status_t unit_status;
static const struct asr_comm_callbacks *hw_cb;

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void asr_protocol_core_register_callbacks(
	const struct asr_comm_callbacks *callbacks)
{
	hw_cb = callbacks;
}

const asr_unit_status_t *asr_protocol_core_get_status(void)
{
	return &unit_status;
}

int asr_protocol_core_init(void)
{
	memset(&unit_status, 0, sizeof(unit_status));
	unit_status.unit_id = 0U;
	return 0;
}

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

static int handle_dynamixel_position_read(uint8_t id,
					  struct asr_protocol_result *result)
{
	int32_t position;
	int ret;

	if ((hw_cb == NULL) || (hw_cb->on_dynamixel_position_read == NULL)) {
		LOG_WRN("joint%u read: position callback not registered", id + 1U);
		return -ENOSYS;
	}

	ret = hw_cb->on_dynamixel_position_read(id, &position);
	if (ret != 0) {
		LOG_WRN("joint%u read callback failed: %d", id + 1U, ret);
		return ret < 0 ? ret : -EIO;
	}

	memset(result->response, 0, sizeof(result->response));
	sys_put_le32((uint32_t)position, &result->response[0]);
	result->has_response = true;
	return 0;
}

static int handle_read(
	const uint8_t request[ASR_COMM_MSG_SIZE],
	struct asr_protocol_result *result)
{
	switch (request[3]) {
	case ASR_COMM_PARAM_IMU:
		if ((hw_cb == NULL) || (hw_cb->on_imu_read == NULL)) {
			LOG_WRN("IMU read: callback not registered");
			return -ENOSYS;
		}

		memset(result->response, 0, sizeof(result->response));

		{
			int ret = hw_cb->on_imu_read(result->response);

			if (ret != 0) {
				LOG_WRN("IMU read callback failed: %d", ret);
				return ret < 0 ? ret : -EIO;
			}
		}

		result->has_response = true;
		return 0;

	case ASR_COMM_PARAM_JOINT1:
		return handle_dynamixel_position_read(ASR_DXL_1, result);

	case ASR_COMM_PARAM_BOARD_ID:
	case ASR_COMM_PARAM_CAN_ID:
	case ASR_COMM_PARAM_LED_ENABLE:
	case ASR_COMM_PARAM_LED_STATUS:
	case ASR_COMM_PARAM_MOTOR:
	case ASR_COMM_PARAM_JOINT1_TORQUE:
	case ASR_COMM_PARAM_JOINT2_TORQUE:
		LOG_DBG("read param=0x%02x not supported (no reply)", request[3]);
		return -ENOTSUP;

	default:
		LOG_WRN("unknown read param=0x%02x", request[3]);
		return -ENOTSUP;
	}
}

static int handle_write(
	const uint8_t request[ASR_COMM_MSG_SIZE],
	struct asr_protocol_result *result)
{
	ARG_UNUSED(result);

	switch (request[3]) {
	case ASR_COMM_PARAM_CAN_ID:
		unit_status.flash_data[0] = request[6];
		unit_status.flash_data[1] = request[7];

		if ((hw_cb == NULL) || (hw_cb->on_flash_write == NULL)) {
			LOG_WRN("flash write: callback not registered");
			return -ENOSYS;
		}

		hw_cb->on_flash_write(
			unit_status.flash_data,
			sizeof(unit_status.flash_data));
		return 0;

	case ASR_COMM_PARAM_LED_ENABLE:
		if (request[7] == 1U) {
			unit_status.led_enable = true;
			return 0;
		}

		if (request[7] == 0U) {
			unit_status.led_enable = false;
			unit_status.led_status = true;

			if ((hw_cb == NULL) || (hw_cb->on_led_write == NULL)) {
				LOG_WRN("led write: callback not registered");
				return -ENOSYS;
			}

			hw_cb->on_led_write(true);
			return 0;
		}

		LOG_WRN("led enable: invalid value %u", request[7]);
		return -EINVAL;

	case ASR_COMM_PARAM_LED_STATUS:
		/* Not yet implemented. */
		return -ENOTSUP;

	case ASR_COMM_PARAM_MOTOR:
		unit_status.cmd_motor[0] = request[6];
		unit_status.cmd_motor[1] = request[7];

		if ((hw_cb == NULL) || (hw_cb->on_motor_set == NULL)) {
			LOG_WRN("motor set: callback not registered");
			return -ENOSYS;
		}

		hw_cb->on_motor_set(0, unit_status.cmd_motor[0]);
		hw_cb->on_motor_set(1, unit_status.cmd_motor[1]);
		return 0;

	case ASR_COMM_PARAM_JOINT1:
		memcpy(unit_status.cmd_joint1, &request[4], 4U);

		if ((hw_cb == NULL) ||
		    (hw_cb->on_dynamixel_goal_position == NULL)) {
			LOG_WRN("joint1: goal position callback not registered");
			return -ENOSYS;
		}

		hw_cb->on_dynamixel_goal_position(
			ASR_DXL_1,
			(int32_t)sys_get_le32(&request[4]));
		return 0;

	case ASR_COMM_PARAM_JOINT2:
		memcpy(unit_status.cmd_joint2, &request[4], 4U);

		if ((hw_cb == NULL) ||
		    (hw_cb->on_dynamixel_goal_position == NULL)) {
			LOG_WRN("joint2: goal position callback not registered");
			return -ENOSYS;
		}

		hw_cb->on_dynamixel_goal_position(
			ASR_DXL_2,
			(int32_t)sys_get_le32(&request[4]));
		return 0;

	case ASR_COMM_PARAM_JOINT1_TORQUE:
		if (request[4] > 1U) {
			LOG_WRN("joint1 torque: invalid value %u", request[4]);
			return -EINVAL;
		}

		unit_status.dynamixel_enable[ASR_DXL_1] =
			request[4] != 0U;

		if ((hw_cb == NULL) ||
		    (hw_cb->on_dynamixel_torque == NULL)) {
			LOG_WRN("joint1 torque: callback not registered");
			return -ENOSYS;
		}

		hw_cb->on_dynamixel_torque(
			ASR_DXL_1,
			unit_status.dynamixel_enable[ASR_DXL_1]);
		return 0;

	case ASR_COMM_PARAM_JOINT2_TORQUE:
		if (request[4] > 1U) {
			LOG_WRN("joint2 torque: invalid value %u", request[4]);
			return -EINVAL;
		}

		unit_status.dynamixel_enable[ASR_DXL_2] =
			request[4] != 0U;

		if ((hw_cb == NULL) ||
		    (hw_cb->on_dynamixel_torque == NULL)) {
			LOG_WRN("joint2 torque: callback not registered");
			return -ENOSYS;
		}

		hw_cb->on_dynamixel_torque(
			ASR_DXL_2,
			unit_status.dynamixel_enable[ASR_DXL_2]);
		return 0;

	default:
		LOG_WRN("unknown write param=0x%02x", request[3]);
		return -ENOTSUP;
	}
}

/* -------------------------------------------------------------------------
 * Main processing entry point
 * ------------------------------------------------------------------------- */

int asr_protocol_core_process(
	const uint8_t request[ASR_COMM_MSG_SIZE],
	struct asr_protocol_result *result)
{
	if ((request == NULL) || (result == NULL)) {
		return -EINVAL;
	}

	/* Zero-initialise the result; has_response defaults to false. */
	memset(result, 0, sizeof(*result));

	/* Preserve the existing rejection of Dynamixel bulk-read frames. */
	if ((request[0] == 0xFFU) && (request[1] == 0xFDU)) {
		LOG_DBG("rejected Dynamixel bulk-read frame");
		return -EINVAL;
	}

	switch (request[2]) {
	case ASR_COMM_CMD_READ:
		return handle_read(request, result);

	case ASR_COMM_CMD_WRITE:
		return handle_write(request, result);

	default:
		LOG_WRN("unknown cmd=0x%02x", request[2]);
		return -ENOTSUP;
	}
}