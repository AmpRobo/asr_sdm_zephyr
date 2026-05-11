/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Application-level Dynamixel helpers: bring-up sequence, cached position
 * limits, and the comm-callback handlers wired up by the screw-unit
 * application. The lower-level register-access helpers live in
 * dynamixel_bus.c / dynamixel_packet.c.
 */

#include <asr/dynamixel.h>

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(asr_dynamixel, LOG_LEVEL_INF);

/* The screw-unit currently maps a single physical servo. The protocol layer
 * passes joint indices (0 == JOINT1) which we compare against this constant
 * before forwarding to the driver. */
#define ASR_DYNAMIXEL_PRIMARY_INSTANCE 0U

#define DXL_MODEL_XM430_W350        1020U
#define DXL_MODE_POSITION_CONTROL   3U
#define DXL_STATUS_RETURN_LEVEL_ALL 2U

static struct {
	bool ready;
	int32_t min_position;
	int32_t max_position;
} dxl_state;

static bool dxl_goal_position_valid(int32_t goal_position)
{
	return (goal_position >= dxl_state.min_position) &&
	       (goal_position <= dxl_state.max_position);
}

bool asr_dynamixel_app_is_ready(void)
{
	return dxl_state.ready;
}

int asr_dynamixel_app_init(void)
{
	uint16_t model_number = 0U;
	uint8_t firmware_version = 0U;
	uint8_t mode = 0U;
	uint8_t status_return_level = 0U;
	int32_t present_position = 0;
	int ret;

	memset(&dxl_state, 0, sizeof(dxl_state));

	ret = asr_dynamixel_init();
	if (ret < 0) {
		LOG_WRN("Dynamixel 驱动初始化失败: %d", ret);
		return ret;
	}

	LOG_INF("开始 Dynamixel bring-up: UART1, baud=57600, dir=GP28(D2), id=1");

	ret = asr_dynamixel_ping(&model_number, &firmware_version);
	if (ret < 0) {
		LOG_WRN("Dynamixel PING 失败: %d", ret);
		if (ret == -ETIMEDOUT) {
			LOG_WRN("未收到舵机状态包; 优先检查 TX->RXD、RX<-TXD、A/B、共地和 DIR 极性");
		}
		return ret;
	}

	LOG_INF("Dynamixel online: model=%u fw=%u",
		(unsigned int)model_number,
		(unsigned int)firmware_version);

	if (model_number != DXL_MODEL_XM430_W350) {
		LOG_WRN("检测到的型号不是 XM430-W350-R: %u", (unsigned int)model_number);
	}

	ret = asr_dynamixel_get_status_return_level(&status_return_level);
	if (ret < 0) {
		LOG_WRN("读取 Status Return Level 失败: %d", ret);
		return ret;
	}

	LOG_INF("Status Return Level=%u", (unsigned int)status_return_level);
	if (status_return_level != DXL_STATUS_RETURN_LEVEL_ALL) {
		ret = asr_dynamixel_set_status_return_level(DXL_STATUS_RETURN_LEVEL_ALL);
		if (ret < 0) {
			LOG_WRN("设置 Status Return Level 失败: %d", ret);
			return ret;
		}
		LOG_INF("Status Return Level 已设置为 %u",
			(unsigned int)DXL_STATUS_RETURN_LEVEL_ALL);
	}

	ret = asr_dynamixel_get_operating_mode(&mode);
	if (ret < 0) {
		LOG_WRN("读取 Operating Mode 失败: %d", ret);
		return ret;
	}

	LOG_INF("Operating Mode=%u", (unsigned int)mode);
	if (mode != DXL_MODE_POSITION_CONTROL) {
		ret = asr_dynamixel_set_torque(false);
		if (ret < 0) {
			LOG_WRN("关闭舵机扭矩失败: %d", ret);
			return ret;
		}

		ret = asr_dynamixel_set_operating_mode(DXL_MODE_POSITION_CONTROL);
		if (ret < 0) {
			LOG_WRN("设置 Operating Mode 失败: %d", ret);
			return ret;
		}

		LOG_INF("Operating Mode 已切换为 Position Control(%u)",
			(unsigned int)DXL_MODE_POSITION_CONTROL);
	}

	ret = asr_dynamixel_get_position_limits(&dxl_state.min_position,
						&dxl_state.max_position);
	if (ret < 0) {
		LOG_WRN("读取位置限位失败: %d", ret);
		return ret;
	}

	ret = asr_dynamixel_get_present_position(&present_position);
	if (ret < 0) {
		LOG_WRN("读取当前位置失败: %d", ret);
		return ret;
	}

	dxl_state.ready = true;
	LOG_INF("Dynamixel ready: pos=%ld, limit=[%ld, %ld]",
		(long)present_position,
		(long)dxl_state.min_position,
		(long)dxl_state.max_position);

	return 0;
}

void asr_dynamixel_app_handle_torque(uint8_t id, bool enable)
{
	int ret;

	if (id != ASR_DYNAMIXEL_PRIMARY_INSTANCE) {
		LOG_DBG("忽略未映射的舵机扭矩命令: joint=%u", id + 1U);
		return;
	}

	if (!dxl_state.ready) {
		LOG_WRN("舵机未就绪, 忽略扭矩命令");
		return;
	}

	ret = asr_dynamixel_set_torque(enable);
	if (ret < 0) {
		LOG_ERR("舵机扭矩设置失败: %d", ret);
		return;
	}

	LOG_INF("舵机扭矩已%s", enable ? "使能" : "关闭");
}

void asr_dynamixel_app_handle_goal_position(uint8_t id, int32_t goal_position)
{
	int ret;
	int32_t present_position;

	if (id != ASR_DYNAMIXEL_PRIMARY_INSTANCE) {
		LOG_DBG("忽略未映射的舵机位置命令: joint=%u", id + 1U);
		return;
	}

	if (!dxl_state.ready) {
		LOG_WRN("舵机未就绪, 忽略位置命令");
		return;
	}

	if (!dxl_goal_position_valid(goal_position)) {
		LOG_WRN("舵机目标位置越界: goal=%ld, limit=[%ld, %ld]",
			(long)goal_position,
			(long)dxl_state.min_position,
			(long)dxl_state.max_position);
		return;
	}

	ret = asr_dynamixel_set_goal_position(goal_position);
	if (ret < 0) {
		LOG_ERR("舵机目标位置设置失败: %d", ret);
		return;
	}

	ret = asr_dynamixel_get_present_position(&present_position);
	if (ret < 0) {
		LOG_WRN("舵机位置回读失败: %d", ret);
		return;
	}

	LOG_INF("舵机目标位置=%ld, 当前回读位置=%ld",
		(long)goal_position, (long)present_position);
}

int asr_dynamixel_app_handle_position_read(uint8_t id, int32_t *position)
{
	int ret;
	int32_t present_position;

	if (id != ASR_DYNAMIXEL_PRIMARY_INSTANCE) {
		LOG_DBG("忽略未映射的舵机读位置命令: joint=%u", id + 1U);
		return -ENOENT;
	}

	if (!dxl_state.ready) {
		LOG_WRN("舵机未就绪, 忽略读位置命令");
		return -EAGAIN;
	}

	if (position == NULL) {
		return -EINVAL;
	}

	ret = asr_dynamixel_get_present_position(&present_position);
	if (ret < 0) {
		LOG_ERR("读取舵机当前位置失败: %d", ret);
		return ret;
	}

	*position = present_position;
	LOG_INF("舵机当前位置=%ld", (long)present_position);
	return 0;
}
