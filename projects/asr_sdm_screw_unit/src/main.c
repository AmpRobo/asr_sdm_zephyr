/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * The application delegates LED, IMU, and UART communication activity to the
 * ASR helper modules.
 */

#include <asr/comm_thread.h>
#include <asr/cpu_monitor_thread.h>
#include <asr/dynamixel.h>
#include <asr/imu.h>
#include <asr/imu_thread.h>
#include <asr/led_thread.h>
#include <asr/robot_base.h>
#include <asr/usb_protocol.h>
#include <asr/usb_thread.h>

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(unit_system, LOG_LEVEL_INF);

#define DXL_MODEL_XM430_W350        1020U
#define DXL_MODE_POSITION_CONTROL   3U
#define DXL_STATUS_RETURN_LEVEL_ALL 2U
#define ASR_FRAME_SYNC_HI           0xAAU
#define ASR_FRAME_SYNC_LO           0x55U

unit_status_t unit_status;
K_MUTEX_DEFINE(unit_status_mutex);

static struct {
	bool ready;
	int32_t min_position;
	int32_t max_position;
} dynamixel_state;

static struct {
	uint8_t state;
	uint8_t payload_len;
	uint8_t payload_pos;
	uint8_t checksum;
	uint8_t payload[ASR_COMM_MSG_SIZE];
} usb_frame_parser;

static void handle_dynamixel_torque(uint8_t id, bool enable);
static void handle_dynamixel_goal_position(uint8_t id, int32_t goal_position);
static int handle_imu_read(uint8_t buf[ASR_COMM_MSG_SIZE]);

static const struct asr_comm_callbacks comm_callbacks = {
	.on_dynamixel_torque = handle_dynamixel_torque,
	.on_dynamixel_goal_position = handle_dynamixel_goal_position,
	.on_imu_read = handle_imu_read,
};

static bool dynamixel_goal_position_valid(int32_t goal_position)
{
	return (goal_position >= dynamixel_state.min_position) &&
	       (goal_position <= dynamixel_state.max_position);
}

static void usb_control_reset_parser(void)
{
	memset(&usb_frame_parser, 0, sizeof(usb_frame_parser));
}

static int usb_control_send_payload(const uint8_t payload[ASR_COMM_MSG_SIZE])
{
	uint8_t frame[3U + ASR_COMM_MSG_SIZE + 1U];
	uint8_t checksum = ASR_COMM_MSG_SIZE;

	frame[0] = ASR_FRAME_SYNC_HI;
	frame[1] = ASR_FRAME_SYNC_LO;
	frame[2] = ASR_COMM_MSG_SIZE;
	memcpy(&frame[3], payload, ASR_COMM_MSG_SIZE);

	for (size_t i = 0; i < ASR_COMM_MSG_SIZE; ++i) {
		checksum ^= payload[i];
	}

	frame[3U + ASR_COMM_MSG_SIZE] = checksum;
	return asr_usb_protocol_send(frame, sizeof(frame));
}

static void handle_dynamixel_position_read(uint8_t id)
{
	uint8_t reply[ASR_COMM_MSG_SIZE] = {0};
	int32_t present_position;
	int ret;

	if (id != ASR_DXL_1) {
		LOG_DBG("忽略未映射的舵机读位置命令: joint=%u", id + 1U);
		return;
	}

	if (!dynamixel_state.ready) {
		LOG_WRN("舵机未就绪, 忽略读位置命令");
		return;
	}

	ret = asr_dynamixel_get_present_position(&present_position);
	if (ret < 0) {
		LOG_ERR("读取舵机当前位置失败: %d", ret);
		return;
	}

	reply[2] = ASR_COMM_CMD_READ;
	reply[3] = ASR_COMM_PARAM_JOINT1;
	sys_put_le32((uint32_t)present_position, &reply[4]);

	ret = usb_control_send_payload(reply);
	if (ret < 0) {
		LOG_ERR("发送舵机位置回复失败: %d", ret);
		return;
	}

	LOG_INF("舵机当前位置=%ld", (long)present_position);
}

static int handle_imu_read(uint8_t buf[ASR_COMM_MSG_SIZE])
{
	ARG_UNUSED(buf);
	return -ENOTSUP;
}

static void usb_control_handle_payload(const uint8_t payload[ASR_COMM_MSG_SIZE])
{
	switch (payload[2]) {
	case ASR_COMM_CMD_WRITE:
		switch (payload[3]) {
		case ASR_COMM_PARAM_JOINT1_TORQUE:
			handle_dynamixel_torque(ASR_DXL_1, payload[4] != 0U);
			break;
		case ASR_COMM_PARAM_JOINT1:
			handle_dynamixel_goal_position(ASR_DXL_1,
						      (int32_t)sys_get_le32(&payload[4]));
			break;
		default:
			LOG_WRN("忽略 USB 未映射参数: param=0x%02x", payload[3]);
			break;
		}
		break;
	case ASR_COMM_CMD_READ:
		switch (payload[3]) {
		case ASR_COMM_PARAM_JOINT1:
			handle_dynamixel_position_read(ASR_DXL_1);
			break;
		default:
			LOG_WRN("忽略 USB 未映射读参数: param=0x%02x", payload[3]);
			break;
		}
		break;
	default:
		LOG_WRN("忽略 USB 未支持命令: cmd=0x%02x", payload[2]);
		break;
	}
}

static void usb_control_rx_cb(const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; ++i) {
		uint8_t byte = data[i];

		switch (usb_frame_parser.state) {
		case 0:
			if (byte == ASR_FRAME_SYNC_HI) {
				usb_frame_parser.state = 1U;
			}
			break;
		case 1:
			if (byte == ASR_FRAME_SYNC_LO) {
				usb_frame_parser.state = 2U;
			} else if (byte != ASR_FRAME_SYNC_HI) {
				usb_control_reset_parser();
			}
			break;
		case 2:
			if (byte != ASR_COMM_MSG_SIZE) {
				LOG_WRN("忽略 USB 非法帧长度: %u", (unsigned int)byte);
				usb_control_reset_parser();
				break;
			}

			usb_frame_parser.payload_len = byte;
			usb_frame_parser.checksum = byte;
			usb_frame_parser.payload_pos = 0U;
			usb_frame_parser.state = 3U;
			break;
		case 3:
			usb_frame_parser.payload[usb_frame_parser.payload_pos++] = byte;
			usb_frame_parser.checksum ^= byte;
			if (usb_frame_parser.payload_pos == usb_frame_parser.payload_len) {
				usb_frame_parser.state = 4U;
			}
			break;
		case 4:
			if (byte != usb_frame_parser.checksum) {
				LOG_WRN("忽略 USB 校验错误帧: expected=0x%02x got=0x%02x",
					usb_frame_parser.checksum, byte);
				usb_control_reset_parser();
				break;
			}

			usb_control_handle_payload(usb_frame_parser.payload);
			usb_control_reset_parser();
			break;
		default:
			usb_control_reset_parser();
			break;
		}
	}
}

static int dynamixel_prepare(void)
{
	uint16_t model_number = 0U;
	uint8_t firmware_version = 0U;
	uint8_t mode = 0U;
	uint8_t status_return_level = 0U;
	int32_t present_position = 0;
	int ret;

	memset(&dynamixel_state, 0, sizeof(dynamixel_state));
	usb_control_reset_parser();

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

	ret = asr_dynamixel_get_position_limits(&dynamixel_state.min_position,
						&dynamixel_state.max_position);
	if (ret < 0) {
		LOG_WRN("读取位置限位失败: %d", ret);
		return ret;
	}

	ret = asr_dynamixel_get_present_position(&present_position);
	if (ret < 0) {
		LOG_WRN("读取当前位置失败: %d", ret);
		return ret;
	}

	dynamixel_state.ready = true;
	LOG_INF("Dynamixel ready: pos=%ld, limit=[%ld, %ld]",
		(long)present_position,
		(long)dynamixel_state.min_position,
		(long)dynamixel_state.max_position);

	return 0;
}

static void handle_dynamixel_torque(uint8_t id, bool enable)
{
	int ret;

	if (id != ASR_DXL_1) {
		LOG_DBG("忽略未映射的舵机扭矩命令: joint=%u", id + 1U);
		return;
	}

	if (!dynamixel_state.ready) {
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

static void handle_dynamixel_goal_position(uint8_t id, int32_t goal_position)
{
	int ret;
	int32_t present_position;

	if (id != ASR_DXL_1) {
		LOG_DBG("忽略未映射的舵机位置命令: joint=%u", id + 1U);
		return;
	}

	if (!dynamixel_state.ready) {
		LOG_WRN("舵机未就绪, 忽略位置命令");
		return;
	}

	if (!dynamixel_goal_position_valid(goal_position)) {
		LOG_WRN("舵机目标位置越界: goal=%ld, limit=[%ld, %ld]",
			(long)goal_position,
			(long)dynamixel_state.min_position,
			(long)dynamixel_state.max_position);
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

int main(void)
{
	int ret;
	bool comm_ready = false;

	LOG_INF("==========================================");
	LOG_INF("Unit system application");
	LOG_INF("==========================================");

	ret = asr_led_thread_init();
	if (ret < 0) {
		LOG_WRN("LED thread init failed: %d (continuing without LED)", ret);
	}

	asr_usb_protocol_register_rx_cb(usb_control_rx_cb);

	ret = asr_usb_protocol_thread_init();
	if (ret < 0) {
		LOG_WRN("USB protocol thread init failed: %d (continuing without USB)", ret);
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

	ret = dynamixel_prepare();
	if (ret < 0) {
		LOG_WRN("Dynamixel bring-up 失败: %d (继续启动其余模块)", ret);
	}

	ret = asr_imu_thread_init();
	if (ret < 0) {
		LOG_WRN("IMU thread init failed: %d (continuing without IMU)", ret);
	}

	LOG_INF("all modules initialised, starting threads");
	k_sleep(K_MSEC(10));

	asr_led_thread_start();
	asr_usb_protocol_thread_start();
	asr_cpu_monitor_thread_start();
	if (comm_ready) {
		asr_comm_thread_start();
	}
	asr_imu_thread_start();

	LOG_INF("all background threads started");
	return 0;
}
