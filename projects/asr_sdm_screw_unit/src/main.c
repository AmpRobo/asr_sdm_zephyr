/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * The application delegates LED, IMU, Dynamixel servo, and UART communication
 * activity to the ASR helper modules. Dynamixel bring-up and command handling
 * live in the dynamixel module (see asr/dynamixel.h and
 * asr/dynamixel_thread.h); main only wires the comm callbacks and dispatches
 * USB control frames.
 */

#include <asr/comm_thread.h>
#include <asr/cpu_monitor_thread.h>
#include <asr/dynamixel.h>
#include <asr/dynamixel_thread.h>
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

#define ASR_FRAME_SYNC_HI 0xAAU
#define ASR_FRAME_SYNC_LO 0x55U

unit_status_t unit_status;
K_MUTEX_DEFINE(unit_status_mutex);

static struct {
	uint8_t state;
	uint8_t payload_len;
	uint8_t payload_pos;
	uint8_t checksum;
	uint8_t payload[ASR_COMM_MSG_SIZE];
} usb_frame_parser;

static int handle_imu_read(uint8_t buf[ASR_COMM_MSG_SIZE]);

static const struct asr_comm_callbacks comm_callbacks = {
	.on_dynamixel_torque = asr_dynamixel_app_handle_torque,
	.on_dynamixel_goal_position = asr_dynamixel_app_handle_goal_position,
	.on_imu_read = handle_imu_read,
};

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

static int handle_imu_read(uint8_t buf[ASR_COMM_MSG_SIZE])
{
	ARG_UNUSED(buf);
	return -ENOTSUP;
}

static void usb_control_send_joint1_position_reply(void)
{
	uint8_t reply[ASR_COMM_MSG_SIZE] = {0};
	int32_t present_position;
	int ret;

	ret = asr_dynamixel_app_handle_position_read(ASR_DXL_1, &present_position);
	if (ret < 0) {
		return;
	}

	reply[2] = ASR_COMM_CMD_READ;
	reply[3] = ASR_COMM_PARAM_JOINT1;
	sys_put_le32((uint32_t)present_position, &reply[4]);

	ret = usb_control_send_payload(reply);
	if (ret < 0) {
		LOG_ERR("发送舵机位置回复失败: %d", ret);
	}
}

static void usb_control_handle_payload(const uint8_t payload[ASR_COMM_MSG_SIZE])
{
	switch (payload[2]) {
	case ASR_COMM_CMD_WRITE:
		switch (payload[3]) {
		case ASR_COMM_PARAM_JOINT1_TORQUE:
			asr_dynamixel_app_handle_torque(ASR_DXL_1, payload[4] != 0U);
			break;
		case ASR_COMM_PARAM_JOINT1:
			asr_dynamixel_app_handle_goal_position(
				ASR_DXL_1, (int32_t)sys_get_le32(&payload[4]));
			break;
		default:
			LOG_WRN("忽略 USB 未映射参数: param=0x%02x", payload[3]);
			break;
		}
		break;
	case ASR_COMM_CMD_READ:
		switch (payload[3]) {
		case ASR_COMM_PARAM_JOINT1:
			usb_control_send_joint1_position_reply();
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
	usb_control_reset_parser();

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

	ret = asr_dynamixel_thread_init();
	if (ret < 0) {
		LOG_WRN("Dynamixel thread init failed: %d (continuing without Dynamixel)", ret);
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
	asr_dynamixel_thread_start();
	asr_imu_thread_start();

	LOG_INF("all background threads started");
	return 0;
}
