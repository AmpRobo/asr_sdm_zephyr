/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT asr_dynamixel_rs485

#include <asr/dynamixel.h>

#include "dynamixel_packet.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(asr_dynamixel, LOG_LEVEL_INF);

#define DXL_ADDR_DRIVE_MODE           10U
#define DXL_ADDR_OPERATING_MODE       11U
#define DXL_ADDR_MAX_POSITION_LIMIT   48U
#define DXL_ADDR_MIN_POSITION_LIMIT   52U
#define DXL_ADDR_TORQUE_ENABLE        64U
#define DXL_ADDR_STATUS_RETURN_LEVEL  68U
#define DXL_ADDR_HARDWARE_ERROR       70U
#define DXL_ADDR_PROFILE_ACCELERATION 108U
#define DXL_ADDR_PROFILE_VELOCITY     112U
#define DXL_ADDR_GOAL_POSITION        116U
#define DXL_ADDR_PRESENT_POSITION     132U

#define ASR_DXL_DEFAULT_BAUDRATE      57600U
#define ASR_DXL_RX_PACKET_SKIP_LIMIT  4U

struct asr_dynamixel_config {
	const struct device *uart;
	struct gpio_dt_spec dir_gpio;
	uint8_t servo_id;
	uint32_t baudrate;
	uint32_t reply_timeout_us;
	uint32_t turnaround_delay_us;
};

struct asr_dynamixel_data {
	struct k_mutex lock;
};

struct asr_dynamixel_driver_api {
	int (*ping)(const struct device *dev, uint16_t *model_number,
		    uint8_t *firmware_version);
	int (*get_operating_mode)(const struct device *dev, uint8_t *mode);
	int (*set_operating_mode)(const struct device *dev, uint8_t mode);
	int (*get_drive_mode)(const struct device *dev, uint8_t *mode);
	int (*set_drive_mode)(const struct device *dev, uint8_t mode);
	int (*set_torque)(const struct device *dev, bool enable);
	int (*get_torque)(const struct device *dev, bool *enabled);
	int (*get_status_return_level)(const struct device *dev, uint8_t *level);
	int (*set_status_return_level)(const struct device *dev, uint8_t level);
	int (*get_profile_acceleration)(const struct device *dev, uint32_t *acceleration);
	int (*set_profile_acceleration)(const struct device *dev, uint32_t acceleration);
	int (*get_profile_velocity)(const struct device *dev, uint32_t *velocity);
	int (*set_profile_velocity)(const struct device *dev, uint32_t velocity);
	int (*set_goal_position)(const struct device *dev, int32_t goal_position);
	int (*get_goal_position)(const struct device *dev, int32_t *goal_position);
	int (*read_control_table)(const struct device *dev, uint16_t address,
				  uint8_t *payload, size_t payload_len);
	int (*write_control_table)(const struct device *dev, uint16_t address,
				   const uint8_t *payload, size_t payload_len);
	int (*get_present_position)(const struct device *dev, int32_t *position);
	int (*get_hardware_error_status)(const struct device *dev, uint8_t *status);
	int (*get_position_limits)(const struct device *dev, int32_t *min_pos,
				   int32_t *max_pos);
};

BUILD_ASSERT(DT_HAS_COMPAT_STATUS_OKAY(asr_dynamixel_rs485),
	     "CONFIG_ASR_DYNAMIXEL requires an asr,dynamixel-rs485 node");

#define ASR_DYNAMIXEL_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(asr_dynamixel_rs485)

static const struct device *const asr_dynamixel_dev =
	DEVICE_DT_GET(ASR_DYNAMIXEL_NODE);

static int asr_dynamixel_set_direction(const struct asr_dynamixel_config *config,
				       int value)
{
	return gpio_pin_set_dt(&config->dir_gpio, value);
}

static const struct asr_dynamixel_driver_api *
asr_dynamixel_get_api(const struct device *dev)
{
	return (const struct asr_dynamixel_driver_api *)dev->api;
}

static void asr_dynamixel_flush_rx(const struct device *uart)
{
	unsigned char discard;

	while (uart_poll_in(uart, &discard) == 0) {
	}
}

static uint32_t asr_dynamixel_tx_time_us(const struct asr_dynamixel_config *config,
					 size_t bytes)
{
	uint64_t bits = (uint64_t)bytes * 10ULL * USEC_PER_SEC;

	return (uint32_t)((bits + config->baudrate - 1U) / config->baudrate);
}

static const char *asr_dynamixel_dir_mode_name(void)
{
	return "manual-dir";
}

static int asr_dynamixel_wait_byte(const struct asr_dynamixel_config *config,
				   uint8_t *byte)
{
	int64_t deadline_ms =
		k_uptime_get() +
		MAX(1, (int32_t)DIV_ROUND_UP(config->reply_timeout_us, 1000U));

	while (k_uptime_get() < deadline_ms) {
		if (uart_poll_in(config->uart, byte) == 0) {
			return 0;
		}

		k_busy_wait(50U);
	}

	return -ETIMEDOUT;
}

static int asr_dynamixel_receive_packet(const struct asr_dynamixel_config *config,
					uint8_t *buffer, size_t *buffer_len)
{
	uint8_t window[4] = {0};
	size_t seen = 0U;
	size_t pos = 0U;
	uint16_t length;
	uint8_t byte;
	int ret;

	while (true) {
		ret = asr_dynamixel_wait_byte(config, &byte);
		if (ret < 0) {
			if (ret == -ETIMEDOUT) {
				LOG_WRN("Dynamixel status header timeout: id=%u baud=%u timeout_us=%u mode=%s",
					(unsigned int)config->servo_id,
					(unsigned int)config->baudrate,
					(unsigned int)config->reply_timeout_us,
					asr_dynamixel_dir_mode_name());
			}
			return ret;
		}

		if (seen < ARRAY_SIZE(window)) {
			window[seen++] = byte;
		} else {
			memmove(&window[0], &window[1], ARRAY_SIZE(window) - 1U);
			window[ARRAY_SIZE(window) - 1U] = byte;
		}

		if ((seen == ARRAY_SIZE(window)) &&
		    (window[0] == 0xFFU) &&
		    (window[1] == 0xFFU) &&
		    (window[2] == 0xFDU) &&
		    (window[3] == 0x00U)) {
			memcpy(buffer, window, ARRAY_SIZE(window));
			break;
		}
	}

	pos = 4U;
	while (pos < 7U) {
		ret = asr_dynamixel_wait_byte(config, &buffer[pos]);
		if (ret < 0) {
			if (ret == -ETIMEDOUT) {
				LOG_WRN("Dynamixel status length timeout after header: id=%u pos=%u",
					(unsigned int)config->servo_id,
					(unsigned int)pos);
			}
			return ret;
		}
		pos++;
	}

	length = sys_get_le16(&buffer[5]);
	if ((7U + length) > ASR_DYNAMIXEL_MAX_PACKET_SIZE) {
		return -EMSGSIZE;
	}

	while (pos < (size_t)(7U + length)) {
		ret = asr_dynamixel_wait_byte(config, &buffer[pos]);
		if (ret < 0) {
			if (ret == -ETIMEDOUT) {
				LOG_WRN("Dynamixel status payload timeout: id=%u expected=%u received=%u",
					(unsigned int)config->servo_id,
					(unsigned int)(7U + length),
					(unsigned int)pos);
			}
			return ret;
		}
		pos++;
	}

	*buffer_len = pos;
	return 0;
}

static int asr_dynamixel_receive_status(const struct asr_dynamixel_config *config,
						uint8_t *buffer, size_t *buffer_len)
{
	struct asr_dynamixel_status_packet status;
	uint32_t skipped_packets = 0U;
	int ret;

	while (true) {
		ret = asr_dynamixel_receive_packet(config, buffer, buffer_len);
		if (ret < 0) {
			return ret;
		}

		ret = asr_dynamixel_parse_status_packet(buffer, *buffer_len, &status);
		if (ret == 0) {
			return 0;
		}

		if (ret != -EPROTO) {
			return ret;
		}

		skipped_packets++;
		LOG_DBG("跳过非 Status 包或本机回显: count=%u len=%u",
			(unsigned int)skipped_packets, (unsigned int)*buffer_len);
		if (skipped_packets >= ASR_DXL_RX_PACKET_SKIP_LIMIT) {
			return ret;
		}
	}
}

static int asr_dynamixel_transaction(const struct device *dev, uint8_t instruction,
				     const uint8_t *params, size_t param_len,
				     bool expect_status,
				     struct asr_dynamixel_status_packet *status)
{
	const struct asr_dynamixel_config *config = dev->config;
	struct asr_dynamixel_data *data = dev->data;
	uint8_t tx_buffer[ASR_DYNAMIXEL_MAX_PACKET_SIZE];
	uint8_t rx_buffer[ASR_DYNAMIXEL_MAX_PACKET_SIZE];
	size_t tx_len;
	size_t rx_len;
	int ret;

	tx_len = asr_dynamixel_build_instruction_packet(config->servo_id, instruction,
							params, param_len,
							tx_buffer,
							sizeof(tx_buffer));
	if (tx_len == 0U) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	asr_dynamixel_flush_rx(config->uart);

	ret = asr_dynamixel_set_direction(config, 1);
	if (ret < 0) {
		goto out_unlock;
	}

	for (size_t i = 0U; i < tx_len; ++i) {
		uart_poll_out(config->uart, tx_buffer[i]);
	}

	k_busy_wait(asr_dynamixel_tx_time_us(config, tx_len));

	ret = asr_dynamixel_set_direction(config, 0);
	if (ret < 0) {
		goto out_unlock;
	}

	k_busy_wait(config->turnaround_delay_us);

	if (!expect_status) {
		ret = 0;
		goto out_unlock;
	}

	ret = asr_dynamixel_receive_status(config, rx_buffer, &rx_len);
	if (ret < 0) {
		if (ret == -ETIMEDOUT) {
			LOG_WRN("Dynamixel instruction 0x%02x timed out waiting for reply: id=%u tx_len=%u turnaround_us=%u",
				(unsigned int)instruction,
				(unsigned int)config->servo_id,
				(unsigned int)tx_len,
				(unsigned int)config->turnaround_delay_us);
		}
		goto out_unlock;
	}

	ret = asr_dynamixel_parse_status_packet(rx_buffer, rx_len, status);
	if (ret < 0) {
		goto out_unlock;
	}

	if (status->id != config->servo_id) {
		ret = -ENOMSG;
		goto out_unlock;
	}

	if (status->alert) {
		LOG_WRN("舵机返回硬件告警位, error=0x%02x", status->error);
	}

	ret = asr_dynamixel_map_status_error(status->error);

out_unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int asr_dynamixel_read_u8(const struct device *dev, uint16_t address,
				 uint8_t *value)
{
	struct asr_dynamixel_status_packet status = {0};
	uint8_t params[4];
	int ret;

	if (value == NULL) {
		return -EINVAL;
	}

	sys_put_le16(address, &params[0]);
	sys_put_le16(1U, &params[2]);

	ret = asr_dynamixel_transaction(dev, ASR_DYNAMIXEL_INST_READ, params,
					sizeof(params), true, &status);
	if (ret < 0) {
		return ret;
	}

	if (status.param_len != 1U) {
		return -EMSGSIZE;
	}

	*value = status.params[0];
	return 0;
}

static int asr_dynamixel_read_control(const struct device *dev, uint16_t address,
					    uint8_t *payload, size_t payload_len)
{
	struct asr_dynamixel_status_packet status = {0};
	uint8_t params[4];
	int ret;

	if ((payload == NULL) || (payload_len > ASR_DYNAMIXEL_MAX_PARAMS)) {
		return -EINVAL;
	}

	sys_put_le16(address, &params[0]);
	sys_put_le16((uint16_t)payload_len, &params[2]);

	ret = asr_dynamixel_transaction(dev, ASR_DYNAMIXEL_INST_READ, params,
					sizeof(params), true, &status);
	if (ret < 0) {
		return ret;
	}

	if (status.param_len != payload_len) {
		return -EMSGSIZE;
	}

	memcpy(payload, status.params, payload_len);
	return 0;
}

static int asr_dynamixel_read_le32(const struct device *dev, uint16_t address,
				   int32_t *value)
{
	struct asr_dynamixel_status_packet status = {0};
	uint8_t params[4];
	int ret;

	if (value == NULL) {
		return -EINVAL;
	}

	sys_put_le16(address, &params[0]);
	sys_put_le16(4U, &params[2]);

	ret = asr_dynamixel_transaction(dev, ASR_DYNAMIXEL_INST_READ, params,
					sizeof(params), true, &status);
	if (ret < 0) {
		return ret;
	}

	if (status.param_len != 4U) {
		return -EMSGSIZE;
	}

	*value = (int32_t)sys_get_le32(status.params);
	return 0;
}

static int asr_dynamixel_write(const struct device *dev, uint16_t address,
			       const uint8_t *payload, size_t payload_len,
			       bool expect_status)
{
	struct asr_dynamixel_status_packet status = {0};
	uint8_t params[2U + ASR_DYNAMIXEL_MAX_PARAMS];

	if ((payload == NULL) || (payload_len > ASR_DYNAMIXEL_MAX_PARAMS)) {
		return -EINVAL;
	}

	sys_put_le16(address, &params[0]);
	memcpy(&params[2], payload, payload_len);

	return asr_dynamixel_transaction(dev, ASR_DYNAMIXEL_INST_WRITE, params,
					 payload_len + 2U, expect_status,
					 expect_status ? &status : NULL);
}

static int asr_dynamixel_dev_ping(const struct device *dev,
				  uint16_t *model_number,
				  uint8_t *firmware_version)
{
	struct asr_dynamixel_status_packet status = {0};
	int ret;

	ret = asr_dynamixel_transaction(dev, ASR_DYNAMIXEL_INST_PING, NULL, 0U,
					true, &status);
	if (ret < 0) {
		return ret;
	}

	if (status.param_len != 3U) {
		return -EMSGSIZE;
	}

	if (model_number != NULL) {
		*model_number = sys_get_le16(status.params);
	}

	if (firmware_version != NULL) {
		*firmware_version = status.params[2];
	}

	return 0;
}

static int asr_dynamixel_dev_get_operating_mode(const struct device *dev,
						uint8_t *mode)
{
	return asr_dynamixel_read_u8(dev, DXL_ADDR_OPERATING_MODE, mode);
}

static int asr_dynamixel_dev_set_operating_mode(const struct device *dev,
						uint8_t mode)
{
	return asr_dynamixel_write(dev, DXL_ADDR_OPERATING_MODE, &mode,
				   sizeof(mode), true);
}

static int asr_dynamixel_dev_get_drive_mode(const struct device *dev, uint8_t *mode)
{
	return asr_dynamixel_read_u8(dev, DXL_ADDR_DRIVE_MODE, mode);
}

static int asr_dynamixel_dev_set_drive_mode(const struct device *dev, uint8_t mode)
{
	return asr_dynamixel_write(dev, DXL_ADDR_DRIVE_MODE, &mode, sizeof(mode), true);
}

static int asr_dynamixel_dev_set_torque(const struct device *dev, bool enable)
{
	uint8_t value = enable ? 1U : 0U;

	return asr_dynamixel_write(dev, DXL_ADDR_TORQUE_ENABLE, &value,
				   sizeof(value), true);
}

static int asr_dynamixel_dev_get_torque(const struct device *dev, bool *enabled)
{
	uint8_t value;
	int ret;

	if (enabled == NULL) {
		return -EINVAL;
	}

	ret = asr_dynamixel_read_u8(dev, DXL_ADDR_TORQUE_ENABLE, &value);
	if (ret < 0) {
		return ret;
	}

	*enabled = value != 0U;
	return 0;
}

static int asr_dynamixel_dev_get_status_return_level(const struct device *dev,
						     uint8_t *level)
{
	return asr_dynamixel_read_u8(dev, DXL_ADDR_STATUS_RETURN_LEVEL, level);
}

static int asr_dynamixel_dev_set_status_return_level(const struct device *dev,
						     uint8_t level)
{
	uint8_t actual_level;
	int ret;

	if (level > 2U) {
		return -ERANGE;
	}

	if (level == 2U) {
		ret = asr_dynamixel_write(dev, DXL_ADDR_STATUS_RETURN_LEVEL, &level,
					  sizeof(level), false);
		if (ret < 0) {
			return ret;
		}

		k_busy_wait(1000U);
		ret = asr_dynamixel_read_u8(dev, DXL_ADDR_STATUS_RETURN_LEVEL,
					    &actual_level);
		if (ret < 0) {
			return ret;
		}

		return (actual_level == level) ? 0 : -EIO;
	}

	return asr_dynamixel_write(dev, DXL_ADDR_STATUS_RETURN_LEVEL, &level,
				   sizeof(level), true);
}

static int asr_dynamixel_dev_get_profile_acceleration(const struct device *dev,
						       uint32_t *acceleration)
{
	int32_t value;
	int ret;

	if (acceleration == NULL) {
		return -EINVAL;
	}

	ret = asr_dynamixel_read_le32(dev, DXL_ADDR_PROFILE_ACCELERATION, &value);
	if (ret < 0) {
		return ret;
	}

	*acceleration = (uint32_t)value;
	return 0;
}

static int asr_dynamixel_dev_set_profile_acceleration(const struct device *dev,
						       uint32_t acceleration)
{
	uint8_t payload[4];

	sys_put_le32(acceleration, payload);
	return asr_dynamixel_write(dev, DXL_ADDR_PROFILE_ACCELERATION, payload,
					   sizeof(payload), true);
}

static int asr_dynamixel_dev_get_profile_velocity(const struct device *dev,
						   uint32_t *velocity)
{
	int32_t value;
	int ret;

	if (velocity == NULL) {
		return -EINVAL;
	}

	ret = asr_dynamixel_read_le32(dev, DXL_ADDR_PROFILE_VELOCITY, &value);
	if (ret < 0) {
		return ret;
	}

	*velocity = (uint32_t)value;
	return 0;
}

static int asr_dynamixel_dev_set_profile_velocity(const struct device *dev,
						   uint32_t velocity)
{
	uint8_t payload[4];

	sys_put_le32(velocity, payload);
	return asr_dynamixel_write(dev, DXL_ADDR_PROFILE_VELOCITY, payload,
					   sizeof(payload), true);
}

static int asr_dynamixel_dev_set_goal_position(const struct device *dev,
					       int32_t goal_position)
{
	uint8_t payload[4];

	sys_put_le32((uint32_t)goal_position, payload);
	return asr_dynamixel_write(dev, DXL_ADDR_GOAL_POSITION, payload,
				   sizeof(payload), true);
}

static int asr_dynamixel_dev_get_goal_position(const struct device *dev,
					       int32_t *goal_position)
{
	return asr_dynamixel_read_le32(dev, DXL_ADDR_GOAL_POSITION, goal_position);
}

static int asr_dynamixel_dev_read_control_table(const struct device *dev,
					       uint16_t address, uint8_t *payload,
					       size_t payload_len)
{
	return asr_dynamixel_read_control(dev, address, payload, payload_len);
}

static int asr_dynamixel_dev_write_control_table(const struct device *dev,
						uint16_t address, const uint8_t *payload,
						size_t payload_len)
{
	return asr_dynamixel_write(dev, address, payload, payload_len, true);
}

static int asr_dynamixel_dev_get_present_position(const struct device *dev,
						  int32_t *position)
{
	return asr_dynamixel_read_le32(dev, DXL_ADDR_PRESENT_POSITION, position);
}

static int asr_dynamixel_dev_get_hardware_error_status(const struct device *dev,
						       uint8_t *status)
{
	return asr_dynamixel_read_u8(dev, DXL_ADDR_HARDWARE_ERROR, status);
}

static int asr_dynamixel_dev_get_position_limits(const struct device *dev,
						 int32_t *min_pos,
						 int32_t *max_pos)
{
	int ret;

	if ((min_pos == NULL) || (max_pos == NULL)) {
		return -EINVAL;
	}

	ret = asr_dynamixel_read_le32(dev, DXL_ADDR_MIN_POSITION_LIMIT, min_pos);
	if (ret < 0) {
		return ret;
	}

	ret = asr_dynamixel_read_le32(dev, DXL_ADDR_MAX_POSITION_LIMIT, max_pos);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int asr_dynamixel_device_init(const struct device *dev)
{
	const struct asr_dynamixel_config *config = dev->config;
	struct asr_dynamixel_data *data = dev->data;
	const struct uart_config uart_cfg = {
		.baudrate = config->baudrate,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};
	int ret;

	if (!device_is_ready(config->uart)) {
		LOG_ERR("Dynamixel UART 未就绪");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->dir_gpio)) {
		LOG_ERR("Dynamixel 方向 GPIO 未就绪");
		return -ENODEV;
	}

	if (config->baudrate == 0U) {
		return -EINVAL;
	}

	k_mutex_init(&data->lock);

	ret = uart_configure(config->uart, &uart_cfg);
	if (ret < 0) {
		LOG_ERR("Dynamixel UART 配置失败: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&config->dir_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("Dynamixel bus on %s @ %u, dir=%s pin=%u",
		config->uart->name,
		(unsigned int)config->baudrate,
		config->dir_gpio.port->name,
		(unsigned int)config->dir_gpio.pin);

	return 0;
}

static const struct asr_dynamixel_driver_api asr_dynamixel_api = {
	.ping = asr_dynamixel_dev_ping,
	.get_operating_mode = asr_dynamixel_dev_get_operating_mode,
	.set_operating_mode = asr_dynamixel_dev_set_operating_mode,
	.get_drive_mode = asr_dynamixel_dev_get_drive_mode,
	.set_drive_mode = asr_dynamixel_dev_set_drive_mode,
	.set_torque = asr_dynamixel_dev_set_torque,
	.get_torque = asr_dynamixel_dev_get_torque,
	.get_profile_acceleration = asr_dynamixel_dev_get_profile_acceleration,
	.set_profile_acceleration = asr_dynamixel_dev_set_profile_acceleration,
	.get_profile_velocity = asr_dynamixel_dev_get_profile_velocity,
	.set_profile_velocity = asr_dynamixel_dev_set_profile_velocity,
	.set_goal_position = asr_dynamixel_dev_set_goal_position,
	.get_goal_position = asr_dynamixel_dev_get_goal_position,
	.read_control_table = asr_dynamixel_dev_read_control_table,
	.write_control_table = asr_dynamixel_dev_write_control_table,
	.get_status_return_level = asr_dynamixel_dev_get_status_return_level,
	.set_status_return_level = asr_dynamixel_dev_set_status_return_level,
	.get_hardware_error_status = asr_dynamixel_dev_get_hardware_error_status,
	.get_present_position = asr_dynamixel_dev_get_present_position,
	.get_position_limits = asr_dynamixel_dev_get_position_limits,
};

#define ASR_DYNAMIXEL_DEFINE(inst)                                                 \
	static struct asr_dynamixel_data asr_dynamixel_data_##inst;                \
	static const struct asr_dynamixel_config asr_dynamixel_config_##inst = {   \
		.uart = DEVICE_DT_GET(DT_INST_PARENT(inst)),                        \
		.dir_gpio = GPIO_DT_SPEC_INST_GET(inst, dir_gpios),                 \
		.servo_id = DT_INST_PROP(inst, servo_id),                           \
		.baudrate = DT_PROP_OR(DT_INST_PARENT(inst), current_speed,         \
				      ASR_DXL_DEFAULT_BAUDRATE),                      \
		.reply_timeout_us = DT_INST_PROP(inst, reply_timeout_us),           \
		.turnaround_delay_us = DT_INST_PROP(inst, turnaround_delay_us),     \
	};                                                                         \
	DEVICE_DT_INST_DEFINE(inst, asr_dynamixel_device_init, NULL,                \
			      &asr_dynamixel_data_##inst,                          \
			      &asr_dynamixel_config_##inst, POST_KERNEL,           \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                \
			      &asr_dynamixel_api)

DT_INST_FOREACH_STATUS_OKAY(ASR_DYNAMIXEL_DEFINE);

int asr_dynamixel_init(void)
{
	return device_is_ready(asr_dynamixel_dev) ? 0 : -ENODEV;
}

int asr_dynamixel_ping(uint16_t *model_number, uint8_t *firmware_version)
{
	const struct asr_dynamixel_driver_api *api = asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->ping(asr_dynamixel_dev, model_number, firmware_version);
}

int asr_dynamixel_get_operating_mode(uint8_t *mode)
{
	const struct asr_dynamixel_driver_api *api =
		asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->get_operating_mode(asr_dynamixel_dev, mode);
}

int asr_dynamixel_set_operating_mode(uint8_t mode)
{
	const struct asr_dynamixel_driver_api *api =
		asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->set_operating_mode(asr_dynamixel_dev, mode);
}

int asr_dynamixel_get_drive_mode(uint8_t *mode)
{
	const struct asr_dynamixel_driver_api *api =
		asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->get_drive_mode(asr_dynamixel_dev, mode);
}

int asr_dynamixel_set_drive_mode(uint8_t mode)
{
	const struct asr_dynamixel_driver_api *api =
		asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->set_drive_mode(asr_dynamixel_dev, mode);
}

int asr_dynamixel_set_torque(bool enable)
{
	const struct asr_dynamixel_driver_api *api = asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->set_torque(asr_dynamixel_dev, enable);
}

int asr_dynamixel_get_torque(bool *enabled)
{
	const struct asr_dynamixel_driver_api *api =
		asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->get_torque(asr_dynamixel_dev, enabled);
}

int asr_dynamixel_get_status_return_level(uint8_t *level)
{
	const struct asr_dynamixel_driver_api *api =
		asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->get_status_return_level(asr_dynamixel_dev, level);
}

int asr_dynamixel_set_status_return_level(uint8_t level)
{
	const struct asr_dynamixel_driver_api *api =
		asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->set_status_return_level(asr_dynamixel_dev, level);
}

int asr_dynamixel_get_profile_acceleration(uint32_t *acceleration)
{
	const struct asr_dynamixel_driver_api *api =
		asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->get_profile_acceleration(asr_dynamixel_dev, acceleration);
}

int asr_dynamixel_set_profile_acceleration(uint32_t acceleration)
{
	const struct asr_dynamixel_driver_api *api =
		asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->set_profile_acceleration(asr_dynamixel_dev, acceleration);
}

int asr_dynamixel_get_profile_velocity(uint32_t *velocity)
{
	const struct asr_dynamixel_driver_api *api =
		asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->get_profile_velocity(asr_dynamixel_dev, velocity);
}

int asr_dynamixel_set_profile_velocity(uint32_t velocity)
{
	const struct asr_dynamixel_driver_api *api =
		asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->set_profile_velocity(asr_dynamixel_dev, velocity);
}

int asr_dynamixel_set_goal_position(int32_t goal_position)
{
	const struct asr_dynamixel_driver_api *api = asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->set_goal_position(asr_dynamixel_dev, goal_position);
}

int asr_dynamixel_get_goal_position(int32_t *goal_position)
{
	const struct asr_dynamixel_driver_api *api =
		asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->get_goal_position(asr_dynamixel_dev, goal_position);
}

int asr_dynamixel_read_control_table(uint16_t address, uint8_t *data, size_t length)
{
	const struct asr_dynamixel_driver_api *api =
		asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->read_control_table(asr_dynamixel_dev, address, data, length);
}

int asr_dynamixel_write_control_table(uint16_t address, const uint8_t *data,
				      size_t length)
{
	const struct asr_dynamixel_driver_api *api =
		asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->write_control_table(asr_dynamixel_dev, address, data, length);
}

int asr_dynamixel_get_present_position(int32_t *position)
{
	const struct asr_dynamixel_driver_api *api = asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->get_present_position(asr_dynamixel_dev, position);
}

int asr_dynamixel_get_hardware_error_status(uint8_t *status)
{
	const struct asr_dynamixel_driver_api *api = asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->get_hardware_error_status(asr_dynamixel_dev, status);
}

int asr_dynamixel_get_position_limits(int32_t *min_pos, int32_t *max_pos)
{
	const struct asr_dynamixel_driver_api *api =
		asr_dynamixel_get_api(asr_dynamixel_dev);

	if (!device_is_ready(asr_dynamixel_dev)) {
		return -ENODEV;
	}

	return api->get_position_limits(asr_dynamixel_dev, min_pos, max_pos);
}
