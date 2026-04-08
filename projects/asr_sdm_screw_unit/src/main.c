/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ICM-42688 sensor demo application
 * for Seeed XIAO RP2350 development board
 *
 * Function: read accelerometer, gyroscope, and temperature data from ICM-42688
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(icm42688_demo, LOG_LEVEL_INF);

/* SPI mode: use the ICM-42688 node from devicetree */
#define ICM42688_NODE DT_NODELABEL(icm42688)

#if !DT_NODE_EXISTS(ICM42688_NODE)
#error "设备树中未找到 icm42688 节点，请检查 overlay 文件"
#endif

int main(void)
{
	const struct device *usb_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	const struct device *icm = DEVICE_DT_GET(ICM42688_NODE);
	struct sensor_value accel[3], gyro[3], temp;
	uint32_t dtr = 0;
	int ret;

	/* Enable USB device */
	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("USB 启用失败: %d", ret);
		return ret;
	}

	/* Wait for USB CDC ACM connection (DTR signal) */
	LOG_INF("等待 USB 串口连接...");
	while (!dtr) {
		uart_line_ctrl_get(usb_dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}

	/* Short delay to stabilize the serial connection */
	k_sleep(K_MSEC(500));

	/* SPI mode: check that the sensor device is ready */
	if (!device_is_ready(icm)) {
		LOG_ERR("ICM-42688 设备未就绪");
		return -ENODEV;
	}

	LOG_INF("==========================================");
	LOG_INF("ICM-42688 传感器示例应用");
	LOG_INF("==========================================");
	LOG_INF("当前模式: SPI (Zephyr Sensor Driver)");
	LOG_INF("设备已就绪，开始采集数据...");
	LOG_INF("");

	while (1) {
		/* Fetch sensor sample */
		ret = sensor_sample_fetch(icm);
		if (ret < 0) {
			LOG_ERR("传感器数据采集失败: %d", ret);
			k_sleep(K_MSEC(1000));
			continue;
		}

		/* Read accelerometer data (unit: m/s²) */
		sensor_channel_get(icm, SENSOR_CHAN_ACCEL_XYZ, accel);

		/* Read gyroscope data (unit: rad/s) */
		sensor_channel_get(icm, SENSOR_CHAN_GYRO_XYZ, gyro);

		/* Read die temperature (unit: °C) */
		sensor_channel_get(icm, SENSOR_CHAN_DIE_TEMP, &temp);

		/* Print accelerometer data */
		LOG_INF("加速度 [m/s²]: X=%10.4f  Y=%10.4f  Z=%10.4f",
			sensor_value_to_double(&accel[0]),
			sensor_value_to_double(&accel[1]),
			sensor_value_to_double(&accel[2]));

		/* Print gyroscope data */
		LOG_INF("角速度 [rad/s]: X=%10.4f  Y=%10.4f  Z=%10.4f",
			sensor_value_to_double(&gyro[0]),
			sensor_value_to_double(&gyro[1]),
			sensor_value_to_double(&gyro[2]));

		/* Print temperature */
		LOG_INF("芯片温度: %.1f °C", sensor_value_to_double(&temp));

		LOG_INF("------------------------------------------");

		/* Sample once per second */
		k_sleep(K_MSEC(1000));
	}

	return 0;
}
