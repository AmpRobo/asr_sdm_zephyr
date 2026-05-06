/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART0 communication module for the ASR SDM protocol.
 *
 * Transports 8-byte protocol messages over UART0 using a lightweight framing
 * layer.  Hardware-specific actions (flash, LED, motor, dynamixel) are
 * dispatched through an optional callback table registered by the application.
 *
 * Frame format (on the wire):
 *
 *   [0xAA] [0x55] [LEN] [DATA_0 .. DATA_N-1] [CHECKSUM]
 *
 *   CHECKSUM = LEN ^ DATA_0 ^ DATA_1 ^ ... ^ DATA_N-1
 */

#ifndef ASR_COMM_THREAD_H_
#define ASR_COMM_THREAD_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ASR_COMM_MSG_SIZE 8

/* Command types (byte[2] of protocol message) */
#define ASR_COMM_CMD_READ  0x02
#define ASR_COMM_CMD_WRITE 0x03

/* Parameter IDs (byte[3] of protocol message) */
#define ASR_COMM_PARAM_BOARD_ID      0x01
#define ASR_COMM_PARAM_CAN_ID        0x02
#define ASR_COMM_PARAM_LED_ENABLE    0x03
#define ASR_COMM_PARAM_LED_STATUS    0x04
#define ASR_COMM_PARAM_MOTOR         0x05
#define ASR_COMM_PARAM_JOINT1        0x06
#define ASR_COMM_PARAM_JOINT2        0x07
#define ASR_COMM_PARAM_JOINT1_TORQUE 0x08
#define ASR_COMM_PARAM_JOINT2_TORQUE 0x09
#define ASR_COMM_PARAM_IMU          0x0A

/* Dynamixel identifiers (0-based) */
#define ASR_DXL_1 0
#define ASR_DXL_2 1

/**
 * Hardware-action callbacks.  Set unused entries to NULL.
 * The structure must remain valid for the lifetime of the module.
 */
struct asr_comm_callbacks {
	void (*on_flash_write)(const uint8_t *data, size_t len);
	void (*on_led_write)(bool state);
	void (*on_motor_set)(uint8_t channel, uint8_t value);
	void (*on_dynamixel_torque)(uint8_t id, bool enable);
	void (*on_dynamixel_goal_position)(uint8_t id, int32_t goal_position);
	/**
	 * Read IMU data into a response buffer.
	 * @param buf  Output buffer of exactly @ref ASR_COMM_MSG_SIZE bytes.
	 * @return 0 on success, negative errno on failure.
	 */
	int (*on_imu_read)(uint8_t buf[ASR_COMM_MSG_SIZE]);
};

/**
 * Persistent unit state managed by the protocol layer.
 */
typedef struct {
	uint8_t flash_data[2];
	uint16_t unit_id;

	bool led_enable;
	bool led_status;

	uint8_t cmd_motor[2];
	uint8_t cmd_joint1[4];
	uint8_t cmd_joint2[4];
	bool dynamixel_enable[2];
} asr_unit_status_t;

/**
 * Register hardware-action callbacks invoked by the protocol handler.
 *
 * @param cb  Pointer to a callback structure (may be NULL to clear).
 */
void asr_comm_register_callbacks(const struct asr_comm_callbacks *cb);

/**
 * Initialise UART and create the comm thread in suspended state.
 *
 * @return 0 on success, -EALREADY if already initialised, negative errno
 *         otherwise.
 */
int asr_comm_thread_init(void);

/**
 * Start the comm thread (k_thread_start; must call asr_comm_thread_init first).
 *
 * @return 0 on success, -EINVAL if not initialised.
 */
int asr_comm_thread_start(void);

/**
 * Transmit an 8-byte protocol message (blocking, poll-based).
 *
 * @param data  Pointer to exactly @ref ASR_COMM_MSG_SIZE bytes.
 * @return 0 on success, negative errno on failure.
 */
int asr_comm_send(const uint8_t data[ASR_COMM_MSG_SIZE]);

/**
 * Get a read-only snapshot of the current unit status.
 */
const asr_unit_status_t *asr_comm_get_status(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_COMM_THREAD_H_ */
