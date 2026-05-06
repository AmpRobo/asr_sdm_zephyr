/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ASR_DYNAMIXEL_H_
#define ASR_DYNAMIXEL_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Validate that the configured Dynamixel bus device is ready.
 *
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_init(void);

/**
 * Ping the configured single-servo Dynamixel target.
 *
 * @param model_number      Optional output for the servo model number.
 * @param firmware_version  Optional output for the firmware version.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_ping(uint16_t *model_number, uint8_t *firmware_version);

/**
 * Read the servo Operating Mode(11).
 *
 * @param mode  Output operating mode value.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_get_operating_mode(uint8_t *mode);

/**
 * Write the servo Operating Mode(11).
 *
 * @param mode  Raw operating mode value.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_set_operating_mode(uint8_t mode);

/**
 * Set the servo torque enable state.
 *
 * @param enable  True to enable torque, false to disable it.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_set_torque(bool enable);

/**
 * Read the servo Status Return Level(68).
 *
 * @param level  Output status return level.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_get_status_return_level(uint8_t *level);

/**
 * Write the servo Status Return Level(68).
 *
 * @param level  Status return level value.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_set_status_return_level(uint8_t level);

/**
 * Write the servo goal position using Dynamixel Protocol 2.0.
 *
 * @param goal_position  Raw Goal Position(116) value.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_set_goal_position(int32_t goal_position);

/**
 * Read the servo Present Position(132).
 *
 * @param position  Output current position value.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_get_present_position(int32_t *position);

/**
 * Read the servo Hardware Error Status(70).
 *
 * @param status  Output hardware error bitfield.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_get_hardware_error_status(uint8_t *status);

/**
 * Read the servo Min/Max Position Limit(52/48).
 *
 * @param min_pos  Output minimum position limit.
 * @param max_pos  Output maximum position limit.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_get_position_limits(int32_t *min_pos, int32_t *max_pos);

#ifdef __cplusplus
}
#endif

#endif /* ASR_DYNAMIXEL_H_ */
