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

/* -------------------------------------------------------------------------
 * Application-level helpers (cached bring-up state + comm-callback wrappers)
 *
 * These wrap the lower-level driver above and add readiness/range checking
 * suitable for the screw-unit application. The bring-up is normally driven by
 * the Dynamixel thread (see asr/dynamixel_thread.h); the handlers below match
 * the asr_comm_callbacks signatures so they can be registered directly.
 * -------------------------------------------------------------------------*/

/**
 * Run the Dynamixel bring-up sequence: ping the servo, validate the model and
 * mode, fetch position limits, and cache the current position. Updates the
 * internal state used by the handlers below.
 *
 * @return 0 on success, negative errno on failure.
 */
int asr_dynamixel_app_init(void);

/** True after a successful @ref asr_dynamixel_app_init. */
bool asr_dynamixel_app_is_ready(void);

/**
 * Comm-callback: enable/disable torque on the primary servo. Performs
 * readiness checks and ignores commands targeting unmapped joints.
 */
void asr_dynamixel_app_handle_torque(uint8_t id, bool enable);

/**
 * Comm-callback: send a goal position to the primary servo. The goal is
 * range-checked against the cached position limits.
 */
void asr_dynamixel_app_handle_goal_position(uint8_t id, int32_t goal_position);

/**
 * Read the present position of the primary servo. Performs the same
 * readiness/mapping checks as the comm-callback handlers and logs the value
 * on success. Intended for the USB control read-back path.
 *
 * @param id        Joint index (only the primary servo is mapped).
 * @param position  Output present position; only written on success.
 * @return 0 on success, negative errno on failure.
 */
int asr_dynamixel_app_handle_position_read(uint8_t id, int32_t *position);

#ifdef __cplusplus
}
#endif

#endif /* ASR_DYNAMIXEL_H_ */
