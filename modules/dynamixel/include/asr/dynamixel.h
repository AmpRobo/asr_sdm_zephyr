/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ASR_DYNAMIXEL_H_
#define ASR_DYNAMIXEL_H_

#include <stdbool.h>
#include <stddef.h>
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
 * Read the servo Drive Mode(10).
 *
 * @param mode  Output raw drive mode value.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_get_drive_mode(uint8_t *mode);

/**
 * Write the servo Drive Mode(10).
 *
 * @param mode  Raw drive mode value.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_set_drive_mode(uint8_t mode);

/**
 * Set the servo torque enable state.
 *
 * @param enable  True to enable torque, false to disable it.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_set_torque(bool enable);

/**
 * Read the servo Torque Enable(64) state.
 *
 * @param enabled  Output torque state.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_get_torque(bool *enabled);

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
 * Read the servo Profile Acceleration(108).
 *
 * In velocity-based profile mode the XM430-W350 range is 0..32767. In
 * time-based profile mode it is 0..32737 and must also satisfy the model's
 * acceleration/velocity relationship. The setter reads Drive Mode and the
 * paired profile register to enforce these mode-dependent constraints.
 *
 * @param acceleration  Output raw profile acceleration value.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_get_profile_acceleration(uint32_t *acceleration);

/**
 * Write the servo Profile Acceleration(108).
 *
 * @param acceleration  Raw value in the range 0..32767.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_set_profile_acceleration(uint32_t acceleration);

/**
 * Read the servo Profile Velocity(112).
 *
 * In velocity-based profile mode the XM430-W350 range is 0..32767. In
 * time-based profile mode it is 0..32737 and must also satisfy the model's
 * acceleration/velocity relationship. The setter reads Drive Mode and the
 * paired profile register to enforce these mode-dependent constraints.
 *
 * @param velocity  Output raw profile velocity value.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_get_profile_velocity(uint32_t *velocity);

/**
 * Write the servo Profile Velocity(112).
 *
 * @param velocity  Raw value in the range 0..32767.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_set_profile_velocity(uint32_t velocity);

/**
 * Write the servo goal position using Dynamixel Protocol 2.0.
 *
 * @param goal_position  Raw Goal Position(116) value.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_set_goal_position(int32_t goal_position);

/**
 * Read the servo Goal Position(116).
 *
 * @param goal_position  Output raw goal position value.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_get_goal_position(int32_t *goal_position);

/**
 * Read arbitrary contiguous bytes from the configured servo control table.
 * Intended for maintenance/configuration snapshots; callers must use the
 * control-table width defined by the target model.
 *
 * @param address  Control-table start address.
 * @param data     Output buffer.
 * @param length   Number of bytes to read.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_read_control_table(uint16_t address, uint8_t *data,
				     size_t length);

/**
 * Write arbitrary contiguous bytes to the configured servo control table.
 * Intended for maintenance/configuration restoration; callers must ensure
 * torque and access conditions required by the target model.
 *
 * @param address  Control-table start address.
 * @param data     Input buffer.
 * @param length   Number of bytes to write.
 * @return 0 on success, negative errno otherwise.
 */
int asr_dynamixel_write_control_table(uint16_t address, const uint8_t *data,
				      size_t length);

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
int asr_dynamixel_app_request_torque(uint8_t id, bool enable);
void asr_dynamixel_app_handle_torque(uint8_t id, bool enable);

/**
 * Comm-callback: send a goal position to the primary servo. The goal is
 * range-checked against the cached position limits.
 */
int asr_dynamixel_app_request_goal_position(uint8_t id, int32_t goal_position);
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
