/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMU and AHRS helper APIs for the project sensor stack.
 */

#ifndef ASR_IMU_H_
#define ASR_IMU_H_

#include <stdbool.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

struct asr_imu_sample {
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    struct sensor_value temp;
};

/**
 * Initialize the IMU device selected by the module's devicetree node.
 * @return 0 on success, negative errno on failure.
 */
int asr_imu_init(void);

/**
 * Fetch and read a single IMU sample.
 * @param sample Output sample storage; must not be NULL.
 * @return 0 on success, negative errno on failure.
 */
int asr_imu_read(struct asr_imu_sample *sample);

/**
 * Fetch a sample, cache it, and write float data to unit_status.
 * @return 0 on success, negative errno on failure.
 */
int asr_imu_update(void);

/**
 * Get the latest cached IMU sample.
 * @param sample  Output storage; must not be NULL.
 * @return 0 on success, -ENODATA if no sample has been captured yet.
 */
int asr_imu_get_latest(struct asr_imu_sample *sample);

struct asr_ahrs_sample {
    float quaternion[4];
    float euler_deg[3];
    float gravity[3];
    float linear_accel[3];
    float earth_accel[3];
    bool initialising;
    bool angular_rate_recovery;
    bool acceleration_recovery;
    bool magnetic_recovery;
};

int asr_ahrs_init(void);
int asr_ahrs_reset(void);
int asr_ahrs_update_from_imu(const float accel_mps2[3], const float gyro_rads[3], float dt_s);
int asr_ahrs_get_latest(struct asr_ahrs_sample *sample);
bool asr_ahrs_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_IMU_H_ */
