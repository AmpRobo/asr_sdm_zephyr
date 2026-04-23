/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Background thread that periodically reads and caches IMU samples.
 */

#ifndef ASR_IMU_THREAD_H_
#define ASR_IMU_THREAD_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Create the IMU thread (not started until asr_imu_thread_start).
 * Hardware init runs in the thread entry after k_thread_start.
 *
 * @return 0 on success, -EALREADY when already initialised.
 */
int asr_imu_thread_init(void);

/**
 * Start the IMU thread (k_thread_start; must call asr_imu_thread_init first).
 *
 * @return 0 on success, -EINVAL if not initialised.
 */
int asr_imu_thread_start(void);

int asr_ahrs_thread_init(void);
int asr_ahrs_thread_start(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_IMU_THREAD_H_ */
