/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Background thread that performs the Dynamixel bring-up sequence
 * (asr_dynamixel_app_init) without blocking main().
 */

#ifndef ASR_DYNAMIXEL_THREAD_H_
#define ASR_DYNAMIXEL_THREAD_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Create the Dynamixel bring-up thread in suspended state. The actual servo
 * bring-up runs in the thread entry after asr_dynamixel_thread_start.
 *
 * @return 0 on success, -EALREADY when already initialised.
 */
int asr_dynamixel_thread_init(void);

/**
 * Start the Dynamixel bring-up thread. Must be called after
 * asr_dynamixel_thread_init.
 *
 * @return 0 on success, -EINVAL if not initialised, -EALREADY if started.
 */
int asr_dynamixel_thread_start(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_DYNAMIXEL_THREAD_H_ */
