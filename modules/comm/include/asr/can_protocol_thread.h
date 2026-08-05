/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CAN protocol processing thread.
 *
 * Provides a background thread that runs the CAN protocol transport loop
 * (receive BCAN PROTOL frames -> filter CAN ID -> protocol_core_process ->
 * optionally send response).
 */

#ifndef ASR_CAN_PROTOCOL_THREAD_H_
#define ASR_CAN_PROTOCOL_THREAD_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialise the CAN protocol thread in suspended state.
 *
 * Must be called before asr_can_protocol_thread_start().
 *
 * @return 0 on success, -EALREADY if already initialised.
 */
int asr_can_protocol_thread_init(void);

/**
 * Start the CAN protocol thread.
 *
 * @return 0 on success, -EINVAL if not initialised.
 */
int asr_can_protocol_thread_start(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_CAN_PROTOCOL_THREAD_H_ */