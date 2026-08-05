/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * HP206F 气压计后台采样线程。
 */

#ifndef ASR_BAROMETER_THREAD_H_
#define ASR_BAROMETER_THREAD_H_

#ifdef __cplusplus
extern "C" {
#endif

/** 创建处于 suspended 状态的气压计线程。 */
int asr_barometer_thread_init(void);

/** 启动已创建的气压计线程。 */
int asr_barometer_thread_start(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_BAROMETER_THREAD_H_ */
