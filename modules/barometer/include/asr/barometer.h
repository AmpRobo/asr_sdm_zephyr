/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * HP206F 气压计项目封装。
 */

#ifndef ASR_BAROMETER_H_
#define ASR_BAROMETER_H_

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

struct asr_barometer_sample {
	struct sensor_value pressure;
	struct sensor_value temperature;
};

/**
 * 初始化气压计设备并设置过采样率。
 *
 * @return 成功返回 0，失败返回负 errno。
 */
int asr_barometer_init(void);

/**
 * 同步读取一次压力和温度。
 *
 * @param sample 输出样本，不能为 NULL。
 * @return 成功返回 0，失败返回负 errno。
 */
int asr_barometer_read(struct asr_barometer_sample *sample);

/**
 * 读取完整样本并更新模块内部缓存。
 *
 * @return 成功返回 0，失败返回负 errno。
 */
int asr_barometer_update(void);

/**
 * 获取最近一次成功读取的完整样本。
 *
 * @param sample 输出样本，不能为 NULL。
 * @return 成功返回 0；无有效缓存返回 -ENODATA；其他失败返回负 errno。
 */
int asr_barometer_get_latest(struct asr_barometer_sample *sample);

#ifdef __cplusplus
}
#endif

#endif /* ASR_BAROMETER_H_ */
