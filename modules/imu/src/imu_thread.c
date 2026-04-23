/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <asr/imu.h>
#include <asr/imu_thread.h>
#include <asr/robot_base.h>

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(asr_imu, LOG_LEVEL_INF);

#define IMU_THREAD_STACK_SIZE 2048U
#define IMU_THREAD_PRIO 7
#define IMU_STARTUP_DELAY_MS CONFIG_ASR_IMU_STARTUP_DELAY_MS
#define IMU_THREAD_PERIOD_MS CONFIG_ASR_IMU_PERIOD_MS
#define ASR_AHRS_LOG_INTERVAL_MS 1000

BUILD_ASSERT(IMU_THREAD_PERIOD_MS > 0U, "CONFIG_ASR_IMU_PERIOD_MS must be > 0");

static K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;
static bool imu_thread_initialized;
static bool imu_thread_started;

static K_SEM_DEFINE(imu_tick_sem, 0, 1);
static struct k_timer imu_timer;

#if IS_ENABLED(CONFIG_ASR_AHRS)
static bool ahrs_ready;

static void imu_copy_ahrs_sample_to_status(const struct asr_ahrs_sample *sample)
{
    for (int i = 0; i < 4; i++)
    {
        unit_status.ahrs.quaternion[i] = sample->quaternion[i];
    }

    for (int i = 0; i < 3; i++)
    {
        unit_status.ahrs.euler_deg[i] = sample->euler_deg[i];
        unit_status.ahrs.gravity[i] = sample->gravity[i];
        unit_status.ahrs.linear_accel[i] = sample->linear_accel[i];
        unit_status.ahrs.earth_accel[i] = sample->earth_accel[i];
    }

    unit_status.ahrs.ready = true;
    unit_status.ahrs_flags.initialising = sample->initialising;
    unit_status.ahrs_flags.angular_rate_recovery = sample->angular_rate_recovery;
    unit_status.ahrs_flags.acceleration_recovery = sample->acceleration_recovery;
    unit_status.ahrs_flags.magnetic_recovery = sample->magnetic_recovery;
}
#endif

static void imu_timer_expiry(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&imu_tick_sem);
}

static void imu_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int ret;

    if (IMU_STARTUP_DELAY_MS > 0U)
    {
        k_sleep(K_MSEC(IMU_STARTUP_DELAY_MS));
    }

    ret = asr_imu_init();
    if (ret < 0)
    {
        return;
    }

#if IS_ENABLED(CONFIG_ASR_AHRS)
    ret = asr_ahrs_init();
    if (ret < 0)
    {
        LOG_ERR("AHRS init failed: %d", ret);
    }
    else
    {
        ahrs_ready = true;
        LOG_INF("AHRS ready, period: %u ms", IMU_THREAD_PERIOD_MS);
    }
#endif

    LOG_INF("IMU device ready, period: %u ms", IMU_THREAD_PERIOD_MS);

    k_timer_init(&imu_timer, imu_timer_expiry, NULL);
    k_timer_start(&imu_timer, K_MSEC(IMU_THREAD_PERIOD_MS), K_MSEC(IMU_THREAD_PERIOD_MS));

#if IS_ENABLED(CONFIG_ASR_AHRS)
    int64_t last_ahrs_update_ms = 0;
    int64_t last_ahrs_log_ms = 0;
#endif

    for (;;)
    {
        k_sem_take(&imu_tick_sem, K_FOREVER);

        if (asr_imu_update() < 0)
        {
            LOG_ERR("IMU read failed");
            continue;
        }

#if IS_ENABLED(CONFIG_ASR_AHRS)
        if (ahrs_ready)
        {
            struct asr_imu_sample imu_sample;
            struct asr_ahrs_sample ahrs_sample;
            float accel_mps2[3];
            float gyro_rads[3];
            float dt_s;
            int64_t now_ms;

            if (asr_imu_get_latest(&imu_sample) < 0)
            {
                continue;
            }

            for (int i = 0; i < 3; i++)
            {
                accel_mps2[i] = sensor_value_to_float(&imu_sample.accel[i]);
                gyro_rads[i] = sensor_value_to_float(&imu_sample.gyro[i]);
            }

            now_ms = k_uptime_get();
            if (last_ahrs_update_ms == 0)
            {
                dt_s = (float)IMU_THREAD_PERIOD_MS / 1000.0f;
            }
            else
            {
                dt_s = (float)(now_ms - last_ahrs_update_ms) / 1000.0f;
            }
            last_ahrs_update_ms = now_ms;

            if (asr_ahrs_update_from_imu(accel_mps2, gyro_rads, dt_s) < 0)
            {
                LOG_ERR("AHRS update failed");
                continue;
            }

            if (asr_ahrs_get_latest(&ahrs_sample) < 0)
            {
                continue;
            }

            k_mutex_lock(&unit_status_mutex, K_FOREVER);
            imu_copy_ahrs_sample_to_status(&ahrs_sample);
            k_mutex_unlock(&unit_status_mutex);

            if ((now_ms - last_ahrs_log_ms) >= ASR_AHRS_LOG_INTERVAL_MS)
            {
                last_ahrs_log_ms = now_ms;
                LOG_INF("euler [deg]: roll=%10.4f  pitch=%10.4f  yaw=%10.4f",
                        (double)ahrs_sample.euler_deg[0], (double)ahrs_sample.euler_deg[1],
                        (double)ahrs_sample.euler_deg[2]);
            }
        }
#endif
    }
}

int asr_imu_thread_init(void)
{
    if (imu_thread_initialized)
    {
        LOG_WRN("IMU thread already initialised");
        return -EALREADY;
    }

    k_thread_create(&imu_thread_data, imu_thread_stack, K_THREAD_STACK_SIZEOF(imu_thread_stack), imu_thread_entry,
                    NULL, NULL, NULL, IMU_THREAD_PRIO, 0, K_FOREVER);
    k_thread_name_set(&imu_thread_data, "imu_thread");
    imu_thread_initialized = true;
    LOG_INF("IMU thread initialised (suspended), prio %d", k_thread_priority_get(&imu_thread_data));
    return 0;
}

int asr_imu_thread_start(void)
{
    if (!imu_thread_initialized)
    {
        LOG_ERR("IMU thread not initialised");
        return -EINVAL;
    }

    if (imu_thread_started)
    {
        LOG_WRN("IMU thread already started");
        return -EALREADY;
    }

    imu_thread_started = true;
    LOG_INF("IMU thread started");
    k_thread_start(&imu_thread_data);
    return 0;
}

int asr_ahrs_thread_init(void)
{
    return asr_imu_thread_init();
}

int asr_ahrs_thread_start(void)
{
    return asr_imu_thread_start();
}
