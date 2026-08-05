/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CPU monitor background thread.
 *
 * A periodic timer gives a semaphore every ASR_CPU_MONITOR_PERIOD_MS.
 * The thread blocks on the semaphore and calls asr_cpu_monitor_update().
 */

#include <asr/cpu_monitor.h>
#include <asr/cpu_monitor_thread.h>

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(asr_cpu_monitor, LOG_LEVEL_INF);

#define CPU_MON_STACK_SIZE CONFIG_ASR_CPU_MONITOR_THREAD_STACK_SIZE
#define CPU_MON_PRIO CONFIG_ASR_CPU_MONITOR_THREAD_PRIORITY
#define CPU_MON_PERIOD_MS CONFIG_ASR_CPU_MONITOR_PERIOD_MS

BUILD_ASSERT(CPU_MON_PERIOD_MS > 0U, "CONFIG_ASR_CPU_MONITOR_PERIOD_MS must be > 0");

static K_THREAD_STACK_DEFINE(cpu_mon_stack, CPU_MON_STACK_SIZE);
static struct k_thread cpu_mon_thread_data;
static bool cpu_mon_started;

static K_SEM_DEFINE(cpu_mon_tick_sem, 0, 1);
static struct k_timer cpu_mon_timer;

static void cpu_mon_timer_expiry(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&cpu_mon_tick_sem);
}

static void cpu_mon_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    k_timer_init(&cpu_mon_timer, cpu_mon_timer_expiry, NULL);
    k_timer_start(&cpu_mon_timer, K_MSEC(CPU_MON_PERIOD_MS), K_MSEC(CPU_MON_PERIOD_MS));

    for (;;)
    {
        k_sem_take(&cpu_mon_tick_sem, K_FOREVER);
        asr_cpu_monitor_update();
    }
}

int asr_cpu_monitor_thread_init(void)
{
    if (cpu_mon_started)
    {
        LOG_WRN("CPU monitor thread already initialised");
        return -EALREADY;
    }

    k_thread_create(&cpu_mon_thread_data, cpu_mon_stack, K_THREAD_STACK_SIZEOF(cpu_mon_stack), cpu_mon_thread_entry,
                    NULL, NULL, NULL, CPU_MON_PRIO, 0, K_FOREVER);
    k_thread_name_set(&cpu_mon_thread_data, "cpu_monitor");
    cpu_mon_started = true;
    LOG_INF("CPU monitor thread initialised (suspended), prio %d", k_thread_priority_get(&cpu_mon_thread_data));
    return 0;
}

int asr_cpu_monitor_thread_start(void)
{
    if (!cpu_mon_started)
    {
        LOG_ERR("CPU monitor thread not initialised");
        return -EINVAL;
    }
    k_thread_start(&cpu_mon_thread_data);
    LOG_INF("CPU monitor thread started");
    return 0;
}
