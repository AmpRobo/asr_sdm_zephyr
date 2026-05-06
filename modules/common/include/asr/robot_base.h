#ifndef ASR_ROBOT_BASE_H_
#define ASR_ROBOT_BASE_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

#ifndef BUF_SIZE
#define BUF_SIZE 64
#endif

typedef union
{
    int16_t data;

    struct
    {
        uint8_t lsb;
        uint8_t msb;
    } element;
} data16_t;

typedef union
{
    float f;
    int32_t i;
} union32_t;

typedef struct
{
    float alpha;
    float state[3];
} imu_filter_t;

typedef struct
{
    data16_t gyro[3];
    data16_t accel[3];
    uint8_t temperature;
} sensor_imu_t;

typedef struct
{
    float gyro[3];
    float accel[3];
    float temperature;
} sensor_imu_float_t;

typedef struct
{
    float quaternion[4];
    float euler_deg[3];
    float gravity[3];
    float linear_accel[3];
    float earth_accel[3];
    bool ready;
} sensor_ahrs_t;

typedef struct
{
    bool initialising;
    bool angular_rate_recovery;
    bool acceleration_recovery;
    bool magnetic_recovery;
} sensor_ahrs_flags_t;

typedef struct
{
    uint32_t unit_id;
    uint8_t msg_can_tx[BUF_SIZE];
    uint8_t msg_can_rx[BUF_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t flashData[8];
    sensor_imu_t imu_raw_data;
    sensor_imu_float_t imu_float_data;
    sensor_ahrs_t ahrs;
    sensor_ahrs_flags_t ahrs_flags;
    imu_filter_t imu_filter;
    uint8_t cmd_motor[2];
    uint8_t cmd_joint1[4];
    uint8_t cmd_joint2[4];
    bool dynamixel_enable[2];
    bool led_enable;
    bool led_status;
} unit_status_t;

extern unit_status_t unit_status;
extern struct k_mutex unit_status_mutex;

#endif /* ASR_ROBOT_BASE_H_ */
