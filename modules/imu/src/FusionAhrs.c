#include "FusionAhrs.h"
#include <float.h>
#include <math.h>

#define INITIAL_GAIN (10.0f)
#define INITIALISATION_PERIOD (3.0f)

static inline FusionVector HalfGravity(const fusion_ahrs_t *const ahrs);
static inline FusionVector HalfMagnetic(const fusion_ahrs_t *const ahrs);
static inline FusionVector Feedback(const FusionVector sensor, const FusionVector reference);
static inline int Clamp(const int value, const int min, const int max);

void fusion_ahrs_init(fusion_ahrs_t *const ahrs, uint16_t sample_rate)
{
    fusion_offset_init(&ahrs->offset, sample_rate);
    const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .sample_rate = sample_rate,
        .sample_period = 1.0f / (float)sample_rate,
        .gain = 0.5f,
        .gyroscopeRange = 0.0f,
        .accelerationRejection = 90.0f,
        .magneticRejection = 90.0f,
        .recoveryTriggerPeriod = 0,
    };
    fusionAhrs_set_settings(ahrs, &settings);
    fusion_ahrs_reset(ahrs);
}

void fusion_ahrs_reset(fusion_ahrs_t *const ahrs)
{
    ahrs->quaternion = FUSION_IDENTITY_QUATERNION;
    ahrs->accelerometer = FUSION_VECTOR_ZERO;
    ahrs->initialising = true;
    ahrs->rampedGain = INITIAL_GAIN;
    ahrs->angularRateRecovery = false;
    ahrs->halfAccelerometerFeedback = FUSION_VECTOR_ZERO;
    ahrs->halfMagnetometerFeedback = FUSION_VECTOR_ZERO;
    ahrs->accelerometerIgnored = false;
    ahrs->accelerationRecoveryTrigger = 0;
    ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
    ahrs->magnetometerIgnored = false;
    ahrs->magneticRecoveryTrigger = 0;
    ahrs->magneticRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
}

void fusionAhrs_set_settings(fusion_ahrs_t *const ahrs, const FusionAhrsSettings *const settings)
{
    ahrs->settings.convention = settings->convention;
    ahrs->settings.sample_rate = settings->sample_rate;
    ahrs->settings.sample_period = settings->sample_period;
    ahrs->settings.gain = settings->gain;
    ahrs->settings.gyroscopeRange = settings->gyroscopeRange == 0.0f ? FLT_MAX : 0.98f * settings->gyroscopeRange;
    ahrs->settings.accelerationRejection =
        settings->accelerationRejection == 0.0f
            ? FLT_MAX
            : powf(0.5f * sinf(FusionDegreesToRadians(settings->accelerationRejection)), 2);
    ahrs->settings.magneticRejection =
        settings->magneticRejection == 0.0f
            ? FLT_MAX
            : powf(0.5f * sinf(FusionDegreesToRadians(settings->magneticRejection)), 2);
    ahrs->settings.recoveryTriggerPeriod = settings->recoveryTriggerPeriod;
    ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
    ahrs->magneticRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
    if ((settings->gain == 0.0f) || (settings->recoveryTriggerPeriod == 0))
    {
        ahrs->settings.accelerationRejection = FLT_MAX;
        ahrs->settings.magneticRejection = FLT_MAX;
    }
    if (ahrs->initialising == false)
    {
        ahrs->rampedGain = ahrs->settings.gain;
    }
    ahrs->rampedGainStep = (INITIAL_GAIN - ahrs->settings.gain) / INITIALISATION_PERIOD;
}

void fusion_ahrs_update(fusion_ahrs_t *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer,
                        const FusionVector magnetometer, const float deltaTime)
{
#define Q ahrs->quaternion.element
    ahrs->accelerometer = accelerometer;

    if ((fabsf(gyroscope.axis.x) > ahrs->settings.gyroscopeRange) ||
        (fabsf(gyroscope.axis.y) > ahrs->settings.gyroscopeRange) ||
        (fabsf(gyroscope.axis.z) > ahrs->settings.gyroscopeRange))
    {
        const FusionQuaternion quaternion = ahrs->quaternion;
        fusion_ahrs_reset(ahrs);
        ahrs->quaternion = quaternion;
        ahrs->angularRateRecovery = true;
    }

    if (ahrs->initialising)
    {
        ahrs->rampedGain -= ahrs->rampedGainStep * deltaTime;
        if ((ahrs->rampedGain < ahrs->settings.gain) || (ahrs->settings.gain == 0.0f))
        {
            ahrs->rampedGain = ahrs->settings.gain;
            ahrs->initialising = false;
            ahrs->angularRateRecovery = false;
        }
    }

    const FusionVector halfGravity = HalfGravity(ahrs);

    FusionVector halfAccelerometerFeedback = FUSION_VECTOR_ZERO;
    ahrs->accelerometerIgnored = false;
    if (FusionVectorIsZero(accelerometer) == false)
    {
        ahrs->halfAccelerometerFeedback = Feedback(halfGravity, FusionVectorNormalise(accelerometer));

        if (ahrs->initialising ||
            (FusionVectorMagnitudeSquared(ahrs->halfAccelerometerFeedback) <= ahrs->settings.accelerationRejection))
        {
            ahrs->accelerometerIgnored = false;
            ahrs->accelerationRecoveryTrigger -= 9;
        }
        else
        {
            ahrs->accelerationRecoveryTrigger += 1;
        }

        if (ahrs->accelerationRecoveryTrigger > ahrs->accelerationRecoveryTimeout)
        {
            ahrs->accelerationRecoveryTimeout = 0;
            ahrs->accelerometerIgnored = false;
        }
        else
        {
            ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
        }
        ahrs->accelerationRecoveryTrigger =
            Clamp(ahrs->accelerationRecoveryTrigger, 0, ahrs->settings.recoveryTriggerPeriod);

        if (ahrs->accelerometerIgnored == false)
        {
            halfAccelerometerFeedback = ahrs->halfAccelerometerFeedback;
        }
    }

    FusionVector halfMagnetometerFeedback = FUSION_VECTOR_ZERO;
    ahrs->magnetometerIgnored = true;
    if (FusionVectorIsZero(magnetometer) == false)
    {
        const FusionVector halfMagnetic = HalfMagnetic(ahrs);
        ahrs->halfMagnetometerFeedback =
            Feedback(FusionVectorNormalise(FusionVectorCrossProduct(halfGravity, magnetometer)), halfMagnetic);

        if (ahrs->initialising ||
            (FusionVectorMagnitudeSquared(ahrs->halfMagnetometerFeedback) <= ahrs->settings.magneticRejection))
        {
            ahrs->magnetometerIgnored = false;
            ahrs->magneticRecoveryTrigger -= 9;
        }
        else
        {
            ahrs->magneticRecoveryTrigger += 1;
        }

        if (ahrs->magneticRecoveryTrigger > ahrs->magneticRecoveryTimeout)
        {
            ahrs->magneticRecoveryTimeout = 0;
            ahrs->magnetometerIgnored = false;
        }
        else
        {
            ahrs->magneticRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
        }
        ahrs->magneticRecoveryTrigger = Clamp(ahrs->magneticRecoveryTrigger, 0, ahrs->settings.recoveryTriggerPeriod);

        if (ahrs->magnetometerIgnored == false)
        {
            halfMagnetometerFeedback = ahrs->halfMagnetometerFeedback;
        }
    }

    const FusionVector halfGyroscope = FusionVectorMultiplyScalar(gyroscope, FusionDegreesToRadians(0.5f));
    const FusionVector adjustedHalfGyroscope = FusionVectorAdd(
        halfGyroscope,
        FusionVectorMultiplyScalar(FusionVectorAdd(halfAccelerometerFeedback, halfMagnetometerFeedback),
                                   ahrs->rampedGain));

    ahrs->quaternion = FusionQuaternionAdd(
        ahrs->quaternion,
        FusionQuaternionMultiplyVector(ahrs->quaternion, FusionVectorMultiplyScalar(adjustedHalfGyroscope, deltaTime)));
    ahrs->quaternion = FusionQuaternionNormalise(ahrs->quaternion);
#undef Q
}

static inline FusionVector HalfGravity(const fusion_ahrs_t *const ahrs)
{
#define Q ahrs->quaternion.element
    switch (ahrs->settings.convention)
    {
    case FusionConventionNwu:
    case FusionConventionEnu:
    {
        const FusionVector halfGravity = {.axis = {
                                              .x = Q.x * Q.z - Q.w * Q.y,
                                              .y = Q.y * Q.z + Q.w * Q.x,
                                              .z = Q.w * Q.w - 0.5f + Q.z * Q.z,
                                          }};
        return halfGravity;
    }
    case FusionConventionNed:
    {
        const FusionVector halfGravity = {.axis = {
                                              .x = Q.w * Q.y - Q.x * Q.z,
                                              .y = -1.0f * (Q.y * Q.z + Q.w * Q.x),
                                              .z = 0.5f - Q.w * Q.w - Q.z * Q.z,
                                          }};
        return halfGravity;
    }
    }
    return FUSION_VECTOR_ZERO;
#undef Q
}

static inline FusionVector HalfMagnetic(const fusion_ahrs_t *const ahrs)
{
#define Q ahrs->quaternion.element
    switch (ahrs->settings.convention)
    {
    case FusionConventionNwu:
    {
        const FusionVector halfMagnetic = {.axis = {
                                               .x = Q.x * Q.y + Q.w * Q.z,
                                               .y = Q.w * Q.w - 0.5f + Q.y * Q.y,
                                               .z = Q.y * Q.z - Q.w * Q.x,
                                           }};
        return halfMagnetic;
    }
    case FusionConventionEnu:
    {
        const FusionVector halfMagnetic = {.axis = {
                                               .x = 0.5f - Q.w * Q.w - Q.x * Q.x,
                                               .y = Q.w * Q.z - Q.x * Q.y,
                                               .z = -1.0f * (Q.x * Q.z + Q.w * Q.y),
                                           }};
        return halfMagnetic;
    }
    case FusionConventionNed:
    {
        const FusionVector halfMagnetic = {.axis = {
                                               .x = -1.0f * (Q.x * Q.y + Q.w * Q.z),
                                               .y = 0.5f - Q.w * Q.w - Q.y * Q.y,
                                               .z = Q.w * Q.x - Q.y * Q.z,
                                           }};
        return halfMagnetic;
    }
    }
    return FUSION_VECTOR_ZERO;
#undef Q
}

static inline FusionVector Feedback(const FusionVector sensor, const FusionVector reference)
{
    if (FusionVectorDotProduct(sensor, reference) < 0.0f)
    {
        return FusionVectorNormalise(FusionVectorCrossProduct(sensor, reference));
    }
    return FusionVectorCrossProduct(sensor, reference);
}

static inline int Clamp(const int value, const int min, const int max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

void fusion_ahrs_update_no_magnetometer(fusion_ahrs_t *const ahrs, const FusionVector gyroscope,
                                        const FusionVector accelerometer, const float deltaTime)
{
    fusion_ahrs_update(ahrs, gyroscope, accelerometer, FUSION_VECTOR_ZERO, deltaTime);

    if (ahrs->initialising)
    {
        FusionAhrsSetHeading(ahrs, 0.0f);
    }
}

void FusionAhrsUpdateExternalHeading(fusion_ahrs_t *const ahrs, const FusionVector gyroscope,
                                     const FusionVector accelerometer, const float heading, const float deltaTime)
{
#define Q ahrs->quaternion.element
    const float roll = atan2f(Q.w * Q.x + Q.y * Q.z, 0.5f - Q.y * Q.y - Q.x * Q.x);
    const float headingRadians = FusionDegreesToRadians(heading);
    const float sinHeadingRadians = sinf(headingRadians);
    const FusionVector magnetometer = {.axis = {
                                           .x = cosf(headingRadians),
                                           .y = -1.0f * cosf(roll) * sinHeadingRadians,
                                           .z = sinHeadingRadians * sinf(roll),
                                       }};
    fusion_ahrs_update(ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
#undef Q
}

FusionQuaternion FusionAhrsGetQuaternion(const fusion_ahrs_t *const ahrs) { return ahrs->quaternion; }

void FusionAhrsSetQuaternion(fusion_ahrs_t *const ahrs, const FusionQuaternion quaternion)
{
    ahrs->quaternion = quaternion;
}

FusionVector FusionAhrsGetGravity(const fusion_ahrs_t *const ahrs)
{
#define Q ahrs->quaternion.element
    const FusionVector gravity = {.axis = {
                                      .x = 2.0f * (Q.x * Q.z - Q.w * Q.y),
                                      .y = 2.0f * (Q.y * Q.z + Q.w * Q.x),
                                      .z = 2.0f * (Q.w * Q.w - 0.5f + Q.z * Q.z),
                                  }};
    return gravity;
#undef Q
}

FusionVector FusionAhrsGetLinearAcceleration(const fusion_ahrs_t *const ahrs)
{
    switch (ahrs->settings.convention)
    {
    case FusionConventionNwu:
    case FusionConventionEnu:
        return fusion_vector_subtract(ahrs->accelerometer, FusionAhrsGetGravity(ahrs));
    case FusionConventionNed:
        return FusionVectorAdd(ahrs->accelerometer, FusionAhrsGetGravity(ahrs));
    }
    return FUSION_VECTOR_ZERO;
}

FusionVector FusionAhrsGetEarthAcceleration(const fusion_ahrs_t *const ahrs)
{
#define Q ahrs->quaternion.element
#define A ahrs->accelerometer.axis
    const float qwqw = Q.w * Q.w;
    const float qwqx = Q.w * Q.x;
    const float qwqy = Q.w * Q.y;
    const float qwqz = Q.w * Q.z;
    const float qxqy = Q.x * Q.y;
    const float qxqz = Q.x * Q.z;
    const float qyqz = Q.y * Q.z;
    FusionVector accelerometer = {
        .axis = {
            .x = 2.0f * ((qwqw - 0.5f + Q.x * Q.x) * A.x + (qxqy - qwqz) * A.y + (qxqz + qwqy) * A.z),
            .y = 2.0f * ((qxqy + qwqz) * A.x + (qwqw - 0.5f + Q.y * Q.y) * A.y + (qyqz - qwqx) * A.z),
            .z = 2.0f * ((qxqz - qwqy) * A.x + (qyqz + qwqx) * A.y + (qwqw - 0.5f + Q.z * Q.z) * A.z),
        }};

    switch (ahrs->settings.convention)
    {
    case FusionConventionNwu:
    case FusionConventionEnu:
        accelerometer.axis.z -= 1.0f;
        break;
    case FusionConventionNed:
        accelerometer.axis.z += 1.0f;
        break;
    }
    return accelerometer;
#undef Q
#undef A
}

FusionAhrsInternalStates FusionAhrsGetInternalStates(const fusion_ahrs_t *const ahrs)
{
    const FusionAhrsInternalStates internalStates = {
        .accelerationError =
            FusionRadiansToDegrees(FusionAsin(2.0f * FusionVectorMagnitude(ahrs->halfAccelerometerFeedback))),
        .accelerometerIgnored = ahrs->accelerometerIgnored,
        .accelerationRecoveryTrigger =
            ahrs->settings.recoveryTriggerPeriod == 0
                ? 0.0f
                : (float)ahrs->accelerationRecoveryTrigger / (float)ahrs->settings.recoveryTriggerPeriod,
        .magneticError = FusionRadiansToDegrees(FusionAsin(2.0f * FusionVectorMagnitude(ahrs->halfMagnetometerFeedback))),
        .magnetometerIgnored = ahrs->magnetometerIgnored,
        .magneticRecoveryTrigger =
            ahrs->settings.recoveryTriggerPeriod == 0
                ? 0.0f
                : (float)ahrs->magneticRecoveryTrigger / (float)ahrs->settings.recoveryTriggerPeriod,
    };
    return internalStates;
}

FusionAhrsFlags FusionAhrsGetFlags(const fusion_ahrs_t *const ahrs)
{
    const FusionAhrsFlags flags = {
        .initialising = ahrs->initialising,
        .angularRateRecovery = ahrs->angularRateRecovery,
        .accelerationRecovery = ahrs->accelerationRecoveryTrigger > ahrs->accelerationRecoveryTimeout,
        .magneticRecovery = ahrs->magneticRecoveryTrigger > ahrs->magneticRecoveryTimeout,
    };
    return flags;
}

void FusionAhrsSetHeading(fusion_ahrs_t *const ahrs, const float heading)
{
#define Q ahrs->quaternion.element
    const float yaw = atan2f(Q.w * Q.z + Q.x * Q.y, 0.5f - Q.y * Q.y - Q.z * Q.z);
    const float halfYawMinusHeading = 0.5f * (yaw - FusionDegreesToRadians(heading));
    const FusionQuaternion rotation = {.element = {
                                           .w = cosf(halfYawMinusHeading),
                                           .x = 0.0f,
                                           .y = 0.0f,
                                           .z = -1.0f * sinf(halfYawMinusHeading),
                                       }};
    ahrs->quaternion = FusionQuaternionMultiply(rotation, ahrs->quaternion);
#undef Q
}
