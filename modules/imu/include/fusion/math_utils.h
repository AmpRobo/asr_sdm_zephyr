#ifndef _MATH_UTILS_H_
#define _MATH_UTILS_H_

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef union
{
    float array[3];

    struct
    {
        float x;
        float y;
        float z;
    } axis;
} FusionVector;

typedef union
{
    float array[4];

    struct
    {
        float w;
        float x;
        float y;
        float z;
    } element;
} FusionQuaternion;

typedef union
{
    float array[3][3];

    struct
    {
        float xx;
        float xy;
        float xz;
        float yx;
        float yy;
        float yz;
        float zx;
        float zy;
        float zz;
    } element;
} FusionMatrix;

typedef union
{
    float array[3];

    struct
    {
        float roll;
        float pitch;
        float yaw;
    } angle;
} FusionEuler;

#define FUSION_VECTOR_ZERO ((FusionVector){.array = {0.0f, 0.0f, 0.0f}})
#define FUSION_VECTOR_ONES ((FusionVector){.array = {1.0f, 1.0f, 1.0f}})
#define FUSION_IDENTITY_QUATERNION ((FusionQuaternion){.array = {1.0f, 0.0f, 0.0f, 0.0f}})
#define FUSION_IDENTITY_MATRIX ((FusionMatrix){.array = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}})
#define FUSION_EULER_ZERO ((FusionEuler){.array = {0.0f, 0.0f, 0.0f}})

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

static inline float FusionDegreesToRadians(const float degrees) { return degrees * ((float)M_PI / 180.0f); }

static inline float FusionRadiansToDegrees(const float radians) { return radians * (180.0f / (float)M_PI); }

static inline float FusionAsin(const float value)
{
    if (value <= -1.0f)
    {
        return (float)M_PI / -2.0f;
    }
    if (value >= 1.0f)
    {
        return (float)M_PI / 2.0f;
    }
    return asinf(value);
}

static inline float FusionFastInverseSqrt(const float x)
{
    typedef union
    {
        float f;
        int32_t i;
    } Union32;

    Union32 union32 = {.f = x};
    union32.i = 0x5F1F1412 - (union32.i >> 1);
    return union32.f * (1.69000231f - 0.714158168f * x * union32.f * union32.f);
}

static inline bool FusionVectorIsZero(const FusionVector vector)
{
    return (vector.axis.x == 0.0f) && (vector.axis.y == 0.0f) && (vector.axis.z == 0.0f);
}

static inline FusionVector FusionVectorAdd(const FusionVector vectorA, const FusionVector vectorB)
{
    const FusionVector result = {.axis = {
                                     .x = vectorA.axis.x + vectorB.axis.x,
                                     .y = vectorA.axis.y + vectorB.axis.y,
                                     .z = vectorA.axis.z + vectorB.axis.z,
                                 }};
    return result;
}

static inline FusionVector fusion_vector_subtract(const FusionVector vectorA, const FusionVector vectorB)
{
    const FusionVector result = {.axis = {
                                     .x = vectorA.axis.x - vectorB.axis.x,
                                     .y = vectorA.axis.y - vectorB.axis.y,
                                     .z = vectorA.axis.z - vectorB.axis.z,
                                 }};
    return result;
}

static inline float FusionVectorSum(const FusionVector vector) { return vector.axis.x + vector.axis.y + vector.axis.z; }

static inline FusionVector FusionVectorMultiplyScalar(const FusionVector vector, const float scalar)
{
    const FusionVector result = {.axis = {
                                     .x = vector.axis.x * scalar,
                                     .y = vector.axis.y * scalar,
                                     .z = vector.axis.z * scalar,
                                 }};
    return result;
}

static inline FusionVector FusionVectorHadamardProduct(const FusionVector vectorA, const FusionVector vectorB)
{
    const FusionVector result = {.axis = {
                                     .x = vectorA.axis.x * vectorB.axis.x,
                                     .y = vectorA.axis.y * vectorB.axis.y,
                                     .z = vectorA.axis.z * vectorB.axis.z,
                                 }};
    return result;
}

static inline FusionVector FusionVectorCrossProduct(const FusionVector vectorA, const FusionVector vectorB)
{
#define A vectorA.axis
#define B vectorB.axis
    const FusionVector result = {.axis = {
                                     .x = A.y * B.z - A.z * B.y,
                                     .y = A.z * B.x - A.x * B.z,
                                     .z = A.x * B.y - A.y * B.x,
                                 }};
    return result;
#undef A
#undef B
}

static inline float FusionVectorDotProduct(const FusionVector vectorA, const FusionVector vectorB)
{
    return FusionVectorSum(FusionVectorHadamardProduct(vectorA, vectorB));
}

static inline float FusionVectorMagnitudeSquared(const FusionVector vector)
{
    return FusionVectorSum(FusionVectorHadamardProduct(vector, vector));
}

static inline float FusionVectorMagnitude(const FusionVector vector)
{
    return sqrtf(FusionVectorMagnitudeSquared(vector));
}

static inline FusionVector FusionVectorNormalise(const FusionVector vector)
{
    const float magnitudeReciprocal = FusionFastInverseSqrt(FusionVectorMagnitudeSquared(vector));
    return FusionVectorMultiplyScalar(vector, magnitudeReciprocal);
}

static inline FusionQuaternion FusionQuaternionAdd(const FusionQuaternion quaternionA,
                                                   const FusionQuaternion quaternionB)
{
    const FusionQuaternion result = {.element = {
                                         .w = quaternionA.element.w + quaternionB.element.w,
                                         .x = quaternionA.element.x + quaternionB.element.x,
                                         .y = quaternionA.element.y + quaternionB.element.y,
                                         .z = quaternionA.element.z + quaternionB.element.z,
                                     }};
    return result;
}

static inline FusionQuaternion FusionQuaternionMultiply(const FusionQuaternion quaternionA,
                                                        const FusionQuaternion quaternionB)
{
#define A quaternionA.element
#define B quaternionB.element
    const FusionQuaternion result = {.element = {
                                         .w = A.w * B.w - A.x * B.x - A.y * B.y - A.z * B.z,
                                         .x = A.w * B.x + A.x * B.w + A.y * B.z - A.z * B.y,
                                         .y = A.w * B.y - A.x * B.z + A.y * B.w + A.z * B.x,
                                         .z = A.w * B.z + A.x * B.y - A.y * B.x + A.z * B.w,
                                     }};
    return result;
#undef A
#undef B
}

static inline FusionQuaternion FusionQuaternionMultiplyVector(const FusionQuaternion quaternion,
                                                              const FusionVector vector)
{
#define Q quaternion.element
#define V vector.axis
    const FusionQuaternion result = {.element = {
                                         .w = -Q.x * V.x - Q.y * V.y - Q.z * V.z,
                                         .x = Q.w * V.x + Q.y * V.z - Q.z * V.y,
                                         .y = Q.w * V.y - Q.x * V.z + Q.z * V.x,
                                         .z = Q.w * V.z + Q.x * V.y - Q.y * V.x,
                                     }};
    return result;
#undef Q
#undef V
}

static inline FusionQuaternion FusionQuaternionNormalise(const FusionQuaternion quaternion)
{
#define Q quaternion.element
    const float magnitudeReciprocal =
        FusionFastInverseSqrt(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
    const FusionQuaternion result = {.element = {
                                         .w = Q.w * magnitudeReciprocal,
                                         .x = Q.x * magnitudeReciprocal,
                                         .y = Q.y * magnitudeReciprocal,
                                         .z = Q.z * magnitudeReciprocal,
                                     }};
    return result;
#undef Q
}

static inline FusionVector FusionMatrixMultiplyVector(const FusionMatrix matrix, const FusionVector vector)
{
#define R matrix.element
    const FusionVector result = {.axis = {
                                     .x = R.xx * vector.axis.x + R.xy * vector.axis.y + R.xz * vector.axis.z,
                                     .y = R.yx * vector.axis.x + R.yy * vector.axis.y + R.yz * vector.axis.z,
                                     .z = R.zx * vector.axis.x + R.zy * vector.axis.y + R.zz * vector.axis.z,
                                 }};
    return result;
#undef R
}

static inline FusionMatrix FusionQuaternionToMatrix(const FusionQuaternion quaternion)
{
#define Q quaternion.element
    const float qwqw = Q.w * Q.w;
    const float qwqx = Q.w * Q.x;
    const float qwqy = Q.w * Q.y;
    const float qwqz = Q.w * Q.z;
    const float qxqy = Q.x * Q.y;
    const float qxqz = Q.x * Q.z;
    const float qyqz = Q.y * Q.z;
    const FusionMatrix matrix = {.element = {
                                     .xx = 2.0f * (qwqw - 0.5f + Q.x * Q.x),
                                     .xy = 2.0f * (qxqy - qwqz),
                                     .xz = 2.0f * (qxqz + qwqy),
                                     .yx = 2.0f * (qxqy + qwqz),
                                     .yy = 2.0f * (qwqw - 0.5f + Q.y * Q.y),
                                     .yz = 2.0f * (qyqz - qwqx),
                                     .zx = 2.0f * (qxqz - qwqy),
                                     .zy = 2.0f * (qyqz + qwqx),
                                     .zz = 2.0f * (qwqw - 0.5f + Q.z * Q.z),
                                 }};
    return matrix;
#undef Q
}

static inline FusionEuler FusionQuaternionToEuler(const FusionQuaternion quaternion)
{
#define Q quaternion.element
    const float halfMinusQySquared = 0.5f - Q.y * Q.y;
    const FusionEuler euler = {
        .angle = {
            .roll = FusionRadiansToDegrees(atan2f(Q.w * Q.x + Q.y * Q.z, halfMinusQySquared - Q.x * Q.x)),
            .pitch = FusionRadiansToDegrees(FusionAsin(2.0f * (Q.w * Q.y - Q.z * Q.x))),
            .yaw = FusionRadiansToDegrees(atan2f(Q.w * Q.z + Q.x * Q.y, halfMinusQySquared - Q.z * Q.z)),
        }};
    return euler;
#undef Q
}

#endif
