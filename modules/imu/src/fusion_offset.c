#include "fusion_offset.h"
#include <math.h>

#define CUTOFF_FREQUENCY (0.02f)
#define TIMEOUT (5)
#define THRESHOLD (3.0f)

void fusion_offset_init(fusion_offset_t *const offset, const uint16_t sampleRate)
{
    offset->filterCoefficient = 2.0f * (float)M_PI * CUTOFF_FREQUENCY * (1.0f / (float)sampleRate);
    offset->timeout = TIMEOUT * sampleRate;
    offset->timer = 0;
    offset->gyroscopeOffset = FUSION_VECTOR_ZERO;
}

FusionVector fusion_offset_update(fusion_offset_t *const offset, FusionVector gyroscope)
{
    gyroscope = fusion_vector_subtract(gyroscope, offset->gyroscopeOffset);

    // if ((fabsf(gyroscope.axis.x) > THRESHOLD) || (fabsf(gyroscope.axis.y) > THRESHOLD) ||
    //     (fabsf(gyroscope.axis.z) > THRESHOLD))
    // {
    //     offset->timer = 0;
    //     return gyroscope;
    // }

    // if (offset->timer < offset->timeout)
    // {
    //     offset->timer++;
    //     return gyroscope;
    // }

    // offset->gyroscopeOffset =
    //     FusionVectorAdd(offset->gyroscopeOffset, FusionVectorMultiplyScalar(gyroscope, offset->filterCoefficient));
    return gyroscope;
}
