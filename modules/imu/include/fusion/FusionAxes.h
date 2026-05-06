/**
 * @file FusionAxes.h
 */

#ifndef _FUSION_AXES_H_
#define _FUSION_AXES_H_

#include "math_utils.h"

typedef enum
{
    FusionAxesAlignmentPXPYPZ,
    FusionAxesAlignmentPXNZPY,
    FusionAxesAlignmentPXNYNZ,
    FusionAxesAlignmentPXPZNY,
    FusionAxesAlignmentNXPYNZ,
    FusionAxesAlignmentNXPZPY,
    FusionAxesAlignmentNXNYPZ,
    FusionAxesAlignmentNXNZNY,
    FusionAxesAlignmentPYNXPZ,
    FusionAxesAlignmentPYNZNX,
    FusionAxesAlignmentPYPXNZ,
    FusionAxesAlignmentPYPZPX,
    FusionAxesAlignmentNYPXPZ,
    FusionAxesAlignmentNYNZPX,
    FusionAxesAlignmentNYNXNZ,
    FusionAxesAlignmentNYPZNX,
    FusionAxesAlignmentPZPYNX,
    FusionAxesAlignmentPZPXPY,
    FusionAxesAlignmentPZNYPX,
    FusionAxesAlignmentPZNXNY,
    FusionAxesAlignmentNZPYPX,
    FusionAxesAlignmentNZNXPY,
    FusionAxesAlignmentNZNYNX,
    FusionAxesAlignmentNZPXNY,
} FusionAxesAlignment;

static inline FusionVector FusionAxesSwap(const FusionVector sensor, const FusionAxesAlignment alignment)
{
    FusionVector result;
    switch (alignment)
    {
    case FusionAxesAlignmentPXPYPZ:
        break;
    case FusionAxesAlignmentPXNZPY:
        result.axis.x = +sensor.axis.x;
        result.axis.y = -sensor.axis.z;
        result.axis.z = +sensor.axis.y;
        return result;
    case FusionAxesAlignmentPXNYNZ:
        result.axis.x = +sensor.axis.x;
        result.axis.y = -sensor.axis.y;
        result.axis.z = -sensor.axis.z;
        return result;
    case FusionAxesAlignmentPXPZNY:
        result.axis.x = +sensor.axis.x;
        result.axis.y = +sensor.axis.z;
        result.axis.z = -sensor.axis.y;
        return result;
    case FusionAxesAlignmentNXPYNZ:
        result.axis.x = -sensor.axis.x;
        result.axis.y = +sensor.axis.y;
        result.axis.z = -sensor.axis.z;
        return result;
    case FusionAxesAlignmentNXPZPY:
        result.axis.x = -sensor.axis.x;
        result.axis.y = +sensor.axis.z;
        result.axis.z = +sensor.axis.y;
        return result;
    case FusionAxesAlignmentNXNYPZ:
        result.axis.x = -sensor.axis.x;
        result.axis.y = -sensor.axis.y;
        result.axis.z = +sensor.axis.z;
        return result;
    case FusionAxesAlignmentNXNZNY:
        result.axis.x = -sensor.axis.x;
        result.axis.y = -sensor.axis.z;
        result.axis.z = -sensor.axis.y;
        return result;
    case FusionAxesAlignmentPYNXPZ:
        result.axis.x = +sensor.axis.y;
        result.axis.y = -sensor.axis.x;
        result.axis.z = +sensor.axis.z;
        return result;
    case FusionAxesAlignmentPYNZNX:
        result.axis.x = +sensor.axis.y;
        result.axis.y = -sensor.axis.z;
        result.axis.z = -sensor.axis.x;
        return result;
    case FusionAxesAlignmentPYPXNZ:
        result.axis.x = +sensor.axis.y;
        result.axis.y = +sensor.axis.x;
        result.axis.z = -sensor.axis.z;
        return result;
    case FusionAxesAlignmentPYPZPX:
        result.axis.x = +sensor.axis.y;
        result.axis.y = +sensor.axis.z;
        result.axis.z = +sensor.axis.x;
        return result;
    case FusionAxesAlignmentNYPXPZ:
        result.axis.x = -sensor.axis.y;
        result.axis.y = +sensor.axis.x;
        result.axis.z = +sensor.axis.z;
        return result;
    case FusionAxesAlignmentNYNZPX:
        result.axis.x = -sensor.axis.y;
        result.axis.y = -sensor.axis.z;
        result.axis.z = +sensor.axis.x;
        return result;
    case FusionAxesAlignmentNYNXNZ:
        result.axis.x = -sensor.axis.y;
        result.axis.y = -sensor.axis.x;
        result.axis.z = -sensor.axis.z;
        return result;
    case FusionAxesAlignmentNYPZNX:
        result.axis.x = -sensor.axis.y;
        result.axis.y = +sensor.axis.z;
        result.axis.z = -sensor.axis.x;
        return result;
    case FusionAxesAlignmentPZPYNX:
        result.axis.x = +sensor.axis.z;
        result.axis.y = +sensor.axis.y;
        result.axis.z = -sensor.axis.x;
        return result;
    case FusionAxesAlignmentPZPXPY:
        result.axis.x = +sensor.axis.z;
        result.axis.y = +sensor.axis.x;
        result.axis.z = +sensor.axis.y;
        return result;
    case FusionAxesAlignmentPZNYPX:
        result.axis.x = +sensor.axis.z;
        result.axis.y = -sensor.axis.y;
        result.axis.z = +sensor.axis.x;
        return result;
    case FusionAxesAlignmentPZNXNY:
        result.axis.x = +sensor.axis.z;
        result.axis.y = -sensor.axis.x;
        result.axis.z = -sensor.axis.y;
        return result;
    case FusionAxesAlignmentNZPYPX:
        result.axis.x = -sensor.axis.z;
        result.axis.y = +sensor.axis.y;
        result.axis.z = +sensor.axis.x;
        return result;
    case FusionAxesAlignmentNZNXPY:
        result.axis.x = -sensor.axis.z;
        result.axis.y = -sensor.axis.x;
        result.axis.z = +sensor.axis.y;
        return result;
    case FusionAxesAlignmentNZNYNX:
        result.axis.x = -sensor.axis.z;
        result.axis.y = -sensor.axis.y;
        result.axis.z = -sensor.axis.x;
        return result;
    case FusionAxesAlignmentNZPXNY:
        result.axis.x = -sensor.axis.z;
        result.axis.y = +sensor.axis.x;
        result.axis.z = -sensor.axis.y;
        return result;
    }
    return sensor;
}

#endif
