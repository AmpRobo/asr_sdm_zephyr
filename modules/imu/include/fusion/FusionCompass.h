/**
 * @file FusionCompass.h
 */

#ifndef _FUSION_COMPASS_H_
#define _FUSION_COMPASS_H_

#include "FusionConvention.h"
#include "math_utils.h"

float FusionCompassCalculateHeading(const FusionConvention convention, const FusionVector accelerometer,
                                    const FusionVector magnetometer);

#endif
