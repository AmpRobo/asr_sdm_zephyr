/**
 * @file FusionCalibration.h
 */

#ifndef _FUSION_CALIBRATION_H_
#define _FUSION_CALIBRATION_H_

#include "math_utils.h"

static inline FusionVector FusionCalibrationInertial(const FusionVector uncalibrated, const FusionMatrix misalignment,
                                                     const FusionVector sensitivity, const FusionVector offset)
{
    return FusionMatrixMultiplyVector(
        misalignment, FusionVectorHadamardProduct(fusion_vector_subtract(uncalibrated, offset), sensitivity));
}

static inline FusionVector FusionCalibrationMagnetic(const FusionVector uncalibrated, const FusionMatrix softIronMatrix,
                                                     const FusionVector hardIronOffset)
{
    return FusionMatrixMultiplyVector(softIronMatrix, fusion_vector_subtract(uncalibrated, hardIronOffset));
}

#endif
