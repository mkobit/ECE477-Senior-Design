#ifndef MATH_HELPERS_H
#define MATH_HELPERS_H

#include "kalman_shared.h"

void MHelpers_ConvertDegreesToRPYQuat(float roll, float pitch, float yaw, YAW_PITCH_ROLL_QUAT *quat);
void MHelpers_FindRotationAxis(YAW_PITCH_ROLL_QUAT *quat, ROTATION_AXIS *axis);
float MHelpers_FastInvSqrt(float x);
float MHelpers_DegreesToRadians(float degrees);

#endif