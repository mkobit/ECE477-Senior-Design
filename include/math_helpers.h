#ifndef MATH_HELPERS_H
#define MATH_HELPERS_H

#include "kalman_shared.h"

#ifndef PI
#define PI 3.1415927f
#endif
//void MHelpers_ConvertDegreesToRPYQuat(float roll, float pitch, float yaw, YAW_PITCH_ROLL_QUAT *quat);
//void MHelpers_FindRotationAxis(YAW_PITCH_ROLL_QUAT *quat, ROTATION_AXIS *axis);

void MHelpers_FillInEuler(const float roll, const float pitch, const float yaw, EULER_ANGLES *const e);
void MHelpers_FillInQuat(const float q0, const float q1, const float q2, const float q3, QUATERNION *const q);
void MHelpers_EulerToQuaternion(const EULER_ANGLES *const source, QUATERNION *const dest);
void MHelpers_QuaternionToEuler(const QUATERNION *const source, EULER_ANGLES *const dest); 
float MHelpers_FastInvSqrt(const float x);
float MHelpers_DegreesToRadians(const float degrees);

#endif