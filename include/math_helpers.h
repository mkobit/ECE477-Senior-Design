#ifndef MATH_HELPERS_H
#define MATH_HELPERS_H

typedef struct EULER_ANGLES {
  float roll;   // rotation about the X-axis
  float pitch;  // rotation about the Y-axis
  float yaw;  // rotation about the Z-axis
} EULER_ANGLES;

typedef struct YAW_PITCH_ROLL_QUAT {
  float alpha;
  float beta_x;
  float beta_y;
  float beta_z;
} YAW_PITCH_ROLL_QUAT;

typedef struct ROTATION_AXIS {
  float axis_x;
  float axis_y;
  float axis_z;
} ROTATION_AXIS;

void Tracking_ConvertDegreesToRPYQuat(float roll, float pitch, float yaw, YAW_PITCH_ROLL_QUAT *quat);
void Tracking_FindRotationAxis(YAW_PITCH_ROLL_QUAT *quat, ROTATION_AXIS *axis);
float Tracking_FastInvSqrt(float x);
float Tracking_DegreesToRadians(float degrees);

#endif