#ifndef KALMAN_SHARED_H
#define KALMAN_SHARED_H

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


#endif