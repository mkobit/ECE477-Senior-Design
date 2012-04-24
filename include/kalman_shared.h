#ifndef KALMAN_SHARED_H
#define KALMAN_SHARED_H

typedef struct EULER_ANGLES {
  float roll;   // rotation about the X-axis
  float pitch;  // rotation about the Y-axis
  float yaw;  // rotation about the Z-axis
} EULER_ANGLES;

// quaternion of sensor frame relative to auxiliary frame
typedef struct SENSOR_FRAME_QUAT {
  float q0;
  float q1;
  float q2;
  float q3;
} YAW_PITCH_ROLL_QUAT;

typedef struct ROTATION_AXIS {
  float axis_x;
  float axis_y;
  float axis_z;
} ROTATION_AXIS;


#endif