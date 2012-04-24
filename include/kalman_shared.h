#ifndef KALMAN_SHARED_H
#define KALMAN_SHARED_H

// Used for Madgwick filtering
#define BETADEF		0.1f		// 2 * proportional gain

// Used for Mahony filtering
#define TWOKPDEF	(2.0f * 0.5f)	// 2 * proportional gain
#define TWOKIDEF	(2.0f * 0.0f)	// 2 * integral gain

// Structure of euler angles that could possibly be used
typedef struct EULER_ANGLES {
  float roll;   // rotation about the X-axis
  float pitch;  // rotation about the Y-axis
  float yaw;  // rotation about the Z-axis
} EULER_ANGLES;

// Quaternion of sensor frame relative to auxiliary frame
typedef struct QUATERNION  {
  float q0;
  float q1;
  float q2;
  float q3;
} QUATERNION;

typedef struct ROTATION_AXIS {
  float axis_x;
  float axis_y;
  float axis_z;
} ROTATION_AXIS;


#endif