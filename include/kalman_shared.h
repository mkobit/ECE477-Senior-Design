#ifndef KALMAN_SHARED_H
#define KALMAN_SHARED_H

// Used for Madgwick filtering
#define BETADEF		0.1f		// 2 * proportional gain

// Used for Mahony filtering
#define TWOKPDEF	(2.0f * 0.5f)	// 2 * proportional gain
#define TWOKIDEF	(2.0f * 0.0f)	// 2 * integral gain

// Structure of euler angles that could possibly be used
typedef struct EULER_ANGLES {
  volatile float phi;   
  volatile float theta;  
  volatile float psi;  
} EULER_ANGLES;

// Quaternion of sensor frame relative to auxiliary frame
typedef struct QUATERNION  {
  volatile float q0;
  volatile float q1;
  volatile float q2;
  volatile float q3;
} QUATERNION;

typedef struct ROTATION_AXIS {
  float axis_x;
  float axis_y;
  float axis_z;
} ROTATION_AXIS;

typedef struct YPR {
  volatile float yaw;   // rotation about the X-axis
  volatile float pitch; // rotation about the Y-axis
  volatile float roll;  // rotation about the Z-axis
} YPR;

#endif