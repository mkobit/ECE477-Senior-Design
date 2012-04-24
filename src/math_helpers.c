#include <p32xxxx.h>
#include <plib.h>
#include "math_helpers.h"

// Floating point support for various calculations
#include <math.h>

// ROLL IS \PHI
// PITCH IS \THETA
// YAW IS \PSI

/* more pages to use when in lab
Kalman - http://www.sharprobotica.com/2010/10/introduction-to-the-kalman-filter-part-ii/
IMU guide *best - http://www.starlino.com/imu_guide.html
Math lib for pic32 - http://tigcc.ticalc.org/doc/math.html
Euler angles and quats - http://forum.onlineconversion.com/showthread.php?t=5408
*/

/*void MHelpers_ConvertDegreesToRPYQuat(float roll, float pitch, float yaw, YAW_PITCH_ROLL_QUAT *quat) {
  // Source: http://forum.onlineconversion.com/showthread.php?t=5408
  quat->alpha = 2 * acosf( cosf(roll / 2) * cosf(pitch / 2) * cosf(yaw / 2) + \
      sinf(roll / 2) * sinf(pitch / 2 ) * sinf(yaw / 2) );
  quat->beta_x = acosf( (sinf(roll / 2) * cosf(pitch / 2) * cosf(yaw/2) - \
      cosf(roll / 2) * sinf(pitch / 2) * sinf(yaw / 2) ) / sinf(quat->alpha / 2));
  quat->beta_y = acosf( (cosf(roll / 2) * sinf(pitch / 2) * cosf(yaw/2) + \
      sinf(roll / 2) * cosf(pitch / 2) * sinf(yaw / 2) ) / sinf(quat->alpha / 2));
  quat->beta_z = acosf( (cosf(roll / 2) * cosf(pitch / 2) * sinf(yaw / 2) - \
      sinf(roll / 2) * sinf(pitch / 2) * cosf(yaw / 2) ) / sinf(quat->alpha / 2));
}*/

/*void MHelpers_FindRotationAxis(YAW_PITCH_ROLL_QUAT *quat, ROTATION_AXIS *axis) {
  // Source: http://forum.onlineconversion.com/showthread.php?t=5408
  axis->axis_x = cosf(quat->beta_x);
  axis->axis_y = cosf(quat->beta_y);
  axis->axis_z = cosf(quat->beta_z);
}*/



float MHelpers_FastInvSqrt(const float x) {
  // Source: http://en.wikipedia.org/wiki/Fast_inverse_square_root
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

float MHelpers_DegreesToRadians(const float degrees) {
  return degrees * (float) (PI / 180);
}