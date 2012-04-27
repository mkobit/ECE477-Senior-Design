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

/************************************************************************************************** 
  Function: 
    void MHelpers_FillInEuler(const float psi, const float theta, const float phi, EULER_ANGLES *const e)
  
  Author(s): 
    mkobit
  
  Summary: 
    fills in (e) Euler angles with roll, pitch, and yaw
  
  Description: 
    Same as summary
  
  Preconditions: 
    None
  
  Parameters: 
    const float psi -
    const float theta -
    const float phi -
    EULER_ANGLES *const e - Euler angles to be filled in
  
  Returns: 
    void
  
  Example: 
    <code>
    MHelpers_FillInEuler(43.0f, 0.0f, 75.0f, &e)
    </code>
  
  Conditions at Exit: 
    (e) filled in with euler angles
  
**************************************************************************************************/
void MHelpers_FillInEuler(const float psi, const float theta, const float phi, EULER_ANGLES *const e) {
  e->psi = psi;
  e->theta = theta;
  e->phi = phi;
}

/************************************************************************************************** 
  Function: 
    void MHelpers_FillInQuat(const float q0, const float q1, const float q2, const float q3, QUATERNION *const q)
  
  Author(s): 
    mkobit
  
  Summary: 
    Fills in quaternion (q) with values q0-q3
  
  Description: 
    Same as summary
  
  Preconditions: 
    None
  
  Parameters: 
    const float q0 - quaternion value q0
    const float q1 - quaternion value q1
    const float q2 - quaternion value q2
    const float q3 - quaternion value q3
    QUATERNION *const q - quaternion to be filled in
  
  Returns: 
    void
  
  Example: 
    <code>
    MHelpers_FillInQuat(1.0f, 0.0f, 0.0f, 0.0f, &q)
    </code>
  
  Conditions at Exit: 
    (q) quaternion filled in with values q0-q3
  
**************************************************************************************************/
void MHelpers_FillInQuat(const float q0, const float q1, const float q2, const float q3, QUATERNION *const q) {
  q->q0 = q0;
  q->q1 = q1;
  q->q2 = q2;
  q->q3 = q3;
}

//TODO doc
void MHelpers_FillInYPR(const float yaw, const float pitch, const float roll, YPR *const ypr) {
  ypr->yaw = yaw;
  ypr->pitch = pitch;
  ypr->roll = roll;
}

/************************************************************************************************** 
  Function: 
    void MHelpers_EulerToQuaternion(const EULER_ANGLES *const source, QUATERNION *const dest)
  
  Author(s): 
    mkobit
  
  Summary: 
    Converts the (source) Euler angles into the (dest) quaternion
  
  Description: 
    Uses equations from Source: http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  
  Preconditions: 
    (source) has vaild Euler angles
  
  Parameters: 
    const EULER_ANGLES *const source - source Euler angles to use
    QUATERNION *const dest - destination quaternion to convert values into
  
  Returns: 
    void
  
  Example: 
    <code>
    MHelpers_EulerToQuaternion(&euls, &quat)
    </code>
  
  Conditions at Exit: 
    (dest) has converted quaternion from the (source) Euler angles
  
**************************************************************************************************/
/*void MHelpers_EulerToQuaternion(const EULER_ANGLES *const source, QUATERNION *const dest) {
  // local variables for readibility
  const float psi = source->psi;
  const float theta = source->theta;
  const float phi = source->phi;
  
  // Local variables for faster conversion
  const float cr = cosf(roll / 2);  // cos of roll
  const float sr = sinf(roll / 2);  // sin of roll
  const float cp = cosf(pitch / 2); // cos of pitch
  const float sp = sinf(pitch / 2); // sin of pitch
  const float cy = cosf(yaw / 2);   // cos of yaw
  const float sy = sinf(yaw / 2);   // sin of yaw
  
  // calculations for quaternion conversion
  dest->q0 = cr * cp * cy + sr * sp * sy;
  dest->q1 = sr * cp * cy - cr * sp * sy;
  dest->q2 = cr * sp * cy + sr * cp * sy;
  dest->q3 = cr * cp * sy - sr * sp * cy;
} */

/************************************************************************************************** 
  Function: 
    void MHelpers_QuaternionToEuler(const QUATERNION *const source, EULER_ANGLES *const dest)
  
  Author(s): 
    mkobit
  
  Summary: 
    Converts the (source) quaternion into the (dest) euler angles
  
  Description: 
    Uses equations from Source: http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  
  Preconditions: 
    (source) is a valid quaternion
  
  Parameters: 
    const QUATERNION *const source - source qauternion to use
    EULER_ANGLES *const dest - destination Euler angles to convert values into
  
  Returns: 
    void
  
  Example: 
    <code>
    MHelpers_EulerToQuaternion(&quat, &euls)
    </code>
  
  Conditions at Exit: 
    (dest) has converted Euler angles from the (source) quaternion
  
**************************************************************************************************/
void MHelpers_QuaternionToEuler(const QUATERNION *const source, EULER_ANGLES *const dest) {
  // local variables for readibility
  const float q0 = source->q0;
  const float q1 = source->q1;
  const float q2 = source->q2;
  const float q3 = source->q3;

  // Local values
  float psi, theta, phi;

  // calculations for euler conversion
  psi = MHelpers_RadiansToDegrees(atan2f(2 * q1 * q2 - 2 * q0 * q3, 2 * q0*q0 + 2 * q1*q1 - 1)); // psi
  theta = MHelpers_RadiansToDegrees(-asinf(2 * q1 * q3 + 2 * q0 * q2)); // theta
  phi = MHelpers_RadiansToDegrees(atan2f(2 * q2 * q3 - 2 * q0 * q1, 2 * q0*q0 + 2 * q3*q3 - 1)); // phi
  // TESTING CALC
  //psi = MHelpers_RadiansToDegrees(atan2f(2 * (q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)));
  //theta = MHelpers_RadiansToDegrees(asinf(2 * (q0 * q2 - q3*q1)));
  //phi = MHelpers_RadiansToDegrees(atan2f(2 * (q0*q3 + q1*q2), 1 - 2 * (q2*q2 +q3*q3)));

  MHelpers_FillInEuler(psi, theta, phi, dest);
} 

void MHelpers_QuaternionToYPR(const QUATERNION *const source, YPR *const ypr) {
  float gx, gy, gz; // estimated gravity direction
  float yaw, pitch, roll;

  // local variables for readibility
  const float q0 = source->q0;
  const float q1 = source->q1;
  const float q2 = source->q2;
  const float q3 = source->q3;

  gx = 2 * (q1*q3 - q0*q2);
  gy = 2 * (q0*q1 + q2*q3);
  gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  yaw = MHelpers_RadiansToDegrees(atan2f(2 * q1 * q2 - 2 * q0 * q3, 2 * q0*q0 + 2 * q1 * q1 - 1));
  pitch = MHelpers_RadiansToDegrees(atanf(gx / sqrtf(gy*gy + gz*gz)));
  roll = MHelpers_RadiansToDegrees(atanf(gy / sqrtf(gx*gx + gz*gz)));
  MHelpers_FillInYPR(yaw, pitch, roll, ypr);
  
}

/************************************************************************************************** 
  Function: 
    float MHelpers_FastInvSqrt(const float x)
  
  Author(s): 
    mkobit
  
  Summary: 
    Returns 1/sqrt(x)
  
  Description: 
    Uses formula from Source: http://en.wikipedia.org/wiki/Fast_inverse_square_root
  
  Preconditions: 
    None
  
  Parameters: 
    const float x - number to find 1/sqrt of
  
  Returns: 
    float y - 1/sqrt(x)
  
  Example: 
    <code>
    MHelpers_FastInvSqrt(9.0f)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
float MHelpers_FastInvSqrt(const float x) {
  volatile const float halfx = 0.5f * x;
  volatile float y = x;
  volatile long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/************************************************************************************************** 
  Function: 
    float MHelpers_DegreesToRadians(const float degrees)
  
  Author(s): 
    mkobit
  
  Summary: 
    Converts degrees to radians
  
  Description: 
    Use PI from math library and returns the radians in floats
  
  Preconditions: 
    None
  
  Parameters: 
    const float degrees - degrees to be converte
  
  Returns: 
    float radians - radians converted from degrees
  
  Example: 
    <code>
    float rads = MHelpers_DegreesToRadians(85.0f)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
float MHelpers_DegreesToRadians(const float degrees) {
  return degrees * M_PI / 180.0f;
}

float MHelpers_RadiansToDegrees(const float radians) {
  return radians * 180.0f / M_PI;
}