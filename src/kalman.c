#include <p32xxxx.h>
#include <plib.h>

#include "kalman.h"
#include "math_helpers.h"
#include "imu.h"

/************************************************************************************************** 
  Function: 
    void Kalman_MadgwickUpdate(const imu_t *const p_imu, KALMAN_STATE_MADGWICK *const kmadg, const float sampleFreq)
  
  Author(s): 
    mkobit, taken from Source: http://www.x-io.co.uk/node/8#open_source_imu_and_ahrs_algorithms
    
  Summary: 
    Uses Madgwick algorithm to updates the Kalman filter state by using the readings from the IMU and the frequency at which the data is sampled
  
  Description: 
    Implementation of Madgwick's IMU and AHRS algorithms.
    See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
  
  Preconditions: 
    IMU initted and updated to most recent values
  
  Parameters: 
    const imu_t *const p_imu - pointer to the IMU updated to use for this Kalman state
    KALMAN_STATE_MADGWICK *const kmadg - pointer to the kalman state to be updated
    const float sampleFreq - sample frequency in Hz
  
  Returns: 
    void
  
  Example: 
    <code>
    IMUUpdate(&imu)
    Kalman_MadgwickUpdate(&imu, &k, 100.0f)
    </code>
  
  Conditions at Exit: 
    Kalman updated to newest state
  
**************************************************************************************************/
void Kalman_MadgwickUpdate(imu_t *const p_imu, KALMAN_STATE_MADGWICK *const kmadg, const float sampleFreq) {
  // Local gyro and accel values
  float gx, gy, gz;
  float ax, ay, az;

  // A local pointer to the quaternion
  QUATERNION *q;
  
  // Store quaternion variables locally for readability
  float q0, q1, q2, q3;
  
  // Variables used by Madgwick Kalman algorithm
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
  
  // Acquire the readings from the IMU
  // acceleration in terms of g's
  ax = ImuGetAccelX(p_imu);
  ay = ImuGetAccelY(p_imu);
  az = ImuGetAccelZ(p_imu);
  // Gyro filtering uses radians, so conversion needed
  gx = MHelpers_DegreesToRadians(ImuGetGyroRoll(p_imu));
  gy = MHelpers_DegreesToRadians(ImuGetGyroPitch(p_imu));
  gz = MHelpers_DegreesToRadians(ImuGetGyroYaw(p_imu));
  
  // Store locally for readability
  q = &kmadg->q;
  q0 = q->q0;
  q1 = q->q1;
  q2 = q->q2;
  q3 = q->q3;
  
  
  // START MADGWICK KALMAN ALGORITHM
  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
  
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
  
    // Normalise accelerometer measurement
    recipNorm = MHelpers_FastInvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;
    
    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = MHelpers_FastInvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    
    // Apply feedback step
    qDot1 -= BETADEF * s0;
    qDot2 -= BETADEF * s1;
    qDot3 -= BETADEF * s2;
    qDot4 -= BETADEF * s3;
  }
  
  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = MHelpers_FastInvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  // END MADGWICK ALGORITHM
  
  // Store variables back into state
  MHelpers_FillInQuat(q0, q1, q2, q3, q);
}

/************************************************************************************************** 
  Function: 
    void Kalman_MadgwickInit(KALMAN_STATE_MADGWICK *const kmadg)
  
  Author(s): 
    mkobit, taken from Source: http://www.x-io.co.uk/node/8#open_source_imu_and_ahrs_algorithms
  
  Summary: 
    Initializes the Kalman state for Madgwick filter usage
  
  Description: 
    Same as summary
  
  Preconditions: 
    None
  
  Parameters: 
    KALMAN_STATE_MADGWICK *const kmadg - pointer to the kalman state to be initialized
  
  Returns: 
    void
  
  Example: 
    <code>
    KALMAN_STATE k
    Kalman_MadgwickInit(&k)
    </code>
  
  Conditions at Exit: 
    Kalman filter put into an initial state
  
**************************************************************************************************/
void Kalman_MadgwickInit(KALMAN_STATE_MADGWICK *const kmadg) {
  // Initilization taken from http://www.x-io.co.uk/node/8#open_source_imu_and_ahrs_algorithms
  MHelpers_FillInQuat(1.0f, 0.0f, 0.0f, 0.0f, &kmadg->q);
}

/************************************************************************************************** 
  Function: 
    void Kalman_MahonyUpdate(const imu_t *const p_imu, KALMAN_STATE_MAHONY *const kmah, const float sampleFreq
  
  Author(s): 
    mkobit, taken from Source: http://www.x-io.co.uk/node/8#open_source_imu_and_ahrs_algorithms
  
  Summary: 
    Uses Mahony algorithm to updates the Kalman filter state by using the readings from the IMU and the frequency at which the data is sampled
  
  Description: 
    Madgwick's implementation of Mayhony's AHRS algorithm.
    See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
  
  Preconditions: 
    IMU initialized and updated to most recent values
  
  Parameters: 
    const imu_t *const p_imu - pointer to the IMU updated to use for this Kalman state
    KALMAN_STATE_MAHONY *const kmah - pointer to the kalman state to be updated
    const float sampleFreq - sample frequency in Hz
  
  Returns: 
    void
  
  Example: 
    <code>
    IMUUpdate(&imu)
    Kalman_MahonyUpdate(&imu, &k, 100.0f)
    </code>
  
  Conditions at Exit: 
    Kalman updated to newest state
  
**************************************************************************************************/
void Kalman_MahonyUpdate(imu_t *const p_imu, KALMAN_STATE_MAHONY *const kmah, const float sampleFreq) {
  // Local gyro and accel values
  float gx, gy, gz;
  float ax, ay, az;

  // Local pointer to the quaternion
  QUATERNION *q;

  // Local integral error terms scaled by Ki
  float integralFBx;
  float integralFBy;
  float integralFBz;
  
  // Local quaternion variables for readability
  float q0, q1, q2, q3;
  
  // Variables used by Mahony Kalman algorithm
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  
  
  // Acquire the readings from the IMU

  // acceleration in terms of g's
  ax = ImuGetAccelX(p_imu);
  ay = ImuGetAccelY(p_imu);
  az = ImuGetAccelZ(p_imu);
  // Gyro filtering uses radians, so conversion needed
  gx = MHelpers_DegreesToRadians(ImuGetGyroRoll(p_imu));
  gy = MHelpers_DegreesToRadians(ImuGetGyroPitch(p_imu));
  gz = MHelpers_DegreesToRadians(ImuGetGyroYaw(p_imu));

  // Store variables locally for readability
  q = &kmah->q;
  integralFBx = kmah->integralFBx;
  integralFBy = kmah->integralFBy;
  integralFBz = kmah->integralFBz;
 
  q0 = q->q0;
  q1 = q->q1;
  q2 = q->q2;
  q3 = q->q3;
  
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = MHelpers_FastInvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f && TWOKIDEF > 0.0f) {
      integralFBx += TWOKIDEF * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += TWOKIDEF * halfey * (1.0f / sampleFreq);
      integralFBz += TWOKIDEF * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f;  // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += TWOKPDEF * halfex;
    gy += TWOKPDEF * halfey;
    gz += TWOKPDEF * halfez;
  }
  
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));    // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = MHelpers_FastInvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  // END MAHONY ALGORITHM
  
  // Store variables back into state
  MHelpers_FillInQuat(q0, q1, q2, q3, q);

  // Store integral feedbacks back into state
  kmah->integralFBx = integralFBx;
  kmah->integralFBy = integralFBy;
  kmah->integralFBz = integralFBz;
}

/************************************************************************************************** 
  Function: 
    void Kalman_MahonyInit(KALMAN_STATE_MAHONY *const kmah)
  
  Author(s): 
    mkobit, taken from Source: http://www.x-io.co.uk/node/8#open_source_imu_and_ahrs_algorithms
  
  Summary: 
    Initializes the Kalman state for Mahony filter usage
  
  Description: 
    Same as summary
  
  Preconditions: 
    None
  
  Parameters: 
    KALMAN_STATE_MAHONY *const kmah - pointer to the kalman state to be initialized
  
  Returns: 
    void
  
  Example: 
    <code>
    KALMAN_STATE k
    Kalman_MahonyInit(&k)
    </code>
  
  Conditions at Exit: 
    Kalman filter put into an initial state
  
**************************************************************************************************/
void Kalman_MahonyInit(KALMAN_STATE_MAHONY *const kmah) {
  // Initilization taken from http://www.x-io.co.uk/node/8#open_source_imu_and_ahrs_algorithms
  MHelpers_FillInQuat(1.0f, 0.0f, 0.0f, 0.0f, &kmah->q);
  kmah->integralFBx = 0.0f;
  kmah->integralFBy = 0.0f;
  kmah->integralFBz = 0.0f;
}
