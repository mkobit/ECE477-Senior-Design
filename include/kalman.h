#ifndef KALMAN_H
#define KALMAN_H

#include "kalman_shared.h"
#include "imu.h"


// Madgwick filter variables
typedef struct KALMAN_STATE_MADGWICK {
  QUATERNION q;
} KALMAN_STATE_MADGWICK;

// Mahony filter variables
typedef struct KALMAN_STATE_MAHONY {
  QUATERNION q;
  // integral error terms scaled by Ki
  volatile float integralFBx;
  volatile float integralFBy;
  volatile float integralFBz;
}KALMAN_STATE_MAHONY;

void Kalman_MadgwickInit(KALMAN_STATE_MADGWICK *const q);
void Kalman_MadgwickUpdate(imu_t *const p_imu, KALMAN_STATE_MADGWICK *const q, const float sampleFreq);
void Kalman_MahonyInit(KALMAN_STATE_MAHONY *const q);
void Kalman_MahonyUpdate(imu_t *const p_imu, KALMAN_STATE_MAHONY *q, const float sampleFreq);

#endif