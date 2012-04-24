#ifndef KALMAN_H
#define KALMAN_H

#include "imu.h"

void Kalman_MadgwickUpdate(const imu_t *const p_imu, KALMAN_STATE *const q, const float sampleFreq);
void Kalman_MahonyUpdate(const imu_t *const p_imu, KALMAN_STATE_MADGWICK *madgwick, const float sampleFreq)

typedef QUATERNION KALMAN_STATE;

#endif