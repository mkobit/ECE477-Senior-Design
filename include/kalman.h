#ifndef KALMAN_H
#define KALMAN_H

#include "imu.h"

void Kalman_MadgwickUpdate(imu_t *p_imu);
void Kalman_MahonyUpdate(imu_t *p_imu);

typedef struct kalman_state {
} kalman_state;

#endif