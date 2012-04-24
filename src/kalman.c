#include <p32xxxx.h>
#include <plib.h>

#include "kalman.h"
#include "math_helpers.h"
#include "imu.h"


void Kalman_MadgwickUpdate(imu_t *p_imu) {
  float gx, gy, gz;
  float ax, ay, az;
  
  // Acquire the readings from the IMU
  gx = ImuGetGyroRoll(p_imu);
  gy = ImuGetGyroPitch(p_imu);
  gz = ImuGetGyroYaw(p_imu);
  ax = ImuGetAccelX(p_imu);
  ay = ImuGetAccelY(p_imu);
  az = ImuGetAccelZ(p_imu);
}


void Kalman_MahonyUpdate(imu_t *p_imu) {
  float gx, gy, gz;
  float ax, ay, az;
  
  // Acquire the readings from the IMU
  gx = ImuGetGyroRoll(p_imu);
  gy = ImuGetGyroPitch(p_imu);
  gz = ImuGetGyroYaw(p_imu);
  ax = ImuGetAccelX(p_imu);
  ay = ImuGetAccelY(p_imu);
  az = ImuGetAccelZ(p_imu);
}
