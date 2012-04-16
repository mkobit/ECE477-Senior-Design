#ifndef IMU_H
#define IMU_H

#include "itg3200.h"
#include "adxl345.h"

typedef struct imu {
  I2C_MODULE i2c_module;
  accel_raw_readings accel;
  gyro_raw_readings gyro;
} imu_readings;

#endif