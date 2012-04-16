#ifndef IMU_H
#define IMU_H

#include "itg3200.h"
#include "adxl345.h"

typedef enum {
  IMU_SUCCESS = 0,
  IMU_FAIL
} IMU_RESULT;

typedef struct imu_t {
  I2C_MODULE i2c_module;
  BOOL isOn;
  accel_raw_t accel_raw;
  gyro_raw_t gyro_raw;
  BOOL updateAccelFirst;
} imu_t;

IMU_RESULT ImuInit(imu_t* imu, 
					unsigned int peripheral_clock_speed, 
					unsigned int i2c_speed, 
					char accel_resolution, 
					char accel_bandwidth, 
					char gyro_dlpf_lpf, 
					char gyro_sample_rate_div, 
					char gyro_power_mgmt_sel);
IMU_RESULT ImuUpdate(imu_t *imu);
accel_raw_t *ImuGetRawAccel(imu_t *imu);
gyro_raw_t *ImuGetRawGyro(imu_t *imu);
BOOL ImuIsOn(imu_t *imu);
double ImuGetGyroTemp(imu_t *imu);
double ImuGetGyroX(imu_t *imu);
double ImuGetGyroY(imu_t *imu);
double ImuGetGyroZ(imu_t *imu);
double ImuGetAccelX(imu_t *imu);
double ImuGetAccelY(imu_t *imu);
double ImuGetAccelZ(imu_t *imu);

#endif