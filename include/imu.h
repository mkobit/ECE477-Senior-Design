#ifndef IMU_H
#define IMU_H

#include "itg3200.h"
#include "adxl345.h"

#define ImuGetGyroRoll(p_imu) (ImuGetGyroX(p_imu))
#define ImuGetGyroPitch(p_imu) (ImuGetGyroY(p_imu))
#define ImuGetGyroYaw(p_imu) (ImuGetGyroZ(p_imu))

typedef UINT8 imu_id;

typedef enum {
  IMU_SUCCESS = 0,
  IMU_FAIL
} IMU_RESULT;

typedef struct imu_t {
  I2C_MODULE i2c_module;
  BOOL isOn;
  BOOL updateAccelFirst;
  accel_raw_t accel_raw;
  gyro_raw_t gyro_raw;
  imu_id id;
} imu_t;

IMU_RESULT ImuInit(imu_t* imu, 
          I2C_MODULE i2c, 
          UINT peripheral_clock_speed, 
          UINT i2c_speed, 
          UINT8 accel_range,
          UINT8 accel_bandwidth,
          UINT8 gyro_dlpf_lpf,
          UINT8 gyro_sample_rate_div, 
          UINT8 gyro_power_mgmt_sel);
IMU_RESULT ImuUpdate(imu_t *p_imu);
accel_raw_t *ImuGetRawAccel(imu_t *p_imu);
gyro_raw_t *ImuGetRawGyro(imu_t *p_imu);
BOOL ImuIsOn(imu_t *p_imu);
float ImuGetGyroTemp(imu_t *p_imu);
float ImuGetGyroX(imu_t *p_imu);       // X === Roll
float ImuGetGyroY(imu_t *p_imu);       // Y === Pitch
float ImuGetGyroZ(imu_t *p_imu);       // Z === Yaw
float ImuGetAccelX(imu_t *p_imu);
float ImuGetAccelY(imu_t *p_imu);
float ImuGetAccelZ(imu_t *p_imu);

#endif