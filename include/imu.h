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

IMU_RESULT ImuInit(imu_t *const imu, 
          const I2C_MODULE i2c, 
          const UINT peripheral_clock_speed, 
          const UINT i2c_speed, 
          const UINT8 accel_range,
          const UINT8 accel_bandwidth,
          const UINT8 gyro_dlpf_lpf,
          const UINT8 gyro_sample_rate_div, 
          const UINT8 gyro_power_mgmt_sel);
IMU_RESULT ImuUpdate(imu_t *const p_imu);
accel_raw_t *ImuGetRawAccel(const imu_t *const p_imu);
gyro_raw_t *ImuGetRawGyro(const imu_t *const p_imu);
BOOL ImuIsOn(const imu_t *const p_imu);
float ImuGetGyroTemp(const imu_t *const p_imu);
float ImuGetGyroX(const imu_t *const p_imu);       // X === Roll
float ImuGetGyroY(const imu_t *const p_imu);       // Y === Pitch
float ImuGetGyroZ(const imu_t *const p_imu);       // Z === Yaw
float ImuGetAccelX(const imu_t *const p_imu);
float ImuGetAccelY(const imu_t *const p_imu);
float ImuGetAccelZ(const imu_t *const p_imu);

#endif