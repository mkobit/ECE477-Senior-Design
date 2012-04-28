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
  accel_t accel;
  gyro_t gyro;
  I2C_MODULE i2c_module;
  BOOL isOn;
  BOOL updateAccelFirst;
  imu_id id;
} imu_t;

IMU_RESULT ImuInit(imu_t *const p_imu,
          const I2C_MODULE i2c, 
          const UINT peripheral_clock_speed, 
          const UINT i2c_speed,
          const UINT accel_addr,
          const UINT8 accel_range,
          const UINT8 accel_bandwidth,
          const UINT gyro_addr,
          const UINT8 gyro_dlpf_lpf,
          const UINT8 gyro_sample_rate_div, 
          const UINT8 gyro_power_mgmt_sel);
IMU_RESULT ImuUpdate(imu_t *const p_imu);
accel_t *ImuGetAccel(imu_t *const p_imu);
accel_raw_t *ImuGetRawAccel(imu_t *const p_imu);
gyro_t *ImuGetGyro(imu_t *const p_imu);
gyro_raw_t *ImuGetRawGyro(imu_t *const p_imu);
IMU_RESULT ImuCalibrate(imu_t *const p_imu, BOOL calGyro, BOOL calAccel, int samplesToTake, UINT ms_delay);
BOOL ImuIsOn(const imu_t *const p_imu);
void ImuSetID(imu_t *const p_imu, const imu_id id);
imu_id ImuGetId(const imu_t *const p_imu);
void ImuResetI2CBus(const imu_t *p_imu);
float ImuGetGyroTemp(imu_t *const p_imu);
float ImuGetGyroX(imu_t *const p_imu);       // X === Roll
float ImuGetGyroY(imu_t *const p_imu);       // Y === Pitch
float ImuGetGyroZ(imu_t *const p_imu);       // Z === Yaw
float ImuGetAccelX(imu_t *const p_imu);
float ImuGetAccelY(imu_t *const p_imu);
float ImuGetAccelZ(imu_t *const p_imu);

#endif