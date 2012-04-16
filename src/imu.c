#include <p32xxxx.h>
#include <plib.h>
#include "imu.h"

static inline void ImuToggleSelector(imu_t* imu);

IMU_RESULT ImuInit(imu_t* imu, 
          unsigned int peripheral_clock_speed, 
					unsigned int i2c_speed, 
					char accel_resolution, 
					char accel_bandwidth, 
          char gyro_dlpf_lpf, 
          char gyro_sample_rate_div, 
          char gyro_power_mgmt_sel) {
                                    
  unsigned int actualClock;
  ACCEL_RESULT accel_init_result;
  GYRO_RESULT gyro_init_result;
  
  // Check clock rate for peripheral bus
  actualClock = I2CSetFrequency(i2c, peripheral_clock_speed, i2c_speed)
  if (actualClock - i2c_speed > i2c_speed / 10) {
    DBPRINTF("AccelInitI2C: Error, I2C clock frequency (%u) error exceeds 10%%n\n", actualClock); 
    return IMU_FAIL;
  }
  
  // Enable I2C
  I2CEnable(i2c, TRUE);
  
  // Init both modules of the imu
  accel_init_result = AccelInitI2C(i2c, accel_resolution, accel_bandwidth, &imu->accel_raw);
  gyro_init_result = GyroInit(i2c, gyro_dlpf_lpf, gyro_sample_rate_div, gyro_power_mgmt_sel);
  
  // Give a semi-random true/false to read accelerometer first  
  imu->updateAccelFirst = ReadCoreTimer() % 2 == 0;
  
  // Check if both succeeded in initializing
  if (accel_init_result == ACCEL_SUCCESS && gyro_init_result == GYRO_SUCCESS) {
    imu->isOn = TRUE;
    return IMU_SUCCESS;
  } else {
    // failure initializing, do not use this IMU
    imu->isOn = FALSE;
    DBPRINTF("ImuInit: Error, could not complete initialization. Results->(accel, gyro) = (%d, %d)\n", accel_init_result, gyro_init_result);
    return IMU_FAIL;
  }
}


IMU_RESULT ImuUpdate(imu_t *imu) {
  ACCEL_RESULT a_result;
  GYRO_RESULT g_result;
  
  // Check if device is on first
  if (!ImuIsOn(imu)) {
    DBPRINTF("ImuUpdate: Error, device with I2C=%d is not on\n", imu->i2c);
    return IMU_FAIL;
  }
  
  if (imu->updateAccelFirst) {
    a_result = AccelReadAllAxes(imu->i2c, &imu->accel_raw);
    g_result = GyroReadAllAxes(imu->i2c, &imu->gyro_raw);
  } else {
    g_result = GyroReadAllAxes(imu->i2c, &imu->gyro_raw);
    a_result = AccelReadAllAxes(imu->i2c, &imu->accel_raw);
  }
  
  // Toggle which is updated first for next time
  ImuToggleSelector(imu);
  
  // Check for any errors
 if (accel_init_result == ACCEL_SUCCESS && gyro_init_result == GYRO_SUCCESS) {
    return IMU_SUCCESS;
  } else {
    // failure initializing, do not use this IMU
    DBPRINTF("ImuUpdate: Error, could not update both accel and gyro at I2C=%d. Results->(accel, gyro) = (%d, %d)\n", imu->i2c, a_result, g_result);
    return IMU_FAIL;
  }
}


accel_raw_t *ImuGetRawAccel(imu_t *imu) {
  return &imu->accel_raw;
}


gyro_raw_t *ImuGetRawGyro(imu_t *imu) {
  return &imu->gyro_raw;
}


BOOL ImuIsOn(imu_t *imu) {
  return imu->isOn;
}


double ImuGetGyroTemp(imu_t *imu) {
  double gtemp;
  gtemp = GyroGetTemp(&imu->gyro_raw);
  return gtemp;
}


double ImuGetGyroX(imu_t *imu) {
  double gx;
  gx = GyroGetX(&imu->gyro_raw);
  return gx;
}


double ImuGetGyroY(imu_t *imu) {
  double gy;
  gy = GyroGetX(&imu->gyro_raw);
  return gy;
}


double ImuGetGyroZ(imu_t *imu) {
  double gz;
  gz = GyroGetX(&imu->gyro_raw);
  return gz;
}


double ImuGetAccelX(imu_t *imu) {
  double ax;
  ax = AccelGetZ(&imu->accel_raw);
  return ax;
}


double ImuGetAccelY(imu_t *imu) {
  double ay;
  ay = AccelGetZ(&imu->accel_raw);
  return ay;
}


double ImuGetAccelZ(imu_t *imu) {
  double az;
  az = AccelGetZ(&imu->accel_raw);
  return az;
}

static inline void ImuToggleSelector(imu_t* imu) {
  imu->readAccelFirst = !imu->readAccelFirst;
}