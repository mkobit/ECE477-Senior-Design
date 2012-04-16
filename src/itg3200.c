#include <p32xxxx.h>
#include <plib.h>
#include "itg3200.h"
#include "i2c_shared.h"

GYRO_RESULT GyroWrite(I2C_MODULE i2c, char i2c_reg, BYTE data) {
  if (I2CShared_Write(i2c, GYRO_WRITE, i2c_reg, data)) {
    return GYRO_SUCCESS;
  } else {
    return GYRO_FAIL;
  }
}


GYRO_RESULT GyroRead(I2C_MODULE i2c, char i2c_reg, char *buffer) {
  if (I2CShared_Read(i2c, GYRO_READ, i2c_reg, buffer)) {
    return GYRO_SUCCESS;
  } else {
    return GYRO_FAIL;
  }
}


// TODO get functions
double GyroGetX(gyro_raw_t *readings) {
  return 0.0;
}


double GyroGetY(gyro_raw_t *readings) {
  return 0.0;
}


double GyroGetZ(gyro_raw_t *readings) {
  return 0.0;
}
