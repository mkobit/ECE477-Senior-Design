#include <p32xxxx.h>
#include <plib.h>
#include "adxl345.h"
#include "i2c_shared.h"

static double SCALES[4] = { .0039, .0078, .0156, .0312};  // from datasheet

static void AccelStopTx(I2C_MODULE i2c);

ACCEL_RESULT AccelInitI2C(I2C_MODULE i2c, char resolution, char bandwidth, accel_raw_t *readings) {

  // I2C should already be enabled
  
  // Write configurations to it
  // Put accel in MEASURE mode
  if (AccelWrite(i2c, ACCEL_POWER_CTL, ACCEL_MEASURE) == ACCEL_FAIL) {
    DBPRINTF("AccelInitI2C: Error, could not write to ACCEL_POWER_CTL to I2C=%d\n", i2c); 
    return ACCEL_FAIL
  }
  
  // Set Data Format
  if (AccelWrite(i2c, ACCEL_DATA_FORMAT, resolution) == ACCEL_FAIL) {
    DBPRINTF("AccelInitI2C: Error, could not write to ACCEL_POWER_CTL to I2C=%d\n", i2c); 
    return ACCEL_FAIL
  }

  // Set Bandwidth
  if (AccelWrite(i2c, ACCEL_BW_RATE, bandwidth) == ACCEL_FAIL) {
    DBPRINTF("AccelInitI2C: Error, could not write to ACCEL_POWER_CTL to I2C=%d\n", i2c); 
    return ACCEL_FAIL
  }
  
  // Determine which scaling to use when getting the values
  switch(resolution) {
    case ACCEL_RANGE_2G: readings->scale_ind = ACCEL_SCALE_2G; break;
    case ACCEL_RANGE_4G: readings->scale_ind = ACCEL_SCALE_4G; break;
    case ACCEL_RANGE_8G: readings->scale_ind = ACCEL_SCALE_8G; break;
    case ACCEL_RANGE_16G: readings->scale_ind = ACCEL_SCALE_16G; break;
    default: DBPRINTF("AccelInitI2C: Error, 0x%c not a valid range for data format for adxl345\n", resolution); return ACCEL_FAIL;
  }
  
  return ACCEL_SUCCESS;
}


ACCEL_RESULT AccelWrite(I2C_MODULE i2c, char i2c_reg, BYTE data) {
  if (I2CShared_WriteByte(i2c, ACCEL_WRITE, i2c_reg, data)) {
    return ACCEL_SUCCESS;
  } else {
    return ACCEL_FAIL;
  }
}


ACCEL_RESULT AccelRead(I2C_MODULE i2c, char i2c_reg, char *buffer) {
  if (I2CShared_ReadByte(i2c, ACCEL_WRITE, ACCEL_READ, i2c_reg, buffer)) {
    return ACCEL_SUCCESS;
  } else {
    return ACCEL_FAIL;
  }
}


ACCEL_RESULT AccelReadAllAxes(I2C_MODULE i2c, accel_raw_t *readings) {
  char reading_rainbow[6];
  
  // read x,y, and z data into buffer
  if (I2CShared_ReadMultipleBytes(I2C_MODULE i2c, ACCEL_WRITE, ACCEL_READ, ACCEL_DATAX0, 6, reading_rainbow)) {
    readings->x = (reading_rainbow[1] << 8) | reading_rainbow[0];
    readings->y = (reading_rainbow[3] << 8) | reading_rainbow[2];
    readings->z = (reading_rainbow[5] << 8) | reading_rainbow[4];
    return ACCEL_SUCCESS;
  } else {
    return ACCEL_FAIL;
  }
}


double AccelGetX(accel_raw_t *readings) {
  double xf;
  xf = (double) readings->x * SCALES[readings->scale_ind];
  return xf;
}


double AccelGetY(accel_raw_t *readings) {
  double yf;
  yf = (double) readings->y * SCALES[readings->scale_ind];
  return yf;
}


double AccelGetZ(accel_raw_t *readings) {
  double zf;
  zf = (double) readings->z * SCALES[readings->scale_ind];
  return zf;
}

