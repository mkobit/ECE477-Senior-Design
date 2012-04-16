#include <p32xxxx.h>
#include <plib.h>
#include "adxl345.h"
#include "i2c_shared.h"

static double SCALES[4] = { .0039, .0078, .0156, .0312};  // from datasheet

static void AccelStopTx(I2C_MODULE i2c);

ACCEL_RESULT AccelInitI2C(I2C_MODULE i2c, 
						unsigned int peripheral_clock_speed, 
						unsigned int i2c_speed, 
						char resolution, 
						char bandwidth, 
						accel_raw_readings *readings) {

  unsigned int actualClock;
  ACCEL_RESULT result;
  
  // Check clock rate for peripheral bus
  actualClock = I2CSetFrequency(i2c, peripheral_clock_speed, i2c_speed)
  if (actualClock - i2c_speed > i2c_speed / 10) {
    DBPRINTF("AccelInitI2C: Error, I2C clock frequency (%u) error exceeds 10%%n\n", actualClock); 
    return ACCEL_FAIL;
  }
  I2CEnable(i2c, TRUE);
  
  // Write configurations to it
  result = AccelWrite(i2c, ACCEL_POWER_CTL, ACCEL_MEASURE);   // Put accel in MEASURE mode
  if (result == ACCEL_FAIL) {
    DBPRINTF("AccelInitI2C: Error, could not write to ACCEL_POWER_CTL to I2C=%d\n", i2c); 
    return ACCEL_FAIL;
  }
  
  result = AccelWrite(i2c, ACCEL_DATA_FORMAT, resolution);   // Data Format
  if (result == ACCEL_FAIL) {
    DBPRINTF("AccelInitI2C: Error, could not write to ACCEL_DATA_FORMAT to I2C=%d\n", i2c); 
    return ACCEL_FAIL;
  }
  
  result = AccelWrite(i2c, ACCEL_BW_RATE, bandwidth);  // Bandwidth
  if (result == ACCEL_FAIL) {
    DBPRINTF("AccelInitI2C: Error, could not write to ACCEL_BW_RATE to I2C=%d\n", i2c);
    return ACCEL_FAIL;
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
  if (I2CShared_Write(i2c, ACCEL_WRITE, i2c_reg, data)) {
    return ACCEL_SUCCESS;
  } else {
    return ACCEL_FAIL;
  }
}


ACCEL_RESULT AccelRead(I2C_MODULE i2c, char i2c_reg, char *buffer) {
  if (I2CShared_Read(i2c, ACCEL_READ, i2c_reg, buffer)) {
    return ACCEL_SUCCESS;
  } else {
    return ACCEL_FAIL;
  }
}


ACCEL_RESULT AccelReadAllAxes(I2C_MODULE i2c, accel_raw_readings *readings) {
  BOOL ack, trans;
  char temp_lsb;
  char temp_msb;

  // Wait until bus is open
  while(!I2CBusIsIdle(i2c));  
  
  // START TRANSACTION
  if(!I2CShared_StartTransfer(i2c, FALSE) {
    DBPRINTF("AccelReadAllAxes: Error, bus collision during transfer start to I2C=%d\n", i2c);
    return ACCEL_FAIL;
  }
    
  // SEND ADDRESS
  // Send address for transaction OR'ed with write bit, address is beginning of axis data
  trans = I2CShared_TransmitOneByte(i2c, (ACCEL_DATAX0 << 1) | I2C_WRITE);
  ack = I2CByteWasAcknowledged(i2c);
  if (!trans || !ack) {
    DBPRINTF("AccelReadAllAxes: Error, could not send address 0x%c to I2C=%d\n", i2c_addr, i2c);
    I2CShared_StopTransfer(i2c);
    return ACCEL_FAIL;
  }
  
  // START READING
  // Read raw X
  // read x LSB
  result = I2CReceiverEnable(i2c, TRUE);  // configure i2c module to receive
  if (result != I2C_SUCCESS) {
    DBPRINTF("AccelReadAllAxes: Error, could not configure I2C=%d to be a receiver\n", i2c);
    I2CShared_StopTransfer(i2c);
    return ACCEL_FAIL;
  }
  while (!I2CReceivedDataIsAvailable(i2c)); // loop until data is ready to be read
  // x LSB
  temp_lsb = I2CGetByte(i2c);
  I2CAcknowledgeByte(i2c, TRUE);
  // read 
  result = I2CReceiverEnable(i2c, TRUE);  // configure i2c module to receive
  if (result != I2C_SUCCESS) {
    DBPRINTF("AccelReadAllAxes: Error, could not configure I2C=%d to be a receiver\n", i2c);
    I2CShared_StopTransfer(i2c);
    return ACCEL_FAIL;
  }
  while (!I2CReceivedDataIsAvailable(i2c)); // loop until data is ready to be read
  // x MSB
  temp_msb = I2CGetByte(i2c);
  I2CAcknowledgeByte(i2c, TRUE);
  // Update X
  accel_raw_readings->x = (temp_msb << 8) | temp_lsb;
  
  // Read raw Y
  // read y LSB
  result = I2CReceiverEnable(i2c, TRUE);  // configure i2c module to receive
  if (result != I2C_SUCCESS) {
    DBPRINTF("AccelReadAllAxes: Error, could not configure I2C=%d to be a receiver\n", i2c);
    I2CShared_StopTransfer(i2c);
    return ACCEL_FAIL;
  }
  while (!I2CReceivedDataIsAvailable(i2c)); // loop until data is ready to be read
  // y LSB
  temp_lsb = I2CGetByte(i2c);
  I2CAcknowledgeByte(i2c, TRUE);
  // read y MSB
  result = I2CReceiverEnable(i2c, TRUE);  // configure i2c module to receive
  if (result != I2C_SUCCESS) {
    DBPRINTF("AccelReadAllAxes: Error, could not configure I2C=%d to be a receiver\n", i2c);
    I2CShared_StopTransfer(i2c);
    return ACCEL_FAIL;
  }
  while (!I2CReceivedDataIsAvailable(i2c)); // loop until data is ready to be read
  // y MSB
  temp_msb = I2CGetByte(i2c);  
  I2CAcknowledgeByte(i2c, TRUE);
  // Update Y
  accel_raw_readings->y = (temp_msb << 8) | temp_lsb;
  
  // Read raw Z
  // read z LSB
  result = I2CReceiverEnable(i2c, TRUE);  // configure i2c module to receive
  if (result != I2C_SUCCESS) {
    DBPRINTF("AccelReadAllAxes: Error, could not configure I2C=%d to be a receiver\n", i2c);
    I2CShared_StopTransfer(i2c);
    return ACCEL_FAIL;
  }
  while (!I2CReceivedDataIsAvailable(i2c)); // loop until data is ready to be read
  // z LSB
  temp_lsb = I2CGetByte(i2c);
  I2CAcknowledgeByte(i2c, TRUE);
  // read z mSB
  result = I2CReceiverEnable(i2c, TRUE);  // configure i2c module to receive
  if (result != I2C_SUCCESS) {
    DBPRINTF("AccelReadAllAxes: Error, could not configure I2C=%d to be a receiver\n", i2c);
    I2CShared_StopTransfer(i2c);
    return ACCEL_FAIL;
  }
  while (!I2CReceivedDataIsAvailable(i2c)); // loop until data is ready to be read
  // z MSB
  temp_msb = I2CGetByte(i2c);  
  // Send NACK to stop reading
  I2CAcknowledgeByte(i2c, FALSE);
  // Update Z
  accel_raw_readings->z = (temp_msb << 8) | temp_lsb;
  // END READ
  
  // Stop the transation
  I2CStop(i2c);
  return ACCEL_SUCCESS;
}


double AccelGetX(accel_raw_readings *readings) {
  double xf;
  xf = (double) readings->x * SCALES[readings->scale_ind];
  return xf;
}


double AccelGetY(accel_raw_readings *readings) {
  double yf;
  yf = (double) readings->y * SCALES[readings->scale_ind];
  return yf;
}


double AccelGetZ(accel_raw_readings *readings) {
  double zf;
  zf = (double) readings->z * SCALES[readings->scale_ind];
  return zf;
}

