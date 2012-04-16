#include <p32xxxx.h>
#include <plib.h>
#include "adxl345.h"

static void AccelStopTx(I2C_MODULE i2c);

ACCEL_RESULT AccelInitI2C(I2C_MODULE i2c, unsigned int i2c_speed, char resolution, char bandwidth) {
  unsigned int actualClock;
  ACCEL_RESULT result;
  
  actualClock = I2CSetFrequency(i2c, GetPeripheralClock(), i2c_speed)
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
  return ACCEL_SUCCESS;
}


ACCEL_RESULT AccelWrite(I2C_MODULE i2c, char i2c_addr, BYTE data) {
  I2C_RESULT result;
  I2C_STATUS status;

  // Wait until bus is open
  while(!I2CBusIsIdle(i2c));  
  
  // START TRANSACTION
  if(I2CStart(i2c) != I2C_SUCCESS) {
    DBPRINTF("AccelRead: Error, bus collision during transfer start to I2C=%d\n", i2c);
    return ACCEL_FAIL;
  }
  do { 
    status = I2CGetStatus(i2c);
  } while (!(status & I2C_START));  // blocks until start is finished
  
  // SEND ADDRESS
  // Wait for the transmitter to be ready
  while(!I2CTransmitterIsReady(i2c));
  // Send address for transaction OR'ed with write bit
  result = I2CSendByte(i2c, (data << 1) | I2C_WRITE);
  while (!I2CTransmissionHasCompleted(i2c));
  if (result != I2C_SUCCESS || !I2CByteWasAcknowledged(i2c)) {
    DBPRINTF("AccelWrite: Error, could not send address 0x%c to I2C=%d\n", i2c_addr, i2c);
    I2CStop(i2c);
    return ACCEL_FAIL;
  }
  
  // WRITE DATA BYTE
  result = I2CSendByte(i2c, data);
  while (!I2CTransmissionHasCompleted(i2c));
  if (result != I2C_SUCCESS || !I2CByteWasAcknowledged(i2c)) {
    DBPRINTF("AccelWrite: Error, could not send data byte 0x%c to I2C=%d\n", data, i2c);
    I2CStop(i2c);
    return ACCEL_FAIL;
  }
    
  // SEND STOP
  do {
    status = I2CGetStatus(i2c);
  } while ( !(status & I2C_STOP) );
  
  // Transaction complete
  return ACCEL_SUCCESS;
}


ACCEL_RESULT AccelRead(I2C_MODULE i2c, char i2c_addr, char *buffer) {
  I2C_RESULT result;
    
  while(!I2CBusIsIdle(i2c));  // wait until bus is open
  
  // Start transaction
  if(I2CStart(EEPROM_I2C_BUS) != I2C_SUCCESS) {
    DBPRINTF("AccelRead: Error, bus collision during transfer start to I2C=%d\n", i2c);
    return ACCEL_FAIL;
  }
  do { // blocks until start is finished
    result = I2CGetStatus(EEPROM_I2C_BUS);
  } while (!(result & I2C_START));
  if (result != I2C_SUCCESS || !I2CByteWasAcknowledged(i2c)) {
    DBPRINTF("AccelRead: Error, could not start a write to I2C=%d\n", i2c);
    I2CStop(i2c);
    return ACCEL_FAIL;
  }
  // Send address for transaction OR'ed with read bit
  result = I2CSendByte(i2c, (data << 1) | I2C_READ);
  while (!I2CTransmissionHasCompleted(i2c));
  if (result != I2C_SUCCESS || !I2CByteWasAcknowledged(i2c)) {
    DBPRINTF("AccelRead: Error, could not send address 0x%c to I2C=%d\n", i2c_addr, i2c);
    I2CStop(i2c);
    return ACCEL_FAIL;
  }
  // Read data byte
  result = I2CReceiverEnable(i2c, TRUE);  // configure i2c module to receive
  if (result != I2C_SUCCESS) {
    DBPRINTF("AccelRead: Error, could not configure I2C=%d to be a receiver\n", i2c);
    I2CStop(i2c);
    return ACCEL_FAIL;
  }
  while (!I2CReceivedDataIsAvailable(i2c)); // loop until data is ready to be read
  I2CAcknowledgeByte(i2c, TRUE);
  *buffer = I2CGetByte(i2c);
  
}


ACCEL_RESULT AccelReadAllAxes(I2C_MODULE i2c, accel_raw_readings *readings) {
}


float AccelGetX(accel_raw_readings *readings) {
  float xf;
  xf = readings->x * 5.0;// TODO this needs to be changed to accurately convert the accel
  return xf;
}


float AccelGetY(accel_raw_readings *readings) {
  float yf;
  yf = readings->y * 5.0; // TODO this needs to be changed to accurately convert the accel
  return yf;
}


float AccelGetZ(accel_raw_readings *readings) {
  float zf;
  zf = readings->z * 5.0; // TODO this needs to be changed to accurately convert the accel
  return zf;
}

