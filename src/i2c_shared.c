#include <p32xxxx.h>
#include <plib.h>
#include "i2c_shared.h"

BOOL I2CShared_StartTransfer(I2C_MODULE i2c, BOOL restart) {
  I2C_STATUS status;

  // Send the Start (or Restart) signal
  if(restart) {
    I2CRepeatStart(i2c);
  }
  else {
    // Wait for the bus to be idle, then start the transfer
    while( !I2CBusIsIdle(i2c) );

    if(I2CStart(i2c) != I2C_SUCCESS)
    {
      DBPRINTF("I2CShared_StartTransfer: Bus collision during transfer Start\n");
      return FALSE;
    }
  }

  // Wait for the signal to complete
  do {
    status = I2CGetStatus(i2c);
  } while (status & I2C_START));

return TRUE;
}


BOOL I2CShared_TransmitOneByte(I2C_MODULE i2c, BYTE data) {
    // Wait for the transmitter to be ready
    while(!I2CTransmitterIsReady(i2c));

    // Transmit the byte
    if(I2CSendByte(i2c, data) == I2C_MASTER_BUS_COLLISION)
    {
        DBPRINTF("Error: I2C Master Bus Collision\n");
        return FALSE;
    }

    // Wait for the transmission to finish
    while(!I2CTransmissionHasCompleted(i2c));

    return TRUE;
}

BOOL I2CShared_Write(I2C_MODULE i2c, char i2c_addr, char i2c_register, BYTE data) {
  BOOL ack, trans;

  // Wait until bus is open
  while(!I2CBusIsIdle(i2c));  
  
  // START TRANSACTION
  if(!I2CShared_StartTransfer(i2c, FALSE) {
    DBPRINTF("I2CShared_Write: Error, bus collision during transfer start to I2C=%d\n", i2c);
    return FALSE;
  }

  // SEND INTERNAL REGISTER
  trans = I2CShared_TransmitOneByte(i2c, i2c_register);
  ack = I2CByteWasAcknowledged(i2c);
  if (!trans || !ack) {
    DBPRINTF("I2CShared_Write: Error, could not send i2c_register 0x%c to I2C=%d\n", i2c_register, i2c);
    I2CShared_StopTransfer(i2c);
    return FALSE;
  }
  
  // SEND ADDRESS
  // Send address for transaction, address is expected to already be formatted
  trans = I2CShared_TransmitOneByte(i2c, i2c_addr);
  ack = I2CByteWasAcknowledged(i2c);
  if (!trans || !ack) {
    DBPRINTF("I2CShared_Write: Error, could not send address 0x%c to I2C=%d\n", i2c_addr, i2c);
    I2CShared_StopTransfer(i2c);
    return FALSE;
  }
  
  // WRITE DATA BYTE
  trans = I2CShared_TransmitOneByte(i2c, data);
  ack = I2CByteWasAcknowledged(i2c);
  if (!trans || !ack) {
    DBPRINTF("I2CShared_Write: Error, could not send data byte 0x%c to I2C=%d\n", data, i2c);
    I2CShared_StopTransfer(i2c);
    return FALSE;
  }
    
  // SEND STOP
  I2CShared_StopTransfer(i2c);
  
  // Transaction complete
  return TRUE;
}


BOOL I2CShared_Read(I2C_MODULE i2c, char i2c_addr, char i2c_register, char *buffer) {
  I2C_RESULT result;
    
  while(!I2CBusIsIdle(i2c));  
  
  // START TRANSACTION
  if(!I2CShared_StartTransfer(i2c, FALSE) {
    DBPRINTF("I2CShared_Read: Error, bus collision during transfer start to I2C=%d\n", i2c);
    return FALSE;
  }
    
  // SEND ADDRESS
  // Send address for transaction, address is expected to already be formatted
  trans = I2CShared_TransmitOneByte(i2c, i2c_addr);
  ack = I2CByteWasAcknowledged(i2c);
  if (!trans || !ack) {
    DBPRINTF("I2CShared_Read: Error, could not send i2c_address 0x%c to I2C=%d\n", i2c_addr, i2c);
    I2CShared_StopTransfer(i2c);
    return FALSE;
  }
  
  // SEND INTERNAL REGISTER
  trans = I2CShared_TransmitOneByte(i2c, i2c_register);
  ack = I2CByteWasAcknowledged(i2c);
  if (!trans || !ack) {
    DBPRINTF("I2CShared_Read: Error, could not send i2c_register 0x%c to I2C=%d\n", i2c_register, i2c);
    I2CShared_StopTransfer(i2c);
    return FALSE;
  }
  
  // READ DATA BYTE
  result = I2CReceiverEnable(i2c, TRUE);  // configure i2c module to receive
  if (result != I2C_SUCCESS) {
    DBPRINTF("I2CShared_Read: Error, could not configure I2C=%d to be a receiver\n", i2c);
    I2CStop(i2c);
    return FALSE;
  }
  while (!I2CReceivedDataIsAvailable(i2c)); // loop until data is ready to be read
  *buffer = I2CGetByte(i2c);
  I2CAcknowledgeByte(i2c, FALSE); // send nack on last byte
  I2CStop(i2c);
  return TRUE;

void I2CShared_StopTransfer(I2C_MODULE i2c) {
  I2C_STATUS  status;

  // Send the Stop signal
  I2CStop(i2c);

  // Wait for the signal to complete
  do
  {
    status = I2CGetStatus(i2c);
  } while (!(status & I2C_STOP));
}