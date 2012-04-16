#include <p32xxxx.h>
#include <plib.h>
#include "i2c_shared.h"

BOOL I2CShared_StartTransfer(I2C_MODULE i2c, BOOL restart)
{
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


BOOL I2CShared_TransmitOneByte(I2C_MODULE i2c, BYTE data)
{
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


void I2CShared_StopTransfer(I2C_MODULE i2c)
{
  I2C_STATUS  status;

  // Send the Stop signal
  I2CStop(i2c);

  // Wait for the signal to complete
  do
  {
    status = I2CGetStatus(i2c);
  } while (!(status & I2C_STOP));
}