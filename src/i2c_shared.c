#include <p32xxxx.h>
#include <plib.h>
#include <stdio.h>
#include "i2c_shared.h"

static BOOL I2CShared_TransmitOneByte(I2C_MODULE i2c, BYTE data);
static BOOL I2CShared_StartTransfer(I2C_MODULE i2c, BOOL restart);
static void I2CShared_StopTransfer(I2C_MODULE i2c);

static void I2CShared_DebugStatus(I2C_MODULE i2c);

/************************************************************************************************** 
  Function: 
    BOOL I2CShared_Init(I2C_MODULE i2c, unsigned int peripheral_clock_speed, unsigned int i2c_speed)
  
  Authors(s):
    mkobit
  
  Summary: 
    Enables an I2C module with the given speed
  
  Description: 
    Configures I2C module for normal use and sets the speed if it can
  
  Preconditions: 
    I2C ports not being used for anything
  
  Parameters: 
    I2C_MODULE i2c - I2C module to be used for initialization
    unsigned int peripheral_clock_speed - clock speed of peripheral bus
    unsigned int i2c_speed - target speed of I2C module
  
  Returns: 
    TRUE - successfully enabled I2C on this module
    FALSE - clock frequency is too fast
  
  Example: 
    <code>
    I2CShared_Init(I2C1, 40000000L, 300000)
    </code>
  
  Conditions at Exit: 
    I2C module (i2c) configured for use as an I2C bus
  
**************************************************************************************************/
BOOL I2CShared_Init(I2C_MODULE i2c, unsigned int peripheral_clock_speed, unsigned int i2c_speed) {
  unsigned int actualClock;
  
  I2CConfigure(i2c, I2C_ENABLE_HIGH_SPEED);

  // Check clock rate for peripheral bus
  actualClock = I2CSetFrequency(i2c, peripheral_clock_speed, i2c_speed);
  if (actualClock - i2c_speed > i2c_speed / 10) {
    printf("I2CShared_Init: Error, I2C clock frequency (%u) error exceeds 10%%n\n", actualClock);
    return FALSE;
  }
  // Enable I2C
  I2CEnable(i2c, TRUE);
  
  //Debug
  //printf("I2CShared_Init status print\n");
  //I2CShared_DebugStatus(i2c);

  I2CClearStatus(i2c, I2C_START | I2C_STOP | I2C_ARBITRATION_LOSS);
  
  return TRUE;
}

/************************************************************************************************** 
  Function: 
    static BOOL I2CShared_StartTransfer(I2C_MODULE i2c, BOOL restart)
  
  Author(s):
    mkobit
  
  Summary: 
    Starts or restarts a transaction
  
  Description: 
    Blocks until transaction has been started, restarts the transaction if (restart) is TRUE
    Static function, used by internal library
  
  Preconditions: 
    I2C module configured
    *Transaction started  - *only if (restart) is TRUE
  
  Parameters: 
    I2C_MODULE i2c - I2C module to be used for this transaction
    BOOL restart - restart transaction when TRUE, just start when FALSE
  
  Returns: 
    TRUE - If successful
    FALSE - If unsuccessful
  
  Example: 
    <code>
    I2CShared_StartTransfer(i2c, FALSE)
    </code>
  
  Conditions at Exit: 
    Bus transaction started
    I2C waiting for next action
  
**************************************************************************************************/
static BOOL I2CShared_StartTransfer(I2C_MODULE i2c, BOOL restart) {
  I2C_STATUS status;

  // Send the start/restart signal
  if(restart) {
    I2CRepeatStart(i2c);
  }
  else {
    // Wait for the bus to be idle, then start the transfer
    while(!I2CBusIsIdle(i2c)){
        //printf("I2CShared_StartTransfer: Waiting for bus to be idle\n");
        //I2CShared_DebugStatus(i2c);
        I2CShared_ResetBus(i2c);
    }

    //printf("I2CShared_StartTransfer\n");
    //I2CShared_DebugStatus(i2c);
    if(I2CStart(i2c) != I2C_SUCCESS)
    {
      printf("I2CShared_StartTransfer: Bus collision during transfer Start\n");
      //I2CShared_DebugStatus(i2c);
      return FALSE;
    }
  }

  // Wait for the signal to complete
  do {
    status = I2CGetStatus(i2c);
    if (status & I2C_ARBITRATION_LOSS) {
        printf("I2CShared_StartTransfer: lost arbitration on i2c bus\n");
        return FALSE;
    }
  } while (!(status & I2C_START));

return TRUE;
}

/************************************************************************************************** 
  Function: 
    static BOOL I2CShared_TransmitOneByte(I2C_MODULE i2c, unsigned char data)
    
  Author(s): 
    mkobit
  
  Summary: 
    Transmits one byte of data to (i2c)
  
  Description: 
    Waits until transmitter is ready and sends the byte of data
    Static function, used by internal library
  
  Preconditions: 
    Transaction started
    I2C module configured
  
  Parameters: 
    I2C_MODULE i2c - I2C module to be used for this transaction
    unsigned char data - to be transmitted
  
  Returns: 
    TRUE - If successful
    FALSE - If unsuccessful
  
  Example: 
    <code>
    I2CShared_TransmitOneByte(I2C1, 0x43)
    </code>
  
  Conditions at Exit: 
    Transmission of byte complete
    I2C bus waiting for next action
  
**************************************************************************************************/
static BOOL I2CShared_TransmitOneByte(I2C_MODULE i2c, unsigned char data) {

    // Wait for the transmitter to be ready
    while(!I2CTransmitterIsReady(i2c)); // TODO testing the !I2CBusIsIdle(i2c) to see if that helps

    // Transmit the byte
    //I2CShared_DebugStatus(status);
    if(I2CSendByte(i2c, data) == I2C_MASTER_BUS_COLLISION)
    {
        printf("I2CShared_TransmitOneByte: Error, I2C Master Bus Collision , status = 0x%x\n", I2CGetStatus(i2c));
        return FALSE;
    }

    // Wait for the transmission to finish
    while(!I2CTransmissionHasCompleted(i2c));

    //printf("I2CShared_TransmitOneByte: after transmission complete, before ackknowledged\n");
    //I2CShared_DebugStatus(i2c);

    if(!I2CByteWasAcknowledged(i2c))
    {
        printf("I2CShared_TransmitOneByte: Error, sent byte was not acknowledged, status = 0x%x\n", I2CGetStatus(i2c));
        //I2CShared_DebugStatus(i2c);
        return FALSE;
    }
    return TRUE;
}

/************************************************************************************************** 
  Function: 
    BOOL I2CShared_WriteByte(I2C_MODULE i2c, unsigned char i2c_addr, unsigned char i2c_register, unsigned char data)
  
  Author(s):
    mkobit
  
  Summary: 
    Writes a single byte, (data), to the (i2c) module to register (i2c_register)
  
  Description: 
    Waits until (i2c) bus is idle and then uses I2C protocol to start a transaction, write to the device and its register,
  
  Preconditions: 
    I2C module configured
  
  Parameters: 
    I2C_MODULE i2c - I2C module to be used for this transaction
    unsigned char i2c_addr - write address of I2C device
    unsigned char i2c_register - I2C register to write to
    unsigned char data - data to be written
  
  Returns: 
    TRUE - write succesful
    FALSE - write unsuccessful 
  
  Example:
    <code>
    // from accelerometer library
    I2CShared_WriteByte(i2c, ACCEL_WRITE, i2c_reg, data)
    </code>
  
  Conditions at Exit: 
    
  
**************************************************************************************************/
BOOL I2CShared_WriteByte(I2C_MODULE i2c, unsigned char i2c_write_addr, unsigned char i2c_register, unsigned char data) {

  // Wait until bus is open
  //while(!I2CBusIsIdle(i2c));
  
  // START TRANSACTION
  if(!I2CShared_StartTransfer(i2c, FALSE)) {
    printf("I2CShared_Write: Error, bus collision during transfer start to I2C=%d\n", i2c);
    return FALSE;
  }

  // SEND ADDRESS
  // Send address for transaction, address is expected to already be formatted
  if (!I2CShared_TransmitOneByte(i2c, i2c_write_addr)) {
    printf("I2CShared_Write: Error, could not send address 0x%c to I2C=%d\n", i2c_write_addr, i2c);
    //I2CShared_StopTransfer(i2c);
    return FALSE;
  }
  
  // SEND INTERNAL REGISTER
  if (!I2CShared_TransmitOneByte(i2c, i2c_register)) {
    printf("I2CShared_Write: Error, could not send i2c_register 0x%c to I2C=%d\n", i2c_register, i2c);
    //I2CShared_StopTransfer(i2c);
    return FALSE;
  }
    
  // WRITE DATA BYTE
  if (!I2CShared_TransmitOneByte(i2c, data)) {
    printf("I2CShared_Write: Error, could not send data byte 0x%c to I2C=%d\n", data, i2c);
    //I2CShared_StopTransfer(i2c);
    return FALSE;
  }
    
  // SEND STOP
  I2CShared_StopTransfer(i2c);
  
  // Transaction complete
  return TRUE;
}

/************************************************************************************************** 
  Function:
    BOOL I2CShared_ReadByte(I2C_MODULE i2c, unsigned char i2c_write_addr, unsigned char i2c_read_addr, unsigned char i2c_register, char *buffer)

  Author(s): 
    mkobit
  
  Summary: 
    Read a single byte from the (i2c) module from (i2c_register) and places it into (buffer)
  
  Description: 
    Waits until (i2c) bus is idle and then uses I2C protocol to start a transaction, write to the device and its register, restart with the read address
    and read the data. Sends a NACK at the end to stop the transmission and then stops the transaction
  
  Preconditions: 
    I2C module configured
  
  Parameters: 
    I2C_MODULE i2c - I2C module to be used for this transaction
    unsigned char i2c_write_addr - write address of I2C device
    unsigned char i2c_read_addr - read address of I2C device
    unsigned char i2c_register - I2C register to read from
    unsigned char *buffer - buffer to place read byte into
  
  Returns: 
    TRUE - read succesful
    FALSE - read unsuccessful
  
  Example: 
    <code>
    // example from accelerometer
    I2CShared_ReadByte(i2c, ACCEL_WRITE, ACCEL_READ, i2c_reg, buffer)
    </code>
  
  Conditions at Exit: 
    Buffer has byte of data in it read from (i2c)
    I2C bus idle
  
**************************************************************************************************/
BOOL I2CShared_ReadByte(I2C_MODULE i2c, unsigned char i2c_write_addr, unsigned char i2c_read_addr, unsigned char i2c_register, unsigned char *buffer) {

  // Wait until bus is idle
  //while(!I2CBusIsIdle(i2c));
  
  // START TRANSACTION
  if(!I2CShared_StartTransfer(i2c, FALSE)) {
    printf("I2CShared_Read: Error, bus collision during transfer start to I2C=%d\n", i2c);
    return FALSE;
  }
    
  // SEND ADDRESS
  // Send write address for transaction, address is expected to already be formatted
  if (!I2CShared_TransmitOneByte(i2c, i2c_write_addr)) {
    printf("I2CShared_Read: Error, could not send write address 0x%c to I2C=%d\n", i2c_write_addr, i2c);
    //I2CShared_StopTransfer(i2c);
    return FALSE;
  }
  
  // SEND INTERNAL REGISTER
  if (!I2CShared_TransmitOneByte(i2c, i2c_register)) {
    printf("I2CShared_Read: Error, could not send i2c_register 0x%x to I2C=%d\n", i2c_register, i2c);
    //I2CShared_StopTransfer(i2c);
    return FALSE;
  }
  
  // SEND START AGAIN FOR READ
  // Send read address
  if(!I2CShared_StartTransfer(i2c, TRUE)) {
    printf("I2CShared_Read: Error, bus collision during transfer start to I2C=%d\n", i2c);
    return FALSE;
  }

  if (!I2CShared_TransmitOneByte(i2c, i2c_read_addr)) {
    printf("I2CShared_Read: Error, could not send i2c_address 0x%x to I2C=%d\n", i2c_read_addr, i2c);
    //I2CShared_StopTransfer(i2c);
    return FALSE;
  }
  
  // READ DATA BYTE
  // configure i2c module to receive
  if (I2CReceiverEnable(i2c, TRUE) != I2C_SUCCESS) {
    printf("I2CShared_Read: Error, could not configure I2C=%d to be a receiver\n", i2c);
    //I2CStop(i2c);
    return FALSE;
  }
  while (!I2CReceivedDataIsAvailable(i2c)); // loop until data is ready to be read
  //printf("Before getbyte\n");
  //I2CShared_DebugStatus(i2c);
  *buffer = I2CGetByte(i2c);
  I2CAcknowledgeByte(i2c, FALSE); // send nack on last byte
  while(!I2CAcknowledgeHasCompleted(i2c));
  I2CStop(i2c);
  return TRUE;
}

/************************************************************************************************** 
  Function:
    BOOL I2CShared_ReadMultipleBytes(I2C_MODULE i2c, unsigned char i2c_write_addr, unsigned char i2c_read_addr, unsigned char i2c_register_start, int nbytes, unsigned char *buffer)

  Author(s):
    mkobit

  Summary:
    Reads (nbytes) from the (i2c) module starting from (i2c_read_addr)  and places in (buffer)

  Description:
    Waits until (i2c) bus is idle and then uses I2C protocol to start a transaction, write to the device and its register, restart with the read address
    and read the data. Sends a NACK at the end to stop the transmission and then stops the transaction

  Preconditions:
    I2C module configured

  Parameters:
    I2C_MODULE i2c - I2C module to be used for this transaction
    unsigned char i2c_write_addr - write address of I2C device
    unsigned char i2c_read_addr - read address of I2C device
    unsigned char i2c_register_start - I2C register to begin reading from
    int nbytes - how many bytes to be read
    unsigned char *buffer - buffer to place read bytes

  Returns:
    TRUE - reads were successful
    FALSE - at least 1 read was not successful

  Example:
    <code>
    // example from gyroscope, reading_rainbow buffer size 8
    I2CShared_ReadMultipleBytes(I2C_MODULE i2c, GYRO_WRITE, GYRO_READ, startReadI2CReg, nDataToRead, reading_rainbow)
    </code>

  Conditions at Exit:
    Buffer has n bytes of data in it read from (i2c)
    I2C bus idle

**************************************************************************************************/
BOOL I2CShared_ReadMultipleBytes(I2C_MODULE i2c, unsigned char i2c_write_addr, unsigned char i2c_read_addr, unsigned char i2c_register_start, int nbytes, unsigned char *buffer) {
  int i;
  unsigned char temp;

  // Wait until bus is open
  //while(!I2CBusIsIdle(i2c));
  
  // START TRANSACTION
  if(!I2CShared_StartTransfer(i2c, FALSE)) {
    printf("I2CShared_ReadMultipleBytes: Error, bus collision during transfer start to I2C=%d\n", i2c);
    return FALSE;
  }
    
  // SEND ADDRESS
  if (!I2CShared_TransmitOneByte(i2c, i2c_write_addr)) {
    printf("I2CShared_ReadMultipleBytes: Error, could not send i2c_address 0x%x to I2C=%d\n", i2c_write_addr, i2c);
    //I2CShared_StopTransfer(i2c);
    return FALSE;
  }
  
  // SEND INTERNAL REGISTER
  if (!I2CShared_TransmitOneByte(i2c, i2c_register_start)) {
    printf("I2CShared_ReadMultipleBytes: Error, could not send i2c_register 0x%x to I2C=%d\n", i2c_register_start, i2c);
    //I2CShared_StopTransfer(i2c);
    return FALSE;
  }
  
  // SEND START AGAIN FOR READ
  // Send read address
  if(!I2CShared_StartTransfer(i2c, TRUE)) {
    printf("I2CShared_ReadMultipleBytes: Error, bus collision during transfer REstart to I2C=%d\n", i2c);
    return FALSE;
  }

  if (!I2CShared_TransmitOneByte(i2c, i2c_read_addr)) {
    printf("I2CShared_ReadMultipleBytes: Error, could not send i2c_address 0x%c to I2C=%d\n", i2c_read_addr, i2c);
    //I2CShared_StopTransfer(i2c);
    return FALSE;
  }
  
  // START READING
  // read all the data bytes and place them into the buffer
  for (i = 0; i < nbytes; i++) {
    // configure i2c module to receive
    if (I2CReceiverEnable(i2c, TRUE) != I2C_SUCCESS) {
      printf("I2CShared_ReadMultipleBytes: Error, could not configure I2C=%d to be a receiver\n", i2c);
      //I2CShared_StopTransfer(i2c);
      return FALSE;
    }
    while (!I2CReceivedDataIsAvailable(i2c)); // loop until data is ready to be read
    temp = I2CGetByte(i2c);
    I2CAcknowledgeByte(i2c, TRUE);
    // place read data in buffer
    buffer[i] = temp;
    // END READ
  }
  
  // Stop the transation
  I2CStop(i2c);
  return TRUE;
}

/************************************************************************************************** 
  Function:
    void I2CShared_StopTransfer(I2C_MODULE i2c)

  Author(s):
    mkobit

  Summary:
    Stops a transaction on the (i2c) module

  Description:
    Sends a stop signal and waits until the signal is complete
    Static function, used by internal library

  Preconditions:
    I2C module configured

  Parameters:
    I2C_MODULE i2c - I2C module to be used for this transaction

  Returns:
    void

  Example:
    <code>
    I2CShared_StopTransfer(I2C1)
    </code>

  Conditions at Exit:
    I2C bus idle

**************************************************************************************************/
static void I2CShared_StopTransfer(I2C_MODULE i2c) {
  I2C_STATUS  status;

  // Send the Stop signal
  I2CStop(i2c);

  // Wait for the signal to complete
  do {
    status = I2CGetStatus(i2c);
  } while (!(status & I2C_STOP));
}

static void I2CShared_DebugStatus(I2C_MODULE i2c) {
    I2C_STATUS status;
    status = I2CGetStatus(i2c);
    if (I2C_TRANSMITTER_FULL & status) printf("I2CShared_DebugStatus: I2C_TRANSMITTER_FULL, 0x%x\n", I2C_TRANSMITTER_FULL & status);
    if (I2C_DATA_AVAILABLE & status) printf("I2CShared_DebugStatus & status) I2C_DATA_AVAILABLE, 0x%x\n", I2C_DATA_AVAILABLE & status);
    if (I2C_SLAVE_READ & status) printf("I2CShared_DebugStatus & status) I2C_SLAVE_READ, 0x%x\n", I2C_SLAVE_READ & status);
    if (I2C_START & status) printf("I2CShared_DebugStatus & status) I2C_START, 0x%x\n", I2C_START & status);
    if (I2C_STOP & status) printf("I2CShared_DebugStatus & status) I2C_STOP, 0x%x\n", I2C_STOP & status);
    if (I2C_SLAVE_DATA & status) printf("I2CShared_DebugStatus & status) I2C_SLAVE_DATA, 0x%x\n", I2C_SLAVE_DATA & status);
    if (I2C_RECEIVER_OVERFLOW & status) printf("I2CShared_DebugStatus & status) I2C_RECEIVER_OVERFLOW, 0x%x\n", I2C_RECEIVER_OVERFLOW & status);
    if (I2C_TRANSMITTER_OVERFLOW & status) printf("I2CShared_DebugStatus & status) I2C_TRANSMITTER_OVERFLOW, 0x%x\n", I2C_TRANSMITTER_FULL & status);
    if (I2C_10BIT_ADDRESS & status) printf("I2CShared_DebugStatus & status) I2C_10BIT_ADDRESS, 0x%x\n", I2C_10BIT_ADDRESS & status);
    if (I2C_GENERAL_CALL & status) printf("I2CShared_DebugStatus & status) I2C_GENERAL_CALL, 0x%x\n", I2C_GENERAL_CALL & status);
    if (I2C_ARBITRATION_LOSS & status) printf("I2CShared_DebugStatus & status) I2C_ARBITRATION_LOSS, 0x%x\n", I2C_ARBITRATION_LOSS & status);
    if (I2C_TRANSMITTER_BUSY & status) printf("I2CShared_DebugStatus & status) I2C_TRANSMITTER_BUSY, 0x%x\n", I2C_TRANSMITTER_BUSY & status);
    if (I2C_BYTE_ACKNOWLEDGED & status) printf("I2CShared_DebugStatus & status) I2C_BYTE_ACKNOWLEDGE, 0x%x\n", I2C_BYTE_ACKNOWLEDGED & status);
}

inline void I2CShared_ResetBus(I2C_MODULE i2c) {
    I2CClearStatus(i2c, I2C_START | I2C_STOP | I2C_ARBITRATION_LOSS | I2C_BYTE_ACKNOWLEDGED);
}