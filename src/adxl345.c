#include <p32xxxx.h>
#include <plib.h>
#include "adxl345.h"
#include "i2c_shared.h"

static double SCALES[4] = { .0039, .0078, .0156, .0312};  // from datasheet

/************************************************************************************************** 
  Function:
    ACCEL_RESULT AccelInit(I2C_MODULE i2c, char resolution, char bandwidth, accel_raw_t *raw)
    
  Author(s):
    mkobit
    
  Summary:
    Initializes the accelerometer
    
  Description:
    Typically will be called from the IMU code to set the sampling rate of the IMU and the resolution on the data

  Preconditions:
    I2C module previously enabled

  Parameters: 
      I2C_MODULE i2c - I2C module associated with this accelerometer
      char range - constant for the range of gravity
          ACCEL_SCALE_2G
          ACCEL_SCALE_4G
          ACCEL_SCALE_8G
          ACCEL_SCALE_16G
      char bandwidth - device bandwidth and output data rate
          ACCEL_BW_1600
          ACCEL_BW_800
          ACCEL_BW_400
          ACCEL_BW_200
          ACCEL_BW_100
          ACCEL_BW_50
          ACCEL_BW_25
      accel_raw_t *raw - accelerometer data structure associated with this accelerometer

  Returns:
    ACCEL_SUCCESS - If successful
    ACCEL_FAIL - If any AccelWrite function fails to write to the accelerometer or the resolution is not 1 of the resolutions provided

  Example:
    <code>
    AccelInit(I2C1, ACCEL_SCALE_4G, ACCEL_BW_100, &raw_values)
    </code>

  Conditions at Exit:
    Accelerometer set to take samples at set data rate and the range is also set. If either of these do not get set, ACCEL_FAIL is returned.
    I2C bus is in idle state

**************************************************************************************************/
ACCEL_RESULT AccelInit(I2C_MODULE i2c, char range, char bandwidth, accel_raw_t *raw) {

  // I2C should already be enabled
  
  // Determine which scaling to use when getting the values
  switch(range) {
    case ACCEL_RANGE_2G: raw->scale_ind = ACCEL_SCALE_2G; break;
    case ACCEL_RANGE_4G: raw->scale_ind = ACCEL_SCALE_4G; break;
    case ACCEL_RANGE_8G: raw->scale_ind = ACCEL_SCALE_8G; break;
    case ACCEL_RANGE_16G: raw->scale_ind = ACCEL_SCALE_16G; break;
    default: DBPRINTF("AccelInitI2C: Error, 0x%c not a valid range for data format for adxl345\n", range); return ACCEL_FAIL;
  }
  
  // Write configurations to it
  // Put accel in MEASURE mode
  if (AccelWrite(i2c, ACCEL_POWER_CTL, ACCEL_MEASURE) == ACCEL_FAIL) {
    DBPRINTF("AccelInitI2C: Error, could not write to ACCEL_POWER_CTL to I2C=%d\n", i2c); 
    return ACCEL_FAIL
  }
  
  // Set Data Format
  if (AccelWrite(i2c, ACCEL_DATA_FORMAT, range) == ACCEL_FAIL) {
    DBPRINTF("AccelInitI2C: Error, could not write to ACCEL_POWER_CTL to I2C=%d\n", i2c); 
    return ACCEL_FAIL
  }

  // Set Bandwidth
  if (AccelWrite(i2c, ACCEL_BW_RATE, bandwidth) == ACCEL_FAIL) {
    DBPRINTF("AccelInitI2C: Error, could not write to ACCEL_POWER_CTL to I2C=%d\n", i2c); 
    return ACCEL_FAIL
  }
  
  return ACCEL_SUCCESS;
}

/************************************************************************************************** 
  Function:
    ACCEL_RESULT AccelWrite(I2C_MODULE i2c, char i2c_reg, BYTE data)

  Author(s):
    mkobit

  Summary:
    Writes a single byte to an accelerometer register using the I2C module parameter

  Description:
    Interface for the IMU to use, passes control to the shared I2C library to write by providings the default accelerometer I2C write address

  Preconditions:
    I2C module previously enabled and running

  Parameters:
    I2C_MODULE i2c - I2C module to connect with
    char i2c_reg - register to write to
    BYTE data - data to be written

  Returns:
    ACCEL_SUCCESS - If successful
    ACCEL_FAIL - If I2CShared library cannot complete its I2CShared_WriteByte

  Example:
    <code>
    AccelWrite(i2c, ACCEL_DATA_FORMAT, ACCEL_SCALE_8G)
    </code>

  Conditions at Exit:
    I2C bus is in idle state

**************************************************************************************************/
ACCEL_RESULT AccelWrite(I2C_MODULE i2c, char i2c_reg, BYTE data) {
  if (I2CShared_WriteByte(i2c, ACCEL_WRITE, i2c_reg, data)) {
    return ACCEL_SUCCESS;
  } else {
    return ACCEL_FAIL;
  }
}

/************************************************************************************************** 
  Function:
    ACCEL_RESULT AccelRead(I2C_MODULE i2c, char i2c_reg, char *buffer)

  Author(s):
    mkobit

  Summary:
    Interface for the IMU to use, reads a single byte into buffer using I2CShared_ReadByte 

  Description:		Passes control to the shared I2C library to read by providings the default accelerometer I2C read address	Preconditions:		I2C module previously enabled and running	Parameters:		I2C_MODULE i2c - I2C module to connect with
    char i2c_reg - register to read from
    char *buffer - buffer to place read byte into	Returns:		ACCEL_SUCCESS - If successful
    ACCEL_FAIL - If I2CShared library cannot complete its I2CShared_ReadByte	Example:		<code>
    char c;
    AccelRead(i2c, ACCEL_DEVID, &c)
    </code>	Conditions at Exit:
    I2C bus is in idle state

**************************************************************************************************/
ACCEL_RESULT AccelRead(I2C_MODULE i2c, char i2c_reg, char *buffer) {
  if (I2CShared_ReadByte(i2c, ACCEL_WRITE, ACCEL_READ, i2c_reg, buffer)) {
    return ACCEL_SUCCESS;
  } else {
    return ACCEL_FAIL;
  }
}

/************************************************************************************************** 
  Function:		ACCEL_RESULT AccelReadAllAxes(I2C_MODULE i2c, accel_raw_t *raw)	Author(s):
    mkobit

  Summary:		Reads x, y, and z acceleration data into the (raw) structure	Description:		Passes control to the shared I2C library to read several bytes into the buffer starting from the X axis register	Preconditions:		I2C module previously enabled and running	Parameters:
    I2C_MODULE i2c -  I2C module to connect with
    accel_raw_t *raw - reference to structure to place read data into	Returns:		ACCEL_SUCCESS - If successful
    ACCEL_FAIL - If I2CShared library cannot complete its I2CShared_ReadMultipleBytes	Example:		<code>
    AccelReadAllAxes(i2c, &accel_data)
    </code>	Conditions at Exit:
    I2C bus is in idle state, (raw) has most recent readings

**************************************************************************************************/
ACCEL_RESULT AccelReadAllAxes(I2C_MODULE i2c, accel_raw_t *raw) {
  char reading_rainbow[6];  // placeholder for read data
  
  // read x,y, and z data into buffer
  if (I2CShared_ReadMultipleBytes(I2C_MODULE i2c, ACCEL_WRITE, ACCEL_READ, ACCEL_DATAX0, 6, reading_rainbow)) {
    //expand data and place them into the accel_raw_t raw
    raw->x = (reading_rainbow[1] << 8) | reading_rainbow[0];
    raw->y = (reading_rainbow[3] << 8) | reading_rainbow[2];
    raw->z = (reading_rainbow[5] << 8) | reading_rainbow[4];
    return ACCEL_SUCCESS;
  } else {
    return ACCEL_FAIL;
  }
}

/************************************************************************************************** 
  Function:		AccelGetX(accel_raw_t *raw)	Author(s):		mkobit	Summary:		Returns 'g' value of X acceleration	Description:		Multiplies the value in the (raw) data by its corresponding scale factor in (SCALES) and returns it	Preconditions:		AccelInit called prior to this on the (raw) value
    AccelReadAllAxes called to place data in raw	Parameters:		accel_raw_t *raw - pointer to raw read data from accelerometer	Returns:		double xf - value in X in terms of 'g's	Example:		<code>
    double xg;
    AccelReadAllAxes(I2C1, &accel_readings)
    xg = AccelGetX(&accel_readings)
    </code>	Conditions at Exit:
    None

**************************************************************************************************/
double AccelGetX(accel_raw_t *raw) {
  double xf;
  xf = (double) raw->x * SCALES[raw->scale_ind];
  return xf;
}

/************************************************************************************************** 
  Function:		AccelGetY(accel_raw_t *raw)	Author(s):		mkobit	Summary:		Returns 'g' value of Y acceleration	Description:		Multiplies the value in the (raw) data by its corresponding scale factor in (SCALES) and returns it	Preconditions:		AccelInit called prior to this on the (raw) value
    AccelReadAllAxes called to place data in raw	Parameters:		accel_raw_t *raw - pointer to raw read data from accelerometer	Returns:		double xf - value in Y in terms of 'g's	Example:		<code>
    double yg;
    AccelReadAllAxes(I2C1, &accel_readings)
    yg = AccelGetY(&accel_readings)
    </code>	Conditions at Exit:
    None

**************************************************************************************************/
double AccelGetY(accel_raw_t *raw) {
  double yf;
  yf = (double) raw->y * SCALES[raw->scale_ind];
  return yf;
}

/************************************************************************************************** 
  Function:		AccelGetZ(accel_raw_t *raw)	Author(s):		mkobit	Summary:		Returns 'g' value of Z acceleration	Description:		Multiplies the value in the (raw) data by its corresponding scale factor in (SCALES) and returns it	Preconditions:		AccelInit called prior to this on the (raw) value
    AccelReadAllAxes called to place data in raw	Parameters:		accel_raw_t *raw - pointer to raw read data from accelerometer	Returns:		double zf - value in Z in terms of 'g's	Example:		<code>
    double zg;
    AccelReadAllAxes(I2C1, &accel_readings)
    zg = AccelGetZ(&accel_readings)
    </code>	Conditions at Exit:
    None

**************************************************************************************************/
double AccelGetZ(accel_raw_t *raw) {
  double zf;
  zf = (double) raw->z * SCALES[raw->scale_ind];
  return zf;
}

