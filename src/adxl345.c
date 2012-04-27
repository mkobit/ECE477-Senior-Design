#include <p32xxxx.h>
#include <plib.h>
#include "adxl345.h"
#include "i2c_shared.h"

static float SCALES[4] = { .0039f, .0078f, .0156f, .0312f};  // from datasheet

static float AccelRawGetX(accel_raw_t *const raw, const float xGain);
static float AccelRawGetY(accel_raw_t *const raw, const float yGain);
static float AccelRawGetZ(accel_raw_t *const raw, const float zGain);

/************************************************************************************************** 
  Function:
    ACCEL_RESULT AccelInit(accel_t *const accel, const I2C_MODULE i2c, const UINT8 range, const UINT8 bandwidth)
    
  Author(s):
    mkobit
    
  Summary:
    Initializes the accelerometer
    
  Description:
    Typically will be called from the IMU code to set the sampling rate of the IMU and the resolution on the data

  Preconditions:
    I2C module previously enabled

  Parameters: 
    accel_t *const accel - pointer to accelerometer structure to be initialized
    const I2C_MODULE i2c - I2C module associated with this accelerometer
    const UINT8 range - constant for the range of gravity
      ACCEL_RANGE_2G  - 2 G's (gravity)
      ACCEL_RANGE_4G  - 4 G's
      ACCEL_RANGE_8G  - 8 G's
      ACCEL_RANGE_16G - 16 G's
    const UINT8 bandwidth - device bandwidth and output data rate
      ACCEL_BW_1600   - 1600 Hz
      ACCEL_BW_800    - 800 Hz
      ACCEL_BW_400    - 400 Hz
      ACCEL_BW_200    - 200 Hz
      ACCEL_BW_100    - 100 Hz
      ACCEL_BW_50     - 50 Hz
      ACCEL_BW_25     - 25 Hz

  Returns:
    ACCEL_SUCCESS - If successful
    ACCEL_FAIL - If any AccelWrite function fails to write to the accelerometer or the resolution is not 1 of the resolutions provided

  Example:
    <code>
    AccelInit(&accel, I2C1, ACCEL_SCALE_4G, ACCEL_BW_100)
    </code>

  Conditions at Exit:
    Accelerometer set to take samples at set data rate and the range is also set. If either of these do not get set, ACCEL_FAIL is returned
    I2C bus is in idle state

**************************************************************************************************/
ACCEL_RESULT AccelInit(accel_t *const accel, const I2C_MODULE i2c, const UINT8 range, const UINT8 bandwidth) {

  accel_raw_t *const raw = &accel->raw;
  // I2C should already be enabled

  accel->i2c = i2c;
  
  AccelSetGains(accel, 1.0f, 1.0f, 1.0f);

  // Determine which scaling to use when getting the values
  switch(range) {
    case ACCEL_RANGE_2G: raw->scale_ind = ACCEL_RANGE_2G; break;
    case ACCEL_RANGE_4G: raw->scale_ind = ACCEL_RANGE_4G; break;
    case ACCEL_RANGE_8G: raw->scale_ind = ACCEL_RANGE_8G; break;
    case ACCEL_RANGE_16G: raw->scale_ind = ACCEL_RANGE_16G; break;
    default: printf("AccelInitI2C: Error, 0x%c not a valid range for data format for adxl345\n", range); return ACCEL_FAIL;
  }

  
  
  // Write configurations to it
  // Put accel in MEASURE mode
  if (AccelWrite(accel, ACCEL_POWER_CTL, ACCEL_MEASURE) == ACCEL_FAIL) {
    printf("AccelInitI2C: Error, could not write to ACCEL_POWER_CTL to I2C=%d\n", i2c);
    return ACCEL_FAIL;
  }
  
  // Set Data Format
  if (AccelWrite(accel, ACCEL_DATA_FORMAT, range) == ACCEL_FAIL) {
    printf("AccelInitI2C: Error, could not write to ACCEL_POWER_CTL to I2C=%d\n", i2c);
    return ACCEL_FAIL;
  }

  // Set Bandwidth
  if (AccelWrite(accel, ACCEL_BW_RATE, bandwidth) == ACCEL_FAIL) {
    printf("AccelInitI2C: Error, could not write to ACCEL_BW_RATE to I2C=%d\n", i2c);
    return ACCEL_FAIL;
  }
  
  return ACCEL_SUCCESS;
}

/************************************************************************************************** 
  Function:
    ACCEL_RESULT AccelWrite(accel_t *const accel, const UINT8 i2c_reg, const UINT8 data)

  Author(s):
    mkobit

  Summary:
    Writes a single byte to an accelerometer register using the I2C module parameter

  Description:
    Interface for the IMU to use, passes control to the shared I2C library to write by providings the default accelerometer I2C write address

  Preconditions:
    I2C module previously enabled and running

  Parameters:
    accel_t *const accel - accelerometer for this function to act on
    const UINT8 i2c_reg - register to write to
    const UINT8 data - data to be written

  Returns:
    ACCEL_SUCCESS - If successful
    ACCEL_FAIL - If I2CShared library cannot complete its I2CShared_WriteByte

  Example:
    <code>
    AccelWrite(&accel, ACCEL_DATA_FORMAT, ACCEL_SCALE_8G)
    </code>

  Conditions at Exit:
    I2C bus is in idle state
    Accelerometer written to

**************************************************************************************************/
ACCEL_RESULT AccelWrite(accel_t *const accel, const UINT8 i2c_reg, const UINT8 data) {
  I2C_MODULE i2c;

  // Get I2C module from accel
  i2c = accel->i2c;
  
  if (I2CShared_WriteByte(i2c, ACCEL_WRITE, i2c_reg, data)) {
    return ACCEL_SUCCESS;
  } else {
    return ACCEL_FAIL;
  }
}

/************************************************************************************************** 
  Function:
    ACCEL_RESULT AccelRead(accel_t *const accel, const UINT8 i2c_reg, UINT8 *buffer)

  Author(s):
    mkobit

  Summary:
    Interface for the IMU to use, reads a single byte into buffer using I2CShared_ReadByte 

  Description:
    Passes control to the shared I2C library to read by providing the default accelerometer I2C read address

  Preconditions:
    I2C module previously enabled and running

  Parameters:
    accel_t *const accel - accelerometer for this function to act on
    const UINT8 i2c_reg - register to read from
    UINT8 *buffer - buffer to place read byte into

  Returns:
    ACCEL_SUCCESS - If successful
    ACCEL_FAIL - If I2CShared library cannot complete its I2CShared_ReadByte

  Example:
    <code>
    char c;
    AccelRead(&accel, ACCEL_DEVID, &c)
    </code>

  Conditions at Exit:
    I2C bus is in idle state

**************************************************************************************************/
ACCEL_RESULT AccelRead(accel_t *const accel, const UINT8 i2c_reg, UINT8 *buffer) {
  I2C_MODULE i2c;

  // Get I2C module from accel
  i2c = accel->i2c;

  if (I2CShared_ReadByte(i2c, ACCEL_WRITE, ACCEL_READ, i2c_reg, buffer)) {
    return ACCEL_SUCCESS;
  } else {
    return ACCEL_FAIL;
  }
}

/************************************************************************************************** 
  Function: 
    ACCEL_RESULT AccelUpdate(accel_t *const accel)
  
  Author(s): 
    mkobit
  
  Summary: 
    Updates the accelerometer
  
  Description: 
    Calls AccelReadAllAxes with a pointer to this accelerometers raw readings
  
  Preconditions: 
    Accelerometer initialized
  
  Parameters: 
    accel_t *const accel - accelerometer to update
  
  Returns: 
    ACCEL_SUCCESS - If successful
    ACCEL_FAIL - If AccelReadAllAxes unsuccessful
  
  Example: 
    <code>
    AccelUpdate(&accel)
    </code>
  
  Conditions at Exit: 
    Accelerometer updated to most recent readings its set I2C
  
**************************************************************************************************/
ACCEL_RESULT AccelUpdate(accel_t *const accel) {
  return AccelReadAllAxes(accel->i2c, &accel->raw);
}

/************************************************************************************************** 
  Function:
    ACCEL_RESULT AccelReadAllAxes(const I2C_MODULE i2c, accel_raw_t *const raw)

  Author(s):
    mkobit

  Summary:
    Reads x, y, and z acceleration data into the (raw) structure

  Description:
    Passes control to the shared I2C library to read several bytes into the buffer starting from the X axis register

  Preconditions:
    I2C module previously enabled and running

  Parameters:
    const I2C_MODULE i2c -  I2C module to connect with
    accel_raw_t *const raw - reference to structure to place read data into

  Returns:
    ACCEL_SUCCESS - If successful
    ACCEL_FAIL - If I2CShared library cannot complete its I2CShared_ReadMultipleBytes

  Example:
    <code>
    AccelReadAllAxes(i2c, &accel_data)
    </code>

  Conditions at Exit:
    I2C bus is in idle state, (raw) has readings from accelerometer

**************************************************************************************************/
ACCEL_RESULT AccelReadAllAxes(const I2C_MODULE i2c, accel_raw_t *const raw) {
  UINT8 reading_rainbow[6];  // placeholder for read data
  
  // read x,y, and z data into buffer
  if (I2CShared_ReadMultipleBytes(i2c, ACCEL_WRITE, ACCEL_READ, ACCEL_DATAX0, 6, reading_rainbow)) {
    //expand data and place them into the accel_raw_t raw
    raw->x = ((INT16)reading_rainbow[1] << 8) | reading_rainbow[0];
    raw->y = ((INT16)reading_rainbow[3] << 8) | reading_rainbow[2];
    raw->z = ((INT16)reading_rainbow[5] << 8) | reading_rainbow[4];
    return ACCEL_SUCCESS;
  } else {
    return ACCEL_FAIL;
  }
}

/************************************************************************************************** 
  Function: 
    void AccelSetGains(accel_t *const accel, float xGain, float yGain, float zGain)
  
  Author(s): 
    mkobit
  
  Summary: 
    Sets gains to be used when reading acceleration values
  
  Description: 
    Same as summary
  
  Preconditions: 
    Accelerometer initialized
  
  Parameters: 
    accel_t *const accel - pointer to accelerometer to get data from
    float xGain - gain to be applied to X
    float yGain - gain to be applied to Y
    float zGain - gain to be applied to Z
  
  Returns: 
    void
  
  Example: 
    <code>
    AccelSetGains(&accel, 1.0f, 1.0f, 1.0f)
    </code>
  
  Conditions at Exit: 
    Gains of this accelerometer are set
  
**************************************************************************************************/
void AccelSetGains(accel_t *const accel, float xGain, float yGain, float zGain) {
  accel->xGain = xGain;
  accel->yGain = yGain;
  accel->zGain = zGain;
}

/************************************************************************************************** 
  Function:
    float AccelGetX(accel_t *const accel)

  Author(s):
    mkobit

  Summary:
    Returns 'g' value of X acceleration

  Description:
    Calls AccelRawGetX

  Preconditions:
    AccelInit called prior to this on the (raw) value
    AccelReadAllAxes called to place data in raw

  Parameters:
    accel_t *const accel - pointer to accelerometer to get data from

  Returns:
    float xf - value in X in terms of 'g's

  Example:
    <code>
    float xg;
    AccelReadAllAxes(I2C1, &accel)
    xg = AccelGetX(&accel)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
float AccelGetX(accel_t *const accel) {
  return AccelRawGetX(&accel->raw, accel->xGain);
}

/************************************************************************************************** 
  Function:
    AccelGetY(const accel_raw_t *const raw)

  Author(s):
    mkobit

  Summary:
    Returns 'g' value of Y acceleration

  Description:
    Calls AccelRawGetY

  Preconditions:
    AccelInit called prior to this on the (raw) value
    AccelReadAllAxes called to place data in raw

  Parameters:
    accel_t *const accel - pointer to accelerometer to get data from

  Returns:
    float xf - value in Y in terms of 'g's

  Example:
    <code>
    float yg;
    AccelReadAllAxes(I2C1, &accel)
    yg = AccelGetY(&accel)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
float AccelGetY(accel_t *const accel) {
  return AccelRawGetY(&accel->raw, accel->yGain);
}

/************************************************************************************************** 
  Function:
    float AccelGetZ(accel_t *const accel)

  Author(s):
    mkobit

  Summary:
    Returns 'g' value of Z acceleration

  Description:
    Calls AccelRawGetZ
    Multiplies the value in the (raw) data by its corresponding scale factor in (SCALES) and returns it

  Preconditions:
    AccelInit called prior to this on the (raw) value
    AccelReadAllAxes called to place data in raw

  Parameters:
    accel_t *const accel - pointer to accelerometer to get data from

  Returns:
    float zf - value in Z in terms of 'g's

  Example:
    <code>
    float zg;
    AccelReadAllAxes(I2C1, &accel)
    zg = AccelGetZ(&accel)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
float AccelGetZ(accel_t *const accel) {
  return AccelRawGetZ(&accel->raw, accel->xGain);
}

/************************************************************************************************** 
  Function: 
    static float AccelRawGetX(accel_raw_t *const raw, const float xGain)
  
  Author(s): 
    mkobit
  
  Summary: 
    Scales the X value read from the accelerometer and returns it
  
  Description: 
    Multiplies the raw value by the scale and by the gain supplied to the function
    Static function, used by internal library
  
  Preconditions: 
    Accelerometer was initialized
    (raw) has valid values in it
  
  Parameters: 
    accel_raw_t *const raw - raw data to operate on
    const float xGain - gain to be applied to the result
  
  Returns: 
    float xf - Scaled value of X, in 'g's'
  
  Example: 
    <code>
    float x = AccelRawGetX(&accel->raw, accel->xGain)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
static float AccelRawGetX(accel_raw_t *const raw, const float xGain) {
  float xf;
  xf = (float) raw->x * SCALES[raw->scale_ind] * xGain;
  return xf;
}

/************************************************************************************************** 
  Function: 
    static float AccelRawGetY(accel_raw_t *const raw, const float yGain)
  
  Author(s): 
    mkobit
  
  Summary: 
    Scales the Y value read from the accelerometer and returns it
  
  Description: 
    Multiplies the raw value by the scale and by the gain supplied to the function
    Static function, used by internal library
  
  Preconditions: 
    Accelerometer was initialized
    (raw) has valid values in it
  
  Parameters: 
    accel_raw_t *const raw - raw data to operate on
    const float yGain - gain to be applied to the result
  
  Returns: 
    float yf - Scaled value of Y, in 'g's'
  
  Example: 
    <code>
    float y = AccelRawGetY(&accel->raw, accel->yGain)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
static float AccelRawGetY(accel_raw_t *const raw, const float yGain) {
  float yf;
  yf = (float) raw->y * SCALES[raw->scale_ind] * yGain;
  return yf;
}

/************************************************************************************************** 
  Function: 
    static float AccelRawGetZ(accel_raw_t *const raw, const float zGain)
  
  Author(s): 
    mkobit
  
  Summary: 
    Scales the Z value read from the accelerometer and returns it
  
  Description: 
    Multiplies the raw value by the scale and by the gain supplied to the function
    Static function, used by internal library
  
  Preconditions: 
    Accelerometer was initialized
    (raw) has valid values in it
  
  Parameters: 
    accel_raw_t *const raw - raw data to operate on
    const float zGain - gain to be applied to the result
  
  Returns: 
    float zf - Scaled value of Z, in 'g's'
  
  Example: 
    <code>
    float z = AccelRawGetZ(&accel->raw, accel->zGain)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
static float AccelRawGetZ(accel_raw_t *const raw, const float zGain) {
  float zf;
  zf = (float) raw->z * SCALES[raw->scale_ind] * zGain;
  return zf;
}