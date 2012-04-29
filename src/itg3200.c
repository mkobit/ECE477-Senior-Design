#include <p32xxxx.h>
#include <plib.h>
#include "itg3200.h"
#include "i2c_shared.h"
#include "delay.h"

static float GyroGetConvertedX(gyro_raw_t *const raw, const INT16 xPol, const float xGain);   // X === Roll
static float GyroGetConvertedY(gyro_raw_t *const raw, const INT16 yPol, const float yGain);   // Y === Pitch
static float GyroGetConvertedZ(gyro_raw_t *const raw, const INT16 zPol, const float zGain);   // Z === Yaw

/************************************************************************************************** 
  Function:
    GYRO_RESULT GyroInit(gyro_t *const gyro, const I2C_MODULE i2c, const UINT8 i2c_address,
      const UINT8 dlpf_lpf, const UINT8 sample_rate_div, const UINT8 power_mgmt_sel)

  Author(s):
    mkobit

  Summary:
    Initializes the gyroscope

  Description:
    Typically will be called from the IMU code to set the sampling rate of the IMU and the resolution on the data

  Preconditions:
    I2C module previously enabled

  Parameters:
    gyro_t *const gyro - gyro to initialized
    const I2C_MODULE i2c - I2C module to connect with
    const UINT8 i2c_address - I2C address of gyroscope
    const UINT8 dlpf_lpf - low pass filter configuration for sensor acquisition
        GYRO_DLPF_LPF_256HZ   - results in 8 kHz sample rate
        GYRO_DLPF_LPF_188HZ   - results in 1 kHz sample rate
        GYRO_DLPF_LPF_98HZ    - *
        GYRO_DLPF_LPF_42HZ    - *
        GYRO_DLPF_LPF_20HZ    - *
        GYRO_DLPF_LPF_10HZ    - *
        GYRO_DLPF_LPF_5HZ     - *
    const UINT8 sample_rate_div - sample rate divider, F = F_internal / (sample_rate_div + 1)
      E.g. -> 1kHz sample rate from dlpf_lpf, sample_rate_div = 9, F = 1 kHz / (9 _ 1) = 100 Hz 
    const UINT8 power_mgmt_sel - device clock selector
      GYRO_PWR_MGM_CLK_SEL_INTERNAL - internal oscillator
      GYRO_PWR_MGM_CLK_SEL_X        - X as clock reference
      GYRO_PWR_MGM_CLK_SEL_Y        - Y as clock reference
      GYRO_PWR_MGM_CLK_SEL_Z        - Z as clock reference

  Returns:
    GYRO_SUCCESS - If successful
    GYRO_FAIL - If any GyroWrite fails

  Example:
    <code>
    GyroInit(I2C1, GYRO_DLPF_LPF_42HZ, 5, GYRO_PWR_MGM_CLK_SEL_X)
    </code>

  Conditions at Exit:
    Gyro set to take samples at set data rate and the power management is also set. If either of these do not get set, GYRO_FAIL is returned
    I2C bus is in idle state

**************************************************************************************************/
GYRO_RESULT GyroInit(gyro_t *const gyro, const I2C_MODULE i2c, const UINT8 i2c_address,
        const UINT8 dlpf_lpf, const UINT8 sample_rate_div, const UINT8 power_mgmt_sel) {

  // Set default polarities, offsets, and gains
  GyroSetOffsets(gyro, 0, 0, 0);

  GyroSetRevPolarity(gyro, FALSE, FALSE, FALSE);

  GyroSetGains(gyro, 1.0f, 1.0f, 1.0f);

  // Assign it the I2C Module and address
  gyro->i2c = i2c;
  gyro->i2c_addr = i2c_address;

  // OR the low pass frequency passed with dflp_config with full scale operation and write it to the gyro
  // Set internal clock and full scale operation
  if (GyroWrite(gyro, GYRO_DLPF_FS, dlpf_lpf | GYRO_DLPF_FS_ON) == GYRO_FAIL) {
    printf("GyroInit: Error, could not write 0x%x to register GYRO_DLPF_FS 0x%x\n", (UINT8) dlpf_lpf | GYRO_DLPF_FS_ON, (UINT8) GYRO_DLPF_FS);
    return GYRO_FAIL;
  }
  
  // Set sample rate divider
  // If dlpf_lpf == GYRO_DLPF_LPF_256HZ, sample rate = 8 kHz / sample_rate_div
  // Else, sample rate = 1 kHz / (sample_rate_div + 1)
  if (GyroWrite(gyro, GYRO_SMPLRT_DIV, sample_rate_div) == GYRO_FAIL) {
    printf("GyroInit: Error, could not write 0x%x to register GYRO_DLPF_FS 0x%x\n", (UINT8) dlpf_lpf | GYRO_DLPF_FS_ON, (UINT8) GYRO_DLPF_FS);
    return GYRO_FAIL;
  }
  
  // Select a gyro PLL for clock source (more stable)
  if (GyroWrite(gyro, GYRO_PWR_MGM, power_mgmt_sel) == GYRO_FAIL) {
    printf("GyroInit: Error, could not write 0x%x to register GYRO_DLPF_FS 0x%x\n", (UINT8) dlpf_lpf | GYRO_DLPF_FS_ON, (UINT8) GYRO_DLPF_FS);
    return GYRO_FAIL;
  }
  
  // Set interrupts - NOT BEING USED CURRENTLY
  
  return GYRO_SUCCESS;
}

/************************************************************************************************** 
  Function:
    GYRO_RESULT GyroWrite(gyro_t *const gyro, const UINT8 i2c_reg, const UINT8 data)

  Author(s):
    mkobit

  Summary:
    Writes a single byte to an gyro register using the I2C module parameter

  Description:
    Interface for the IMU to use, passes control to the shared I2C library to write by providings the default gyro I2C write address

  Preconditions:
    I2C module previously enabled and running

  Parameters:
    gyro_t *const gyro - gyro to be written to
    const UINT8 i2c_reg - register to write to
    const UINT8 data - data to be written

  Returns:
    GYRO_SUCCESS - If successful
    GYRO_FAIL - If I2CShared library cannot complete its I2CShared_WriteByte

  Example:
    <code>
    GyroWrite(I2C1, GYRO_SMPLRT_DIV, 5)
    </code>

  Conditions at Exit:
    

**************************************************************************************************/
GYRO_RESULT GyroWrite(gyro_t *const gyro, const UINT8 i2c_reg, const UINT8 data) {
  I2C_MODULE i2c;

  // Get I2C module from gyro
  i2c = gyro->i2c;

  if (I2CShared_WriteByte(i2c, gyro->i2c_addr, i2c_reg, data)) {
    return GYRO_SUCCESS;
  } else {
    return GYRO_FAIL;
  }
}

/************************************************************************************************** 
  Function:
    GYRO_RESULT GyroRead(gyro_t *const gyro, UINT8 i2c_reg, UINT8 *const buffer)

  Author(s):
    mkobit

  Summary:
    Interface for the IMU to use, reads a single byte into buffer using I2CShared_ReadByte

  Description:
    Passes control to the shared I2C library to read by providing the default gyro I2C read address

  Preconditions:
    I2C module previously enabled and running

  Parameters:
    gyro_t *const gyro - gyro to be read from
    const UINT8 i2c_reg - register to read from
    UINT8 *const buffer - buffer to place read byte into

  Returns:
    GYRO_SUCCESS - If successful
    GYRO_FAIL - If I2CShared library cannot complete its I2CShared_ReadByte

  Example:
    <code>
    char c;
    GyroRead(i2c, GYRO_WHO_AM_I, &c)
    </code>

  Conditions at Exit:
    

**************************************************************************************************/
GYRO_RESULT GyroRead(gyro_t *const gyro, const UINT8 i2c_reg, UINT8 *const buffer) {
  I2C_MODULE i2c;

  // Get I2C module from gyro
  i2c = gyro->i2c;
  if (I2CShared_ReadByte(i2c, gyro->i2c_addr, i2c_reg, buffer)) {
    return GYRO_SUCCESS;
  } else {
    return GYRO_FAIL;
  }
}

/************************************************************************************************** 
  Function: 
    GYRO_RESULT GyroCalibrate(gyro_t *const gyro, int samplesToTake, UINT ms_delay)
  
  Author(s): 
    mkobit
  
  Summary: 
    Calibrates the gyro by determining the offset for future reads and updates. The gyro should be held still during calibration
  
  Description: 
    Reads a bunch of samples and averages them to determine the natural offset of the gyroscope
  
  Preconditions: 
    Gyro initialized
    I2C bus of this gyroscope is idle
  
  Parameters: 
    gyro_t *const gyro - gyroto calibrate
    int samplesToTake - number of samples to take before averaging
    UINT ms_delay - delay before each sample should be taken
  
  Returns: 
    GYRO_SUCCESS - If successful
    GYRO_FAIL - If calibration times out or cannot be completed
  
  Example: 
    <code>
    </code>
  
  Conditions at Exit: 
    
  
**************************************************************************************************/
GYRO_RESULT GyroCalibrate(gyro_t *const gyro, int samplesToTake, UINT ms_delay) {
  int samplesTaken = 0;
  float tempOffsets[3];
  GYRO_RESULT res;
  I2C_MODULE i2c;
  
  int timeout = 0;

  // Get gyro's I2C module
  i2c = gyro->i2c;

  // Take samples and update the calibration
  do {
    res = GyroReadAllAxes(gyro->i2c, gyro->i2c_addr, &gyro->raw, FALSE);
    if (res == GYRO_SUCCESS) {
      tempOffsets[0] += GyroGetRawX(gyro);
      tempOffsets[1] += GyroGetRawY(gyro);
      tempOffsets[2] += GyroGetRawZ(gyro);
      samplesTaken++;
      //printf("Sample %d: tX = %4.2f, tY = %4.2f, tZ = %4.2f\n", samplesTaken, tempOffsets[0], tempOffsets[1], tempOffsets[2]);
    } else {
      // If this many read fails, just exit with failure to calibrate
      if (timeout++ == 5000) {
        // Switch back in old values
        printf("error in calibrate, error %d\n", timeout);
        return GYRO_FAIL;
      }
    }
    DelayMs(ms_delay);
  } while (samplesTaken < samplesToTake);

  // successful calibration, update the offsets
  GyroSetOffsets(gyro, -tempOffsets[0] / samplesToTake, -tempOffsets[1] / samplesToTake, -tempOffsets[2] / samplesToTake);
  return GYRO_SUCCESS;
}

/************************************************************************************************** 
  Function: 
    GYRO_RESULT GyroUpdate(gyro_t *const gyro, const BOOL readTemp)
  
  Author(s): 
    mkobit
  
  Summary: 
    Updates the gyro to the most recent values
  
  Description: 
    Calls GyroReadAllAxes with this gyro's raw values
  
  Preconditions: 
    Gyro initialized
    I2C idle
  
  Parameters: 
    gyro_t *const gyro - gyro to update the values of
    const BOOL readTemp - if the gyroscope should also read the temperature
  
  Returns: 
    GYRO_SUCCESS - If successful
    GYRO_FAIL - If I2CShared library cannot complete its I2CShared_ReadMultipleBytes
  
  Example: 
    <code>
    GyroUpdate(&gyro, TRUe)
    </code>
  
  Conditions at Exit: 
    Gyro has the most recent values
    I2C idle
  
**************************************************************************************************/
GYRO_RESULT GyroUpdate(gyro_t *const gyro, const BOOL readTemp) {
  GYRO_RESULT res;

  res = GyroReadAllAxes(gyro->i2c, gyro->i2c_addr, &gyro->raw, readTemp);
  GyroAddOffsets(gyro);
  return res;
}

/************************************************************************************************** 
  Function:
    GYRO_RESULT GyroReadAllAxes(const I2C_MODULE i2c, const UINT8 i2c_address,
        gyro_t *const gyro, const BOOL readTemp)

  Author(s):
    mkobit

  Summary:
    Reads x, y, and z angular momentum data into the (raw) structure, as well as the temperature if readTemp is TRUE

  Description:
    Passes control to the shared I2C library to read several bytes into the buffer starting from the X axis register or the temperature register depending
    on whether or not readTemp is TRUE or FALSE. Then places the corresponding data into the gyro

  Preconditions:
    I2C module previously enabled and idle

  Parameters:
    const I2C_MODULE i2c - I2C module to connect with
    const UINT8 i2c_address - i2c address of gyroscope
    gyro_raw_t *const raw - pointer to raw read data from gyro
    const BOOL readTemp - if TRUE read temperature if FALSE do not read temperature

  Returns:
    GYRO_SUCCESS - If successful
    GYRO_FAIL - If I2CShared library cannot complete its I2CShared_ReadMultipleBytes

  Example:
    <code>
    GyroReadAllAxes(I2C1, &gyro_data, TRUE)
    </code>

  Conditions at Exit:
    I2C bus is in idle state, (raw) has most recent readings

**************************************************************************************************/
GYRO_RESULT GyroReadAllAxes(const I2C_MODULE i2c, const UINT8 i2c_address,
        gyro_raw_t *const raw, const BOOL readTemp) {
  UINT8 reading_rainbow[8];
  int nDataToRead;
  int offsetForTemp;
  UINT8 startReadI2CReg;

  nDataToRead = 6 + (readTemp ? 2 : 0);
  offsetForTemp = 0 + (readTemp ? 2 : 0);
  startReadI2CReg = readTemp ? GYRO_TEMP_OUT_H : GYRO_XOUT_H;
  
  // read x,y, and z data into buffer
  if (I2CShared_ReadMultipleBytes(i2c, i2c_address, startReadI2CReg, nDataToRead, reading_rainbow)) {
    //expand data and place them into the accel_raw_t readings
    
    // if readTemp was true, update temperature
    if (readTemp) {
      raw->temp = ((INT16)reading_rainbow[0] << 8) | reading_rainbow[1];
    }

    // Add offset
    raw->x = ((INT16)reading_rainbow[0 + offsetForTemp] << 8) | reading_rainbow[1 + offsetForTemp];
    raw->y = ((INT16)reading_rainbow[2 + offsetForTemp] << 8) | reading_rainbow[3 + offsetForTemp];
    raw->z = ((INT16)reading_rainbow[4 + offsetForTemp] << 8) | reading_rainbow[5 + offsetForTemp];

    return GYRO_SUCCESS;
  } else {
    return GYRO_FAIL;
  }
}

/************************************************************************************************** 
  Function: 
    void GyroAddOffsets(gyro_t *const gyro)
  
  Author(s): 
    mkobit
  
  Summary: 
    Adds the gyros offsets to its raw values
  
  Description: 
    Same as summary
  
  Preconditions: 
    Gyro initialized
    Gyro update called
    Typically gyro has been calibrated
  
  Parameters: 
    gyro_t *const gyro - gyro to offset the raw values of
  
  Returns: 
    void
  
  Example: 
    <code>
    GyroAddOffsets(&gyro)
    </code>
  
  Conditions at Exit: 
    
  
**************************************************************************************************/
void GyroAddOffsets(gyro_t *const gyro) {
  gyro_raw_t *raw = &gyro->raw;

  raw->x += gyro->xOffset;
  raw->y += gyro->yOffset;
  raw->z += gyro->zOffset;
}

/************************************************************************************************** 
  Function:
    float GyroGetTemp(const gyro_t *const gyro)

  Author(s):
    mkobit

  Summary:
    Converts temperature read from device to Celsius and returns it

  Description:
    Uses values provided by the datasheet to convert the read temperature into Celsius

  Preconditions:
    GyroInit called prior to this on the (raw) value
    GyroReadAllAxes called to place data in raw using TRUE for reading temperature

  Parameters:
    gyro_raw_t *raw - pointer to raw read data from gyro

  Returns:
    float temperature - converted temperature in Celsius

  Example:
    <code>
    float tempC
    tempC = GyroGetTemp(&raw_gyro)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
float GyroGetTemp(gyro_t *const gyro) {
  float temperature;
 gyro_raw_t *raw;

  // Get pointer to raw values
  raw = &gyro->raw;
  
  temperature = -GYRO_TEMP_OFFSET + raw->temp;
  temperature /= GYRO_TEMP_CONV_TO_DEGREES;
  temperature += GYRO_TEMP_OFFSET_DEGS;
  return temperature;
}

/************************************************************************************************** 
  Function:
    float GyroGetX(const gyro_t *const gyro)

  Author(s):
    mkobit

  Summary:
    Returns 'degrees/s' of raw data X which is roll

  Description:
    Multiplies the value in the (raw) data by the scale factor provided by the data sheet

  Preconditions:
    GyroInit called prior to this on the (raw) value
    GyroReadAllAxes called to place data in raw

  Parameters:
    gyro_raw_t *raw - pointer to raw read data from gyro

  Returns:
    float xd - roll in terms of degrees/s

  Example:
    <code>
    float roll
    roll = GyroGetX(&raw_gyro)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
float GyroGetX(gyro_t *const gyro) {
  return GyroGetConvertedX(&gyro->raw, gyro->xPolarity, gyro->xGain);
}

/************************************************************************************************** 
  Function:
    float GyroGetY(const gyro_t *const gyro)

  Author(s):
    mkobit

  Summary:
    Returns 'degrees/s' of raw data Y which is pitch

  Description:
    Multiplies the value in the (raw) data by the scale factor provided by the data sheet

  Preconditions:
    GyroInit called prior to this on the (raw) value
    GyroReadAllAxes called to place data in raw

  Parameters:
    gyro_raw_t *raw - pointer to raw read data from gyro

  Returns:
    float yd - pitch in terms of degrees/s

  Example:
    <code>
    float pitch
    pitch = GyroGetY(&raw_gyro)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
float GyroGetY(gyro_t *const gyro) {
  return GyroGetConvertedY(&gyro->raw, gyro->yPolarity, gyro->yGain);
}

/************************************************************************************************** 
  Function:
    float GyroGetZ(const gyro_t *const gyro)

  Author(s):
    mkobit

  Summary:
    Returns 'degrees/s' of raw data Z which is yaw

  Description:
    Multiplies the value in the (raw) data by the scale factor provided by the data sheet

  Preconditions:
    GyroInit called prior to this on the (raw) value
    GyroReadAllAxes called to place data in raw

  Parameters:
    gyro_raw_t *raw - pointer to raw read data from gyro

  Returns:
    float zd - yaw in terms of degrees/s

  Example:
    <code>
    float yaw
    yaw = GyroGetZ(&raw_gyro)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
float GyroGetZ(gyro_t *const gyro) {
  return GyroGetConvertedZ(&gyro->raw, gyro->zPolarity, gyro->zGain);
}

/************************************************************************************************** 
  Function: 
    void GyroSetRevPolarity(gyro_t *const gyro, const BOOL xPol, const BOOL yPol, const BOOL zPol)
  
  Author(s): 
    mkobit
  
  Summary: 
    Sets polarities of the axes to be read
  
  Description: 
    If the parameters are TRUE, sets the polarities as reverse when they are read
  
  Preconditions: 
    Gyro initialized
  
  Parameters: 
    gyro_t *const gyro - gyroscope to set offsets of
    const BOOL xPol - reverse polarity of X
    const BOOL yPol - reverse polarity of Y
    const BOOL zPol - reverse polarity of Z
  
  Returns: 
    void
  
  Example: 
    <code>
    GyroSetRevPolarity(&gyro, FALSE, FALSE, FALSE)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
void GyroSetRevPolarity(gyro_t *const gyro, const BOOL xPol, const BOOL yPol, const BOOL zPol) {
  gyro->xPolarity = xPol ? -1 : 1;
  gyro->yPolarity = yPol ? -1 : 1;
  gyro->zPolarity = zPol ? -1 : 1;
}

/************************************************************************************************** 
  Function: 
    void GyroSetOffsets(gyro_t *const gyro, INT16 xOff, INT16 yOff, INT16 zOff)
  
  Author(s): 
    mkobit
  
  Summary: 
    Sets gains of this gyro
  
  Description: 
    Same as summary
  
  Preconditions: 
    Gyro initialized
  
  Parameters: 
    gyro_t *const gyro - gyroscope to set offsets of
    INT16 xOff - offset to be added to X readings
    INT16 yOff - offset to be added to X readings
    INT16 zOff - offset to be added to X readings
  
  Returns: 
    void
  
  Example: 
    <code>
    GyroSetOffsets(&gyro, 5, -3, 16)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
void GyroSetOffsets(gyro_t *const gyro, INT16 xOff, INT16 yOff, INT16 zOff) {
  gyro->xOffset = xOff;
  gyro->yOffset = yOff;
  gyro->zOffset = zOff;
}

/************************************************************************************************** 
  Function: 
    void GyroSetGains(gyro_t *const gyro, const float xGain, const float yGain, const float zGain)
  
  Author(s): 
    mkobit
  
  Summary: 
    Sets gains of this gyro
  
  Description: 
    Same as summary
  
  Preconditions: 
    Gyro initialized
  
  Parameters: 
    gyro_t *const gyro - gyroscope to set gains of
    const float xGain - gain of X rotations
    const float yGain - gain of Y rotations
    const float zGain - gain of Z rotations
  
  Returns: 
    void
  
  Example: 
    <code>
    GyroSetGains(&gyro, 1.0f, 1.0f, 1.0f)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
void GyroSetGains(gyro_t *const gyro, const float xGain, const float yGain, const float zGain) {
  gyro->xGain = xGain;
  gyro->yGain = yGain;
  gyro->zGain = zGain;
}

/************************************************************************************************** 
  Function: 
    static float GyroGetConvertedX(gyro_raw_t *const raw, const INT16 xPol, const float xGain)
  
  Author(s): 
    mkobit
  
  Summary: 
    Returns raw X value in degrees/s
  
  Description: 
    Multiplies the raw value by the polarity, gain, and the scale factor to convert the reading into degrees
    Static function, used by internal library
  
  Preconditions: 
    Gyro initialized
    Gyro update called
  
  Parameters: 
    gyro_t *const gyro - gyroscope to get raw value from
    const INT16 xPol - polarity of X rotation
    const float xGain - gain of X rotation
  
  Returns: 
    float xd - angle of X rotation in degrees/s
  
  Example: 
    <code>
    GyroGetConvertedX(&gyro, 1, 1.0f)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
static float GyroGetConvertedX(gyro_raw_t *const raw, const INT16 xPol, const float xGain) { // X === Roll
  float xd;
  
  xd = (float) (raw->x * xPol) / GYRO_CONV_TO_DEGREES * xGain;
  return xd;
}

/************************************************************************************************** 
  Function: 
    static float GyroGetConvertedY(gyro_raw_t *const raw, const INT16 yPol, const float yGain)
  
  Author(s): 
    mkobit
  
  Summary: 
    Returns raw Y value in degrees/s
  
  Description: 
    Multiplies the raw value by the polarity, gain, and the scale factor to convert the reading into degrees
    Static function, used by internal library
  
  Preconditions: 
    Gyro initialized
    Gyro update called
  
  Parameters: 
    gyro_t *const gyro - gyroscope to get raw value from
    const INT16 yPol - polarity of Y rotation
    const float yGain - gain of Y rotation
  
  Returns: 
    float yd - angle of Y rotation in degrees/s
  
  Example: 
    <code>
    GyroGetConvertedY(&gyro, 1, 1.0f)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
static float GyroGetConvertedY(gyro_raw_t *const raw, const INT16 yPol, const float yGain) {   // Y === Pitch
  float yd;
  
  yd = (float) (raw->y * yPol) / GYRO_CONV_TO_DEGREES * yGain;
  return yd;
}

/************************************************************************************************** 
  Function: 
    static float GyroGetConvertedZ(gyro_raw_t *const raw, const INT16 zPol, const float zGain)
  
  Author(s): 
    mkobit
  
  Summary: 
    Returns raw Z value in degrees/s
  
  Description: 
    Multiplies the raw value by the polarity, gain, and the scale factor to convert the reading into degrees
    Static function, used by internal library
  
  Preconditions: 
    Gyro initialized
    Gyro update called
  
  Parameters: 
    gyro_t *const gyro - gyroscope to get raw value from
    const INT16 zPol - polarity of Z rotation
    const float zGain - gain of Z rotation
  
  Returns: 
    float zd - angle of Z rotation in degrees/s
  
  Example: 
    <code>
    GyroGetConvertedZ(&gyro, 1, 1.0f)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
static float GyroGetConvertedZ(gyro_raw_t *const raw, const INT16 zPol, const float zGain) {   // Z === Yaw
  float zd;
  
  zd = (float) (raw->z * zPol) / GYRO_CONV_TO_DEGREES * zGain;
  return zd;
}

/************************************************************************************************** 
  Function: 
    INT16 GyroGetRawX(gyro_t *const gyro)
  
  Author(s): 
    mkobit
  
  Summary: 
    Returns raw X value multiplied by the polarity
  
  Description: 
    Same as summary
  
  Preconditions: 
    Gyro initialized
    Gyro update called
  
  Parameters: 
    gyro_t *const gyro - gyroscope to get raw value from
  
  Returns: 
    INT16 rawX - raw value multiplied by polarity
  
  Example: 
    <code>
    GyroGetRawX(&gyro)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
INT16 GyroGetRawX(gyro_t *const gyro) {
  return gyro->raw.x * gyro->xPolarity;
}

/************************************************************************************************** 
  Function: 
    INT16 GyroGetRawY(gyro_t *const gyro)
  
  Author(s): 
    mkobit
  
  Summary: 
    Returns raw Y value multiplied by the polarity
  
  Description: 
    Same as summary
  
  Preconditions: 
    Gyro initialized
    Gyro update called
  
  Parameters: 
    gyro_t *const gyro - gyroscope to get raw value from
  
  Returns: 
    INT16 rawY - raw value multiplied by polarity
  
  Example: 
    <code>
    GyroGetRawY(&gyro)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
INT16 GyroGetRawY(gyro_t *const gyro) {
  return gyro->raw.y * gyro->yPolarity;
}

/************************************************************************************************** 
  Function: 
    INT16 GyroGetRawZ(gyro_t *const gyro)
  
  Author(s): 
    mkobit
  
  Summary: 
    Returns raw Z value multiplied by the polarity
  
  Description: 
    Same as summary
  
  Preconditions: 
    Gyro initialized
    Gyro update called
  
  Parameters: 
    gyro_t *const gyro - gyroscope to get raw value from
  
  Returns: 
    INT16 rawZ - raw value multiplied by polarity
  
  Example: 
    <code>
    GyroGetRawZ(&gyro)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
INT16 GyroGetRawZ(gyro_t *const gyro) {
  return gyro->raw.z * gyro->zPolarity;
}