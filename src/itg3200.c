#include <p32xxxx.h>
#include <plib.h>
#include "itg3200.h"
#include "i2c_shared.h"
#include "delay.h"

/************************************************************************************************** 
  Function:
    GYRO_RESULT GyroInit(const I2C_MODULE i2c, const UINT8 dlpf_lpf, const UINT8 sample_rate_div, const UINT8 power_mgmt_sel)

  Author(s):
    mkobit

  Summary:
    Initializes the gyroscope

  Description:
    Typically will be called from the IMU code to set the sampling rate of the IMU and the resolution on the data

  Preconditions:
    I2C module previously enabled

  Parameters:
    const I2C_MODULE i2c - I2C module to connect with
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
GYRO_RESULT GyroInit(gyro_t *const gyro, const I2C_MODULE i2c, const UINT8 dlpf_lpf, const UINT8 sample_rate_div, const UINT8 power_mgmt_sel) {

  // Set default polarities, offsets, and gains
  gyro->xOffset = 0.0f;
  gyro->yOffset = 0.0f;
  gyro->zOffset = 0.0f;

  gyro->xPolarity = 1;
  gyro->yPolarity = 1;
  gyro->zPolarity = 1;

  gyro->xGain = 1.0f;
  gyro->yGain = 1.0f;
  gyro->zGain = 1.0f;

  // OR the low pass frequency passed with dflp_config with full scale operation and write it to the gyro
  // Set internal clock and full scale operation
  if (GyroWrite(i2c, GYRO_DLPF_FS, dlpf_lpf | GYRO_DLPF_FS_ON) == GYRO_FAIL) {
    printf("GyroInit: Error, could not write 0x%x to register GYRO_DLPF_FS 0x%x\n", (UINT8) dlpf_lpf | GYRO_DLPF_FS_ON, (UINT8) GYRO_DLPF_FS);
    return GYRO_FAIL;
  }
  
  // Set sample rate divider
  // If dlpf_lpf == GYRO_DLPF_LPF_256HZ, sample rate = 8 kHz / sample_rate_div
  // Else, sample rate = 1 kHz / (sample_rate_div + 1)
  if (GyroWrite(i2c, GYRO_SMPLRT_DIV, sample_rate_div) == GYRO_FAIL) {
    printf("GyroInit: Error, could not write 0x%x to register GYRO_DLPF_FS 0x%x\n", (UINT8) dlpf_lpf | GYRO_DLPF_FS_ON, (UINT8) GYRO_DLPF_FS);
    return GYRO_FAIL;
  }
  
  // Select a gyro PLL for clock source (more stable)
  if (GyroWrite(i2c, GYRO_PWR_MGM, power_mgmt_sel) == GYRO_FAIL) {
    printf("GyroInit: Error, could not write 0x%x to register GYRO_DLPF_FS 0x%x\n", (UINT8) dlpf_lpf | GYRO_DLPF_FS_ON, (UINT8) GYRO_DLPF_FS);
    return GYRO_FAIL;
  }
  
  // Set interrupts - NOT BEING USED CURRENTLY
  
  return GYRO_SUCCESS;
}

/************************************************************************************************** 
  Function:
    GYRO_RESULT GyroWrite(const I2C_MODULE i2c, const UINT8 i2c_reg, const UINT8 data)

  Author(s):
    mkobit

  Summary:
    Writes a single byte to an gyro register using the I2C module parameter

  Description:
    Interface for the IMU to use, passes control to the shared I2C library to write by providings the default gyro I2C write address

  Preconditions:
    I2C module previously enabled and running

  Parameters:
    const I2C_MODULE i2c - I2C module to connect with
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
GYRO_RESULT GyroWrite(const I2C_MODULE i2c, const UINT8 i2c_reg, const UINT8 data) {
  if (I2CShared_WriteByte(i2c, GYRO_WRITE, i2c_reg, data)) {
    return GYRO_SUCCESS;
  } else {
    return GYRO_FAIL;
  }
}

/************************************************************************************************** 
  Function:
    GYRO_RESULT GyroRead(const I2C_MODULE i2c, const UINT8 i2c_reg, UINT8 *const buffer)

  Author(s):
    mkobit

  Summary:
    Interface for the IMU to use, reads a single byte into buffer using I2CShared_ReadByte

  Description:
    Passes control to the shared I2C library to read by providing the default gyro I2C read address

  Preconditions:
    I2C module previously enabled and running

  Parameters:
    const I2C_MODULE i2c - I2C module to connect with
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
GYRO_RESULT GyroRead(const I2C_MODULE i2c, const UINT8 i2c_reg, UINT8 *const buffer) {
  if (I2CShared_ReadByte(i2c, GYRO_WRITE,GYRO_READ, i2c_reg, buffer)) {
    return GYRO_SUCCESS;
  } else {
    return GYRO_FAIL;
  }
}


GYRO_RESULT GyroCalibrate(I2C_MODULE i2c, gyro_t *const gyro, int samplesToTake, UINT ms_delay) {
  int samplesTaken = 0;
  float tempOffsets[3];
  GYRO_RESULT res;
  int timeout = 0;

  // Take samples and update the calibration
  do {
    res = GyroReadAllAxes(i2c, gyro, TRUE);
    if (res == GYRO_SUCCESS) {
      tempOffsets[0] += GyroGetX(gyro);
      tempOffsets[1] += GyroGetY(gyro);
      tempOffsets[2] += GyroGetZ(gyro);
      samplesTaken++;
    } else {
      timeout++;
      // TODO constant for timeout
      // If this many read fails, just exit with 
      if (timeout == 5000) {
        return GYRO_FAIL;
      }
    }
    DelayMs(ms_delay);
  } while (samplesTaken < samplesToTake);

  // successful calibration, update the offsets
  gyro->xOffset = (INT16) (tempOffsets[0] / samplesToTake);
  gyro->yOffset = (INT16) (tempOffsets[1] / samplesToTake);
  gyro->zOffset = (INT16) (tempOffsets[2] / samplesToTake);
  return GYRO_SUCCESS;
}

/************************************************************************************************** 
  Function:
    GYRO_RESULT GyroReadAllAxes(const I2C_MODULE i2c, gyro_t *const gyro, const BOOL readTemp)

  Author(s):
    mkobit

  Summary:
    Reads x, y, and z angular momentum data into the (raw) structure, as well as the temperature if readTemp is TRUE

  Description:
    Passes control to the shared I2C library to read several bytes into the buffer starting from the X axis register or the temperature register depending
    on whether or not readTemp is TRUE or FALSE

  Preconditions:
    I2C module previously enabled and running

  Parameters:
    const I2C_MODULE i2c - I2C module to connect with
    gyro_t *const gyro - pointer to raw read data from gyro
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
GYRO_RESULT GyroReadAllAxes(const I2C_MODULE i2c, gyro_t *const gyro, const BOOL readTemp) {
  UINT8 reading_rainbow[8];
  int nDataToRead;
  int offsetForTemp;
  UINT8 startReadI2CReg;
  gyro_raw_t *raw;

  // Get pointer to raw values
  raw = &gyro->raw;
  
  nDataToRead = 6 + (readTemp ? 2 : 0);
  offsetForTemp = 0 + (readTemp ? 2 : 0);
  startReadI2CReg = readTemp ? GYRO_TEMP_OUT_H : GYRO_XOUT_H;
  
  // read x,y, and z data into buffer
  if (I2CShared_ReadMultipleBytes(i2c, GYRO_WRITE, GYRO_READ, startReadI2CReg, nDataToRead, reading_rainbow)) {
    //expand data and place them into the accel_raw_t readings
    
    // if readTemp was true, update temperature
    if (readTemp) {
      raw->temp = ((INT16)reading_rainbow[0] << 8) | reading_rainbow[1];
    }

    // Add offset
    raw->x = ((INT16)reading_rainbow[0 + offsetForTemp] << 8) | reading_rainbow[1 + offsetForTemp] + gyro->xOffset;
    raw->y = ((INT16)reading_rainbow[2 + offsetForTemp] << 8) | reading_rainbow[3 + offsetForTemp] + gyro->yOffset;
    raw->z = ((INT16)reading_rainbow[4 + offsetForTemp] << 8) | reading_rainbow[5 + offsetForTemp] + gyro->zOffset;

    return GYRO_SUCCESS;
  } else {
    return GYRO_FAIL;
  }
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
  float xd;
  gyro_raw_t *raw;

  // Get pointer to raw values
  raw = &gyro->raw;
  
  xd = (float) (raw->x * gyro->xPolarity) / GYRO_CONV_TO_DEGREES * gyro->xGain;
  return xd;
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
  float yd;
  gyro_raw_t *raw;

  // Get pointer to raw values
  raw = &gyro->raw;
  yd = (float) (raw->y * gyro->yPolarity) / GYRO_CONV_TO_DEGREES * gyro->yGain;
  return yd;
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
  float zd;
  gyro_raw_t *raw;

  // Get pointer to raw values
  raw = &gyro->raw;
  zd = (float) (raw->z * gyro->zPolarity) / GYRO_CONV_TO_DEGREES * gyro->zGain;
  return zd;
}

void GyroSetRevPolarity(gyro_t *const gyro, const BOOL xPol, const BOOL yPol, const BOOL zPol) {
  gyro->xPolarity = xPol ? -1 : 1;
  gyro->yPolarity = yPol ? -1 : 1;
  gyro->zPolarity = zPol ? -1 : 1;
}

void GyroSetGains(gyro_t *const gyro, const float xGain, const float yGain, const float zGain) {
  gyro->xGain = xGain;
  gyro->yGain = yGain;
  gyro->zGain = zGain;
}