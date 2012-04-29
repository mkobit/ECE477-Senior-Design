#include <p32xxxx.h>
#include <plib.h>
#include "imu.h"

static inline void ImuToggleSelector(imu_t* imu);

/************************************************************************************************** 
  Function:
    IMU_RESULT ImuInit(imu_t *const p_imu,
          const I2C_MODULE i2c,
          const UINT peripheral_clock_speed,
          const UINT i2c_speed,
          const UINT accel_addr,
          const UINT8 accel_range,
          const UINT8 accel_bandwidth,
          const UINT gyro_addr,
          const UINT8 gyro_dlpf_lpf,
          const UINT8 gyro_sample_rate_div,
          const UINT8 gyro_power_mgmt_sel)

  Author(s):
    mkobit

  Summary:
    Initializes IMU and its I2C port

  Description:
    Goes through each step of initializing the I2C port for a particular IMU, as well as writing configuration settings to the accelerometer and gyroscope

  Preconditions:
    Nothing else using I2C port
    I2C not previously configure

  Parameters:
    imu_t *const p_imu - reference to IMU to be initialized
    const I2C_MODULE i2c - I2C module to associate with this IMU
    const UINT peripheral_clock_speed - peripheral bus speed
    const UINT i2c_speed - target I2C bus speed
    const UINT accel_addr - I2C address of accelerometer
    const UINT8 accel_range - range of accelerometer
      ACCEL_SCALE_2G  - 2 G's (gravity)
      ACCEL_SCALE_4G  - 4 G's
      ACCEL_SCALE_8G  - 8 G's
      ACCEL_SCALE_16G - 16 G's
    const UINT8 accel_bandwidth - bandwidth of accelerometer
      ACCEL_BW_1600   - 1600 Hz
      ACCEL_BW_800    - 800 Hz
      ACCEL_BW_400    - 400 Hz
      ACCEL_BW_200    - 200 Hz
      ACCEL_BW_100    - 100 Hz
      ACCEL_BW_50     - 50 Hz
      ACCEL_BW_25     - 25 Hz
    const UINT gyro_addr - I2C address of gyroscope
    const UINT8 dlpf_lpf - low pass filter configuration for sensor acquisition
      GYRO_DLPF_LPF_256HZ    - results in 8 kHz sample rate
      GYRO_DLPF_LPF_188HZ   - results in 1 kHz sample rate
      GYRO_DLPF_LPF_98HZ    - *
      GYRO_DLPF_LPF_42HZ    - *
      GYRO_DLPF_LPF_20HZ    - *
      GYRO_DLPF_LPF_10HZ    - *
      GYRO_DLPF_LPF_5HZ     - *
    const UINT8 sample_rate_div - sample rate divider, F = F_internal / (sample_rate_div + 1)
      e.g. -> 1kHz sample rate from dlpf_lpf, sample_rate_div = 9, F = 1 kHz / (9 _ 1) = 100 Hz 
    const UINT8 power_mgmt_sel - device clock selector
      GYRO_PWR_MGM_CLK_SEL_INTERNAL - internal oscillator
      GYRO_PWR_MGM_CLK_SEL_X        - X as clock reference
      GYRO_PWR_MGM_CLK_SEL_Y        - Y as clock reference
      GYRO_PWR_MGM_CLK_SEL_Z        - Z as clock reference

  Returns:
    IMU_SUCCESS - IMU successfully initialized
    IMU_FAIL - IMU unsuccessfully initialized

  Example:
    <code>
      ImuInit(&imu,
                I2C1,
              40000000L,
              300000,
              ACCEL_RANGE_4G,
              ACCEL_BW_100,
              GYRO_DLPF_LPF_42HZ,
              9,
              GYRO_PWR_MGM_CLK_SEL_X)
    </code>

  Conditions at Exit:
    I2C module initialized
    Accelerometer initialized
    Gyroscope initialized
    *Even on failure, the IMU is associated with the (i2c) provided to this function

**************************************************************************************************/
IMU_RESULT ImuInit(imu_t *const p_imu,
          const I2C_MODULE i2c,
          const UINT peripheral_clock_speed,
          const UINT i2c_speed,
          const UINT accel_addr,
          const UINT8 accel_range,
          const UINT8 accel_bandwidth,
          const UINT gyro_addr,
          const UINT8 gyro_dlpf_lpf,
          const UINT8 gyro_sample_rate_div,
          const UINT8 gyro_power_mgmt_sel) {
                                    
  ACCEL_RESULT accel_init_result;
  GYRO_RESULT gyro_init_result;
  
  // Associate i2c with this IMU
  p_imu->i2c_module = i2c;
  p_imu->isOn = FALSE;

  if (!I2CShared_Init(i2c, peripheral_clock_speed, i2c_speed)) {
    printf("AccelInitI2C: Error, I2C could not be initialized\n");
    return IMU_FAIL;
  }
  // Init both modules of the imu
  accel_init_result = AccelInit(&p_imu->accel, i2c, accel_addr, accel_range, accel_bandwidth);
  if (accel_init_result != ACCEL_SUCCESS) {
      // accel failure, don't try to read line
    p_imu->isOn = FALSE;
    printf("ImuInit: Error, could not complete initialization due to accel fail.\n");
    return IMU_FAIL;
  }
  gyro_init_result = GyroInit(&p_imu->gyro, i2c, gyro_addr, gyro_dlpf_lpf, gyro_sample_rate_div, gyro_power_mgmt_sel);
  
  // Give a semi-random true/false to read accelerometer first  
  p_imu->updateAccelFirst = ReadCoreTimer() % 2 == 0;
  
  // Check if both succeeded in initializing
  if (accel_init_result == ACCEL_SUCCESS && gyro_init_result == GYRO_SUCCESS) {
    p_imu->isOn = TRUE;
    return IMU_SUCCESS;
  } else {
    // failure initializing, do not use this IMU
    p_imu->isOn = FALSE;
    printf("ImuInit: Error, could not complete initialization. Results->(accel, gyro) = (%d, %d)\n", accel_init_result, gyro_init_result);
    return IMU_FAIL;
  }
}

/************************************************************************************************** 
  Function:
    IMU_RESULT ImuUpdate(imu_t *const p_imu)

  Author(s):
    mkobit

  Summary:
    Updates both the gyroscope and accelerometer raw values

  Description:
    Checks if the IMU is online and then updates the accelerometer and gyroscope hased on which was updated first last time. 

  Preconditions:
    ImuInit called

  Parameters:
    imu_t *const p_imu - reference to the IMU being looked at

  Returns:
    IMU_SUCCESS - If successful
    IMU_FAIL - If IMU is not on or read is unsuccessful

  Example:
    <code>
    ImuUpdate(&imu)
    </code>

  Conditions at Exit:
    Gyroscope updated
    Accelerometer updated

**************************************************************************************************/
IMU_RESULT ImuUpdate(imu_t *const p_imu) {
  ACCEL_RESULT a_result;
  GYRO_RESULT g_result;
  
  // Check if device is on first
  if (!ImuIsOn(p_imu)) {
    printf("ImuUpdate: Error, device with I2C=%d is not on\n", p_imu->i2c_module);
    return IMU_FAIL;
  }
  
  if (p_imu->updateAccelFirst) {
    a_result = AccelUpdate(&p_imu->accel);
    if (a_result == ACCEL_FAIL) {
      printf("ImuUpdate: Error, could not update accel at I2C=%d\n", p_imu->i2c_module);
      return IMU_FAIL;
    }
    g_result = GyroUpdate(&p_imu->gyro, TRUE);
  } else {
    g_result = GyroUpdate(&p_imu->gyro, TRUE);
    if (g_result == GYRO_FAIL) {
      printf("ImuUpdate: Error, could not update gyro at I2C=%d\n", p_imu->i2c_module);
      return IMU_FAIL;
    }
    a_result = AccelUpdate(&p_imu->accel);
  }
  
  // Toggle which is updated first for next time
  ImuToggleSelector(p_imu);
  
  // Check for any errors
 if (a_result == ACCEL_SUCCESS && g_result == GYRO_SUCCESS) {
    return IMU_SUCCESS;
  } else {
    // failure updating, do not use this IMU
    printf("ImuUpdate: Error, could not update both accel and gyro at I2C=%d. Results->(accel, gyro) = (%d, %d)\n", p_imu->i2c_module, a_result, g_result);
    return IMU_FAIL;
  }
}

/************************************************************************************************** 
  Function: 
    IMU_RESULT ImuCalibrate(imu_t *const p_imu, BOOL calGyro, BOOL calAccel, int samplesToTake, UINT ms_delay)
  
  Author(s): 
    mkobit
  
  Summary: 
    Calibrates IMU for more accurate readings
  
  Description: 
    Calibrates both the accelerometer and gyroscope by taking their offset they have
  
  Preconditions: 
    IMU initialized successfully
  
  Parameters: 
    imu_t *const p_imu - IMU being reference for this calibration
    BOOL calGyro - if gyro should be calibrated
    BOOL calAccel - if accel should be calibrated
    int samplesToTake - amount of samples to calibrate with
    UINT ms_delay - how long the delay should be before each sample is taken
  
  Returns: 
    IMU_SUCCESS - IMU successfully calibrated
    IMU_FAIL - IMU unsuccessfully calibrated
  
  Example: 
    <code>
    ImuCalibrate(&imu, TRUE, TRUE, 128, 10)
    </code>
  
  Conditions at Exit: 
    Gyro and accelerometer have calculated their offsets
  
**************************************************************************************************/
IMU_RESULT ImuCalibrate(imu_t *const p_imu, BOOL calGyro, BOOL calAccel, int samplesToTake, UINT ms_delay) {
  // Default value for result
  GYRO_RESULT gRes = GYRO_SUCCESS;
  ACCEL_RESULT aRes = ACCEL_SUCCESS;
  
  if (calGyro) {
    gRes = GyroCalibrate(&p_imu->gyro, samplesToTake, ms_delay);
  }

  if (gRes == GYRO_SUCCESS && calAccel) {
    // TODO: currently no calibration for accelerometer
  }

  return (aRes == ACCEL_SUCCESS && gRes == GYRO_SUCCESS) ? IMU_SUCCESS : IMU_FAIL;
}

/************************************************************************************************** 
  Function: 
    void ImuSetID(imu_t *const p_imu, const imu_id id)
  
  Author(s): 
    mkobit
  
  Summary: 
    Assigns an ID to this IMU
  
  Description: 
    Same as summary
  
  Preconditions: 
    None
  
  Parameters: 
    imu_t *const p_imu - IMU to operation
    const imu_id id - id to 
  
  Returns: 
    void
  
  Example: 
    <code>
    ImuSetID(&imu, 0)
    </code>
  
  Conditions at Exit: 
    IMU assigned this ID
  
**************************************************************************************************/
void ImuSetID(imu_t *const p_imu, const imu_id id) {
    p_imu->id = id;
}

/************************************************************************************************** 
  Function: 
    imu_id ImuGetId(const imu_t *const p_imu)
  
  Author(s): 
    mkobit
  
  Summary: 
    Gets ID of this IMU
  
  Description: 
    Same as summary
  
  Preconditions: 
    ImuSetID called
  
  Parameters: 
    imu_t *const p_imu - IMU to get ID from
  
  Returns: 
    imu_id id - ID of this IMU
  
  Example: 
    <code>
    ImuGetId(&imu)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
imu_id ImuGetId(const imu_t *const p_imu) {
  return (p_imu->id);
}

/************************************************************************************************** 
  Function: 
    void ImuResetI2CBus(const imu_t *p_imu)
  
  Author(s): 
    mkobit
  
  Summary: 
    Resets I2C bus that this IMU is using
  
  Description: 
    Calls I2CShared_ResetBus to reset bus
  
  Preconditions: 
    IMU initialized
  
  Parameters: 
    imu_t *const p_imu - IMU who's I2C bus to reset
  
  Returns: 
    void
  
  Example: 
    <code>
    ImuResetI2CBus(&imu)
    </code>
  
  Conditions at Exit: 
    I2C bus reset
  
**************************************************************************************************/
void ImuResetI2CBus(const imu_t *p_imu) {
  I2CShared_ResetBus(p_imu->i2c_module);
}

/************************************************************************************************** 
  Function: 
    accel_t *ImuGetAccel(imu_t *const p_imu)
  
  Author(s): 
    mkobit
  
  Summary: 
    Returns a pointer to the accelerometer associated with this IMU
  
  Description: 
    Same as description
  
  Preconditions: 
    IMU initialized
  
  Parameters: 
    imu_t *const p_imu - IMU to get its accelerometer from
  
  Returns: 
    accel_t *accel - pointer to accelerometer associated with this IMU
  
  Example: 
    <code>
    accel_t *p_accel = ImuGetAccel(&imu)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
accel_t *ImuGetAccel(imu_t *const p_imu) {
  return &p_imu->accel;
}

/************************************************************************************************** 
  Function:
    accel_raw_t *ImuGetRawAccel(imu_t *const p_imu)

  Author(s):
    mkobit

  Summary:
    Gives a reference to the last read raw accelerometer data

  Description:
    Same as summary

  Preconditions:
    ImuInit called
    ImuUpdate typically called

  Parameters:
    imu_t *const p_imu - IMU to get raw data from

  Returns:
    accel_raw_t * - reference to imu's raw accelerometer data

  Example:
    <code>
    accel_raw_t *raw_a = ImuGetRawAccel(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
accel_raw_t *ImuGetRawAccel(imu_t *const p_imu) {
  return &(p_imu->accel.raw);
}

/************************************************************************************************** 
  Function: 
    gyro_t *ImuGetGyro(imu_t *const p_imu)
  
  Author(s): 
    mkobit
  
  Summary: 
    Returns a pointer to the gyro associated with this IMU
  
  Description: 
    Same as description
  
  Preconditions: 
    IMU initialized
  
  Parameters: 
    imu_t *const p_imu - IMU to get its gyro from
  
  Returns: 
    gyro_t *gyro - pointer to gyro associated with this IMU
  
  Example: 
    <code>
    gyro_t *p_gyro = ImuGetGyro(&imu)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
gyro_t *ImuGetGyro(imu_t *const p_imu) {
  return &p_imu->gyro;
}

/************************************************************************************************** 
  Function:
    gyro_raw_t *ImuGetRawGyro(imu_t *const p_imu)

  Author(s):
    mkobit

  Summary:
    Gives a reference to the last read raw gyroscope data

  Description:
    Same as summary

  Preconditions:
    ImuInit called
    ImuUpdate typically called
    
  Parameters:
    imu_t *const p_imu - IMU to get raw data from

  Returns:
    gyro_raw_t * - reference to imu's raw gyro data

  Example:
    <code>
    gyro_raw_t *raw_g = ImuGetRawGyro(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
gyro_raw_t *ImuGetRawGyro(imu_t *const p_imu) {
  return &(p_imu->gyro.raw);
}

/************************************************************************************************** 
  Function:
    BOOL ImuIsOn(const imu_t *const p_imu)

  Author(s):
    mkobit

  Summary:
    Tells calling program if the IMU is online or offline

  Description:
    Same as summary

  Preconditions:
    ImuInit called

  Parameters:
    const imu_t *const p_imu - reference to the IMU being used

  Returns:
    TRUE - IMU is online
    FALSE - IMU is offline

  Example:
    <code>
    ImuIsOn(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
BOOL ImuIsOn(const imu_t *const p_imu) {
  return p_imu->isOn;
}

/************************************************************************************************** 
  Function:
    float ImuGetGyroTemp(const imu_t *const p_imu)

  Author(s):
    mkobit

  Summary:
    Gets temperature of gyroscope in terms of Celsius

  Description:
    Calls GyroGetTemp

  Preconditions:
    ImuInit called
    ImuUpdate called

  Parameters:
    const imu_t *const p_imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    float gtemp - gyroscope temperature

  Example:
    <code>
    float gtemp;
    gtemp = ImuGetGyroTemp(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
float ImuGetGyroTemp(imu_t *const p_imu) {
  float gtemp;
  gtemp = GyroGetTemp(&p_imu->gyro);
  return gtemp;
}

/************************************************************************************************** 
  Function:
    float ImuGetGyroX(const imu_t *const p_imu)

  Author(s):
    mkobit

  Summary:
    Gets angular momentum, roll, in terms of 'degrees / s'

  Description:
    Calls GyroGetX

  Preconditions:
    ImuInit called
    ImuUpdate called

  Parameters:
    const imu_t *const p_imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    float gx - gyroscope roll in terms of degrees/s

  Example:
    <code>
    float gx;
    gx = ImuGetGyroX(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
float ImuGetGyroX(imu_t *const p_imu) {
  float gx;
  gx = GyroGetX(&p_imu->gyro);
  return gx;
}

/************************************************************************************************** 
  Function:
    float ImuGetGyroY(const imu_t *const p_imu)

  Author(s):
    mkobit

  Summary:
    Gets angular momentum, pitch, in terms of 'degrees / s'

  Description:
    Calls GyroGetY

  Preconditions:
    ImuInit called
    ImuUpdate called

  Parameters:
    const imu_t *const p_imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    float gy - gyroscope pitch in terms of degrees/s

  Example:
    <code>
    float gz;
    gz = ImuGetGyroY(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
float ImuGetGyroY(imu_t *const p_imu) {
  float gy;
  gy = GyroGetY(&p_imu->gyro);
  return gy;
}

/************************************************************************************************** 
  Function:
    float ImuGetGyroZ(const imu_t *const p_imu)

  Author(s):
    mkobit

  Summary:
    Gets angular momentum, yaw, in terms of 'degrees / s'

  Description:
    Calls GyroGetZ
    
  Preconditions:
    ImuInit called
    ImuUpdate called

  Parameters:
    const imu_t *const p_imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    float gz - gyroscope yaw in terms of degrees/s

  Example:
    <code>
    float gz;
    gz = ImuGetGyroZ(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
float ImuGetGyroZ(imu_t *const p_imu) {
  float gz;
  gz = GyroGetZ(&p_imu->gyro);
  return gz;
}

/************************************************************************************************** 
  Function:
    float ImuGetAccelX(const imu_t *const p_imu)

  Author(s):
    mkobit

  Summary:
    Gets acceleration in X direction in terms of 'g's

  Description:
    Calls AccelGetX  

  Preconditions:
    ImuInit called
    ImuUpdate called

  Parameters:
    const imu_t *const p_imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    float ax - accelerometer value in X in terms of 'g's

  Example:
    <code>
    float az;
    az = ImuGetAccelZ(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
float ImuGetAccelX(imu_t *const p_imu) {
  float ax;
  ax = AccelGetX(&p_imu->accel);
  return ax;
}

/************************************************************************************************** 
  Function:
    float ImuGetAccelY(const imu_t *const p_imu)

  Author(s):
    mkobit

  Summary:
    Gets acceleration in Y direction in terms of 'g's

  Description:
    Calls AccelGetY
    
  Preconditions:
    ImuInit called
    ImuUpdate called

  Parameters:
    const imu_t *const p_imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    float ay - accelerometer value in Y in terms of 'g's

  Example:
    <code>
    float ay;
    ay = ImuGetAccelY(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
float ImuGetAccelY(imu_t *const p_imu) {
  float ay;
  ay = AccelGetY(&p_imu->accel);
  return ay;
}

/************************************************************************************************** 
  Function:
    float ImuGetAccelZ(const imu_t *const p_imu)

  Author(s):
    mkobit

  Summary:
    Gets acceleration in Z direction in terms of 'g's

  Description:
    Calls AccelGetZ

  Preconditions:
    ImuInit called
    ImuUpdate called

  Parameters:
    const imu_t *const p_imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    float az - accelerometer value in Z in terms of 'g's

  Example:
    <code>
    float az;
    az = ImuGetAccelZ(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
float ImuGetAccelZ(imu_t *const p_imu) {
  float az;
  az = AccelGetZ(&p_imu->accel);
  return az;
}

/************************************************************************************************** 
  Function:
    static inline void ImuToggleSelector(const imu_t *const p_imu)

  Author(s):
    mkobit

  Summary:
    Sets the IMU to read the other module first

  Description:
    Toggled after every update. Accel readings will happen first every other time because of this
    Static function, used by internal library

  Preconditions:
    ImuInit called

  Parameters:
    imu_t *p_imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    void

  Example:
    <code>
    ImuToggleSelector(&imu)
    </code>

  Conditions at Exit:
    IMU will read the gyro/accel first next time

**************************************************************************************************/
static inline void ImuToggleSelector(imu_t *p_imu) {
  p_imu->updateAccelFirst = !p_imu->updateAccelFirst;
}