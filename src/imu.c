#include <p32xxxx.h>
#include <plib.h>
#include "imu.h"

static imu_id IMU_ID = 0;     // Assign a unique ID to each imu that calls init
static inline void ImuToggleSelector(imu_t* imu);

/************************************************************************************************** 
  Function:
    ImuInit(imu_t* imu,
          I2C_MODULE i2c,
          unsigned int peripheral_clock_speed, 
          unsigned int i2c_speed, 
          unsigned char accel_resolution,
          unsigned char accel_bandwidth,
          unsigned char gyro_dlpf_lpf,
          unsigned char gyro_sample_rate_div,

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
    imu_t* imu - reference to IMU to be initialized
    I2C_MODULE i2c - I2C module to associate with this IMU
    unsigned int peripheral_clock_speed - peripheral bus speed
    unsigned int i2c_speed - target I2C bus speed
    unsigned char accel_range - range of accelerometer
      ACCEL_SCALE_2G  - 2 G's (gravity)
      ACCEL_SCALE_4G  - 4 G's
      ACCEL_SCALE_8G  - 8 G's
      ACCEL_SCALE_16G - 16 G's
    unsigned char accel_bandwidth - bandwidth of accelerometer
      ACCEL_BW_1600   - 1600 Hz
      ACCEL_BW_800    - 800 Hz
      ACCEL_BW_400    - 400 Hz
      ACCEL_BW_200    - 200 Hz
      ACCEL_BW_100    - 100 Hz
      ACCEL_BW_50     - 50 Hz
      ACCEL_BW_25     - 25 Hz
    unsigned char dlpf_lpf - low pass filter configuration for sensor acquisition
      GYRO_DLPF_LPF_256HZ    - results in 8 kHz sample rate
      GYRO_DLPF_LPF_188HZ   - results in 1 kHz sample rate
      GYRO_DLPF_LPF_98HZ    - *
      GYRO_DLPF_LPF_42HZ    - *
      GYRO_DLPF_LPF_20HZ    - *
      GYRO_DLPF_LPF_10HZ    - *
      GYRO_DLPF_LPF_5HZ     - *
    unsigned char sample_rate_div - sample rate divider, F = F_internal / (sample_rate_div + 1)
      e.g. -> 1kHz sample rate from dlpf_lpf, sample_rate_div = 9, F = 1 kHz / (9 _ 1) = 100 Hz 
    unsigned char power_mgmt_sel - device clock selector
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

**************************************************************************************************/
IMU_RESULT ImuInit(imu_t* imu,
          I2C_MODULE i2c,
          unsigned int peripheral_clock_speed, 
          unsigned int i2c_speed, 
          unsigned char accel_range,
          unsigned char accel_bandwidth,
          unsigned char gyro_dlpf_lpf,
          unsigned char gyro_sample_rate_div, 
          unsigned char gyro_power_mgmt_sel) {
                                    
  unsigned int actualClock;
  ACCEL_RESULT accel_init_result;
  GYRO_RESULT gyro_init_result;
  
  if (!I2CShared_Init(i2c, peripheral_clock_speed, i2c_speed)) {
    printf("AccelInitI2C: Error, I2C could not be initted\n", actualClock);
    return IMU_FAIL;
  }
  // Init both modules of the imu
  accel_init_result = AccelInit(i2c, accel_range, accel_bandwidth, &imu->accel_raw);
  if (accel_init_result != ACCEL_SUCCESS) {
      // accel failure, don't try to read line
    imu->isOn = FALSE;
    printf("ImuInit: Error, could not complete initialization due to accel fail.\n");
    return IMU_FAIL;
  }
  gyro_init_result = GyroInit(i2c, gyro_dlpf_lpf, gyro_sample_rate_div, gyro_power_mgmt_sel);
  
  // Give a semi-random true/false to read accelerometer first  
  imu->updateAccelFirst = ReadCoreTimer() % 2 == 0;
  
  // Check if both succeeded in initializing
  if (accel_init_result == ACCEL_SUCCESS && gyro_init_result == GYRO_SUCCESS) {
    imu->isOn = TRUE;
    // Assign the IMU an ID and increment the ID
    imu->id = IMU_ID; // TODO if this does not work move responsibility to main program to initialize the ID
    IMU_ID++;
    return IMU_SUCCESS;
  } else {
    // failure initializing, do not use this IMU
    imu->isOn = FALSE;
    printf("ImuInit: Error, could not complete initialization. Results->(accel, gyro) = (%d, %d)\n", accel_init_result, gyro_init_result);
    return IMU_FAIL;
  }
}

/************************************************************************************************** 
  Function:
    IMU_RESULT ImuUpdate(imu_t *imu)

  Author(s):
    mkobit

  Summary:
    Updates both the gyroscope and accelerometer raw values

  Description:
    Checks if the IMU is online and then updates the accelerometer and gyroscope hased on which was updated first last time. 

  Preconditions:
    ImuInit called

  Parameters:
    imu_t *imu - reference to the IMU being looked at

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
IMU_RESULT ImuUpdate(imu_t *imu) {
  ACCEL_RESULT a_result;
  GYRO_RESULT g_result;
  
  // Check if device is on first
  if (!ImuIsOn(imu)) {
    printf("ImuUpdate: Error, device with I2C=%d is not on\n", imu->i2c);
    return IMU_FAIL;
  }
  
  if (imu->updateAccelFirst) {
    a_result = AccelReadAllAxes(imu->i2c_module, &imu->accel_raw);
    g_result = GyroReadAllAxes(imu->i2c_module, &imu->gyro_raw, TRUE);
  } else {
    g_result = GyroReadAllAxes(imu->i2c_module, &imu->gyro_raw, TRUE);
    a_result = AccelReadAllAxes(imu->i2c_module, &imu->accel_raw);
  }
  
  // Toggle which is updated first for next time
  ImuToggleSelector(imu);
  
  // Check for any errors
 if (a_result == ACCEL_SUCCESS && g_result == GYRO_SUCCESS) {
    return IMU_SUCCESS;
  } else {
    // failure updating, do not use this IMU
    printf("ImuUpdate: Error, could not update both accel and gyro at I2C=%d. Results->(accel, gyro) = (%d, %d)\n", imu->i2c, a_result, g_result);
    return IMU_FAIL;
  }
}

/************************************************************************************************** 
  Function:
    accel_raw_t *ImuGetRawAccel(imu_t *imu)

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
    imu_t *imu - reference to the IMU being looked at

  Returns:
    accel_raw_t * - reference to imu's raw accelerometer data

  Example:
    <code>
    accel_raw_t *raw_a
    raw_a = ImuGetRawGyro(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
accel_raw_t *ImuGetRawAccel(imu_t *imu) {
  return &imu->accel_raw;
}

/************************************************************************************************** 
  Function:
    gyro_raw_t *ImuGetRawGyro(imu_t *imu)

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
    imu_t *imu - reference to the IMU being looked at

  Returns:
    gyro_raw_t * - reference to imu's raw gyro data

  Example:
    <code>
    gyro_raw_t *raw_g
    raw_g = ImuGetRawGyro(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
gyro_raw_t *ImuGetRawGyro(imu_t *imu) {
  return &imu->gyro_raw;
}

/************************************************************************************************** 
  Function:
    BOOL ImuIsOn(imu_t *imu)

  Author(s):
    mkobit

  Summary:
    Tells calling program if the IMU is online or offline

  Description:
    Same as summary

  Preconditions:
    ImuInit called

  Parameters:
    imu_t *imu - reference to the IMU being used

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
BOOL ImuIsOn(imu_t *imu) {
  return imu->isOn;
}

/************************************************************************************************** 
  Function:
    

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
    imu_t *imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    double gtemp - gyroscope temperature

  Example:
    <code>
    double gtemp;
    gtemp = ImuGetGyroTemp(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
double ImuGetGyroTemp(imu_t *imu) {
  double gtemp;
  gtemp = GyroGetTemp(&imu->gyro_raw);
  return gtemp;
}

/************************************************************************************************** 
  Function:
    double ImuGetGyroX(imu_t *imu)

  Author(s):
    mkobit

  Summary:
    Gets angular momentum, roll, in terms of 'degrees / s'

  Description:
    Calls ImuGetGyroX

  Preconditions:
    ImuInit called
    ImuUpdate called

  Parameters:
    imu_t *imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    double gx - gyroscope roll in terms of degrees/s

  Example:
    <code>
    double gx;
    gx = ImuGetGyroX(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
double ImuGetGyroX(imu_t *imu) {
  double gx;
  gx = GyroGetX(&imu->gyro_raw);
  return gx;
}

/************************************************************************************************** 
  Function:
    double ImuGetGyroY(imu_t *imu)

  Author(s):
    mkobit

  Summary:
    Gets angular momentum, pitch, in terms of 'degrees / s'

  Description:
    Calls ImuGetGyroY

  Preconditions:
    ImuInit called
    ImuUpdate called

  Parameters:
    imu_t *imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    double gy - gyroscope pitch in terms of degrees/s

  Example:
    <code>
    double gz;
    gz = ImuGetGyroZ(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
double ImuGetGyroY(imu_t *imu) {
  double gy;
  gy = GyroGetX(&imu->gyro_raw);
  return gy;
}

/************************************************************************************************** 
  Function:
    double ImuGetGyroZ(imu_t *imu)

  Author(s):
    mkobit

  Summary:
    Gets angular momentum, yaw, in terms of 'degrees / s'

  Description:
    Calls ImuGetGyroZ
    
  Preconditions:
    ImuInit called
    ImuUpdate called

  Parameters:
    imu_t *imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    double gz - gyroscope yaw in terms of degrees/s

  Example:
    <code>
    double gz;
    gz = ImuGetGyroZ(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
double ImuGetGyroZ(imu_t *imu) {
  double gz;
  gz = GyroGetX(&imu->gyro_raw);
  return gz;
}

/************************************************************************************************** 
  Function:
    double ImuGetAccelX(imu_t *imu)

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
    imu_t *imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    double ax - accelerometer value in X in terms of 'g's

  Example:
    <code>
    double az;
    az = ImuGetAccelZ(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
double ImuGetAccelX(imu_t *imu) {
  double ax;
  ax = AccelGetZ(&imu->accel_raw);
  return ax;
}

/************************************************************************************************** 
  Function:
    double ImuGetAccelY(imu_t *imu)

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
    imu_t *imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    double ay - accelerometer value in Y in terms of 'g's

  Example:
    <code>
    double az;
    az = ImuGetAccelZ(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
double ImuGetAccelY(imu_t *imu) {
  double ay;
  ay = AccelGetZ(&imu->accel_raw);
  return ay;
}

/************************************************************************************************** 
  Function:
    double ImuGetAccelZ(imu_t *imu)

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
    imu_t *imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    double az - accelerometer value in Z in terms of 'g's

  Example:
    <code>
    double az;
    az = ImuGetAccelZ(&imu)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
double ImuGetAccelZ(imu_t *imu) {
  double az;
  az = AccelGetZ(&imu->accel_raw);
  return az;
}

/************************************************************************************************** 
  Function:
    static inline void ImuToggleSelector(imu_t* imu)

  Author(s):
    mkobit

  Summary:
    Sets the IMU to read the other module first

  Description:
    Toggled after every update. Accel readings will happen first every other time because of this

  Preconditions:
    ImuInit called

  Parameters:
    imu_t *imu - pointer to imu containing raw accelerometer and raw gyroscope data

  Returns:
    void

  Example:
    <code>
    ImuToggleSelector(&imu)
    </code>

  Conditions at Exit:
    IMU will read the gyro/accel first next time

**************************************************************************************************/
static inline void ImuToggleSelector(imu_t* imu) {
  imu->updateAccelFirst = !imu->updateAccelFirst;
}