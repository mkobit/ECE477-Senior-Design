#include <plib.h>
#include <p32xxxx.h>

#include "configs.h"

#pragma config DEBUG = ON
#pragma config ICESEL = ICS_PGx1

#include "imu.h"
#include "delay.h"
#include "i2c_shared.h"
#include "kalman.h"
#include "math_helpers.h"
#include "xbee.h"



#define TEST_UPDATE_FREQ 10

#define USE_MADGWICK 1
#define USE_MAHONY 2
#define KALMAN_SELECT USE_MAHONY

#define TEST_TIME_S 5000

// IDs that will be used in main program
#define ID_UPPER_ARM 0
#define ID_FORE_ARM 1
#define ID_HAND 2

// This can be changed to determine which IMU will be used
#define TESTING_BODYPART ID_HAND

typedef struct TRANSMIT_PACKAGE {
  UINT8 n_bytes;
  imu_id id;
  QUATERNION q;
} TRANSMIT_PACKAGE;

// Clock Constants
#define SYS_CLOCK (80000000L)
#define GetSystemClock()            (SYS_CLOCK)
#define GetInstructionClock()       (SYS_CLOCK)

#define TEST_I2C_BUS_ID              I2C1
#define TEST_I2C_BUS_SPEED           (400000)
#define BAUDRATE 57600

// Gains that will be tested on Kalman Filter
#define TEST_BETA_DEF (KALMAN_DEFAULT_BETADEF * 32)
#define TEST_TWOKPDEF (KALMAN_DEFAULT_TWOKPDEF * 32)
#define TEST_TWOKIDEF (KALMAN_DEFAULT_TWOKIDEF * 32)

#define TEST_XBEE UART3

void PrintOutPackage(TRANSMIT_PACKAGE *package);
void SetPackageData(const imu_id id, QUATERNION *q, TRANSMIT_PACKAGE *package);

/**************************************************************************************************
  Title: 
    Kalman Filter Test
    
  Version: 
    0.22
    
  Filename: 
    kalmantest-0.22.c
    
  Author(s): 
    mkobit
    
  Purpose of Program: 
    Testing Kalman filter with transmission on the PCB
    
  How to build: 
    delay.c
    imu.c
    i2c_shared.c
    math_helpers.c
    kalman.c
    xbee.c
    
  Update History:
        
    
**************************************************/
int main() {
  imu_t imu;
  imu_t *p_imu;
  IMU_RESULT imu_res = IMU_FAIL;
  unsigned int pbFreq;
  KALMAN_STATE_MADGWICK kmad;
  KALMAN_STATE_MAHONY kmah;
  EULER_ANGLES e;
  QUATERNION *q;
  float updateRate;
  TRANSMIT_PACKAGE package;

  package.n_bytes = sizeof(TRANSMIT_PACKAGE);


  pbFreq = SYSTEMConfigPerformance(GetSystemClock());
  DelayInit(pbFreq);
  p_imu = &imu;
  imu_res = ImuInit(p_imu,
      TEST_I2C_BUS_ID,
      pbFreq,
      TEST_I2C_BUS_SPEED,
      ACCEL_DEFAULT_ADDR,
      ACCEL_RANGE_2G,
      ACCEL_BW_100,
      GYRO_DEFAULT_ADDR,
      GYRO_DLPF_LPF_20HZ,
      9,
      GYRO_PWR_MGM_CLK_SEL_X);

  ImuSetID(p_imu, TESTING_BODYPART);

  if (imu_res == IMU_FAIL) {
      while(1);
  }

  Kalman_MahonyInit(&kmah);
  Kalman_MadgwickInit(&kmad);
  DelayS(2);
  Kalman_MadgwickSetGains(TEST_BETA_DEF);
  Kalman_MahonySetGains(TEST_TWOKPDEF, TEST_TWOKIDEF);
  XBeeConfigure(TEST_XBEE, pbFreq, BAUDRATE);
  DelayS(1);
  imu_res = ImuCalibrate(p_imu, TRUE, TRUE, 256, TEST_UPDATE_FREQ);
  DelayS(2);

  while(1) {
    DelayMs(TEST_UPDATE_FREQ);
    imu_res = ImuUpdate(p_imu);
    if (imu_res == IMU_FAIL) {
      ImuResetI2CBus(p_imu);
      DelayMs(50);
      imu_res = ImuUpdate(p_imu);
    }
    updateRate = 1.0f / (float) TEST_UPDATE_FREQ * 1000.0f;
#if (KALMAN_SELECT == USE_MADGWICK)
    Kalman_MadgwickUpdate(p_imu, &kmad, updateRate);
#elif (KALMAN_SELECT == USE_MAHONY)
    Kalman_MahonyUpdate(p_imu, &kmah, updateRate);
#endif
#if (KALMAN_SELECT == USE_MADGWICK)
    q = &(kmad.q);
    MHelpers_QuaternionToEuler(q, &e);
#elif (KALMAN_SELECT == USE_MAHONY)
    q = &(kmah.q);
    MHelpers_QuaternionToEuler(q, &e);
#endif

    // Test xbee sending
    SetPackageData(ImuGetId(p_imu), q, &package);
    XBeeSendDataBuffer(TEST_XBEE, (char *) &package, package.n_bytes);

    q = &(kmad.q);

    MHelpers_QuaternionToEuler(q, &e);

  }

  // Never exit
  while(1);
  return 0;
}

void PrintOutPackage(TRANSMIT_PACKAGE *package) {
  printf("%u%u%f%f%f%f\n", package->n_bytes, package->n_bytes, package->q.q0, package->q.q1, package->q.q2, package->q.q3);
}

void SetPackageData(const imu_id id, QUATERNION *q, TRANSMIT_PACKAGE *package) {
  package->id = id;
  package->q.q0 = q->q0;
  package->q.q1 = q->q1;
  package->q.q2 = q->q2;
  package->q.q3 = q->q3;
}