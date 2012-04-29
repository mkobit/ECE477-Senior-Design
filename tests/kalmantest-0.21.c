#include <plib.h>
#include <p32xxxx.h>
//#include <stdio.h>
#include "imu.h"
#include "delay.h"
#include "i2c_shared.h"
#include "kalman.h"
#include "math_helpers.h"
#include "xbee.h"

#define TEST_UPDATE_FREQ 10

#define KALMAN_SELECT 2

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

//#define FILE_SAVE_TEST
#define FILE_SAVE_LINES 400

// Configuration Bit settings
//
// SYSCLK = 80 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 40 MHz
// WDT OFF

#pragma config FPLLMUL = MUL_20
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLODIV = DIV_1
#pragma config POSCMOD = HS
#pragma config FNOSC = PRIPLL
#pragma config FWDTEN = OFF // watchdog off
#pragma config FPBDIV = DIV_1

// Clock Constants
#define SYS_CLOCK (80000000L)
#define GetSystemClock()            (SYS_CLOCK)
#define GetInstructionClock()       (SYS_CLOCK)

#define TEST_I2C_BUS_ID              I2C1
#define TEST_I2C_BUS_SPEED           (400000)
#define BAUDRATE 57600

#define CLEAR_VT "\033[2J"
#define NEW_LINE_MODE "\033[20h"

#define TEST_XBEE UART1

void PrintOutPackage(TRANSMIT_PACKAGE *package);
void SetPackageData(const imu_id id, QUATERNION *q, TRANSMIT_PACKAGE *package);

/**************************************************************************************************
  Title: 
    Kalman Filter Test
    
  Version: 
    0.2
    
  Filename: 
    kalmantest-0.2.c
    
  Author(s): 
    mkobit
    
  Purpose of Program: 
    Either test the kalman filter by capturing the output in HyperTerminal 3 times,
    once for hand, once for arm, and once for shoulder to get data for each
    point and test it on the rendering side, or just directly test sending the
    Kalman filter data live be XBee
    
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
  us_t start, end;
  EULER_ANGLES e;
  QUATERNION *q;
  unsigned int secsPassed = 0;
  YPR ypr;
  float updateRate;
  unsigned int reInitKalman = 0;
  TRANSMIT_PACKAGE package;

  package.n_bytes = sizeof(TRANSMIT_PACKAGE);

#ifdef FILE_SAVE_TEST
  int i = 0;
#endif


  pbFreq = SYSTEMConfigPerformance(GetSystemClock());
  OpenUART2(UART_EN | UART_NO_PAR_8BIT | UART_1STOPBIT, UART_RX_ENABLE | UART_TX_ENABLE, (pbFreq/16/BAUDRATE) - 1);
  DelayInit(pbFreq);
  putsUART2(CLEAR_VT);
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
      printf("IMU fail\n");
      while(1);
  }

  printf("PB speed = %u\n", pbFreq);
  Kalman_MahonyInit(&kmah);
  Kalman_MadgwickInit(&kmad);
  DelayS(1);
  printf("Kalmans initialized\n");
  XBeeConfigure(TEST_XBEE, pbFreq, XBEE_BAUDRATE);
  printf("XBee UART initialized\n");
  DelayS(1);
  ImuCalibrate(p_imu, TRUE, TRUE, 128, TEST_UPDATE_FREQ);
  printf("Finished IMU calibration.\n");
  DelayS(1);
  printf("Update rate = %d ms\n", TEST_UPDATE_FREQ);
  DelayS(1);
  printf("Size of TRANSMIT PACKAGE: %u\n", sizeof(TRANSMIT_PACKAGE)); // debug
  if (ImuGetId(p_imu) == ID_FORE_ARM) {
    printf("Testing fore arm (ID_FORE_ARM, %d) IMU\n", TESTING_BODYPART);
  } else if(ImuGetId(p_imu) == ID_UPPER_ARM) {
    printf("Testing upper arm, (ID_UPPER_ARM IMU, %d) IMU\n", TESTING_BODYPART);
  } else if (ImuGetId(p_imu) == ID_HAND) {
    printf("Testing hand, (ID_HAND IMU, %d) IMU IMU\n", TESTING_BODYPART);
  }
  DelayS(1);
  printf("\nThis test will last for %d seconds\n", TEST_TIME_S);
  DelayS(1);
  printf("Clearing VT in 1, then wait 5 for acquisition and filtering...\n");
  DelayS(1);
  putsUART2(CLEAR_VT);
  DelayS(5);

  // Initialize time for the test
  start = DelayUtilGetUs();
  printf("Starting\n");
  while(1) {
    DelayMs(TEST_UPDATE_FREQ);
#ifndef FILE_SAVE_TEST
    //putsUART2(CLEAR_VT);
#endif
    imu_res = ImuUpdate(p_imu);
    if (imu_res == IMU_FAIL) {
      ImuResetI2CBus(p_imu);
      DelayMs(50);
      imu_res = ImuUpdate(p_imu);
    }
    updateRate = 1.0f / (float) TEST_UPDATE_FREQ * 1000.0f;
#if (KALMAN_SELECT == 1)
    Kalman_MadgwickUpdate(p_imu, &kmad, updateRate);
#elif (KALMAN_SELECT == 2)

    Kalman_MahonyUpdate(p_imu, &kmah, updateRate);
#endif
#ifndef FILE_SAVE_TEST
    
    //printf("Ax = %7.3f, Ay = %7.3f, Az = %7.3f\n", ImuGetAccelX(p_imu), ImuGetAccelY(p_imu), ImuGetAccelZ(p_imu));
    //printf("Gx = %7.3f, Gy = %7.3f, Gz = %7.3f, Gt = %7.3f\n\n", ImuGetGyroX(p_imu), ImuGetGyroY(p_imu), ImuGetGyroZ(p_imu), ImuGetGyroTemp(p_imu));
#if (KALMAN_SELECT == 1)
    q = &(kmad.q);
    MHelpers_QuaternionToEuler(q, &e);
#elif (KALMAN_SELECT == 2)
    q = &(kmah.q);
    MHelpers_QuaternionToEuler(q, &e);
#endif
#else
    q = &(kmad.q);
    //printf("%f,%f,%f,%f\n", q->q0, q->q1, q->q2, q->q3);
    if (i++ == FILE_SAVE_LINES) {
      break;
    }
#endif
    /*if (reInitKalman++ == 25000) {
      Kalman_MahonyInit(&kmah);
      Kalman_MadgwickInit(&kmad);
      reInitKalman = 0;
      //printf("\nREINITIALIZED\n");
      DelayMs(TEST_UPDATE_FREQ);
    }*/

    
    // Test xbee sending
    SetPackageData(ImuGetId(p_imu), q, &package);
    //PrintOutPackage(&package);
    XBeeSendDataBuffer(TEST_XBEE, (char *) &package, package.n_bytes);

    // Determine if the test has ended
    end = DelayUtilGetUs();
    if (secsPassed >= TEST_TIME_S) {
      break;
    }else if (DelayUtilElapsedUs(end, start) % 1000000  < 10000 ) {
      ++secsPassed;
      printf ("%d seconds gone by, will stop at %d\n", secsPassed, TEST_TIME_S);
    }
  }

  //putsUART2(CLEAR_VT);
  printf("Test completed\n");
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