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

/**************************************************************************************************
  Title: 
    Kalman Filter Test
    
  Version: 
    0.2
    
  Filename: 
    kalmantest-0.2.c
    
  Author(s): 
    mkobit
    ahong
    
  Purpose of Program: 
    Test the kalman filter with transmission of the packets
    
  How to build: 
    delay.c
    imu.c
    i2c_shared.c
    math_helpers.c
    kalman.c
    xbee.c
    
  Update History:
    4/27/12: Changed to test I2C2
    4/27/12: reverted back to I2C1 after bus errors. Changed IMU init to match new library
    4/29/12: Fixed some errors with initialization and deleted a bunch of unused code
    
    
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
      ACCEL_BW_200,
      GYRO_DEFAULT_ADDR,
      GYRO_DLPF_LPF_20HZ,
      4,
      GYRO_PWR_MGM_CLK_SEL_X);

  ImuSetID(p_imu, 0x1);

  if (imu_res == IMU_FAIL) {
      printf("IMU fail\n");
      while(1);
  }

  Kalman_MahonyInit(&kmah);
  Kalman_MadgwickInit(&kmad);
  printf("PB speed = %u\n", pbFreq);
  printf("Kalmans initialized\n");
  XBeeConfigure(TEST_XBEE, pbFreq, XBEE_BAUDRATE);
  printf("XBee UART initialized\n");
  DelayS(2);
  ImuCalibrate(p_imu, TRUE, TRUE, 128, TEST_UPDATE_FREQ);
  printf("Finished IMU calibration.\n\n");
  printf("Update rate = %d ms\n", TEST_UPDATE_FREQ);
  printf("Size of TRANSMIT PACKAGE: %u\n", sizeof(TRANSMIT_PACKAGE)); // debug
  DelayS(3);
  printf("Clearing VT in 2, then wait 1 for acquisition and filtering...\n");
  DelayS(2);
  putsUART2(CLEAR_VT);
  DelayS(1);
  while(1) {
    DelayMs(TEST_UPDATE_FREQ);
#ifndef FILE_SAVE_TEST
    putsUART2(CLEAR_VT);
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
    package.id = ImuGetId(p_imu);
    package.q.q0 = q->q0;
    package.q.q1 = q->q1;
    package.q.q2 = q->q2;
    package.q.q3 = q->q3;
    XBeeSendDataBuffer(TEST_XBEE, (char *) &package, package.n_bytes);
  }

  // Never exit
  while(1);
  return 0;
}