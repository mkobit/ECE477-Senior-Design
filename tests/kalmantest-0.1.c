#include <plib.h>
#include <p32xxxx.h>
#include <stdio.h>
#include "imu.h"
#include "delay.h"
#include "i2c_shared.h"
#include "kalman.h"
#include "math_helpers.h"

#define TEST_UPDATE_FREQ 10

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


/**************************************************************************************************
  Title: 
    Kalman Filter Test
    
  Version: 
    0.1
    
  Filename: 
    kalmantest-0.1.c
    
  Author(s): 
    mkobit
    
  Purpose of Program: 
    Test both versions of the Kalman filters
    
  How to build: 
    delay.c
    imu.c
    i2c_shared.c
    math_helpers.
    kalman.c
    
  Update History: 
    4/25/12: Increased update rate and delay before output so I could capture with HyperTerminal and send it so Steve for debugging
    
**************************************************/
int main() {
  imu_t imu;
  imu_t *p_imu;
  IMU_RESULT imu_res = IMU_FAIL;
  unsigned int pbFreq;
  us_t t1, t2, t3, t4, t5, t6;
  KALMAN_STATE_MADGWICK kmad;
  KALMAN_STATE_MAHONY kmah;
  EULER_ANGLES e;
  QUATERNION *q;
#ifdef FILE_SAVE_TEST
  int i = 0;
#endif


  pbFreq = SYSTEMConfigPerformance(GetSystemClock());
  //OPENDEBUG();
  OpenUART2(UART_EN | UART_NO_PAR_8BIT | UART_1STOPBIT, UART_RX_ENABLE | UART_TX_ENABLE,
            (pbFreq/16/BAUDRATE) - 1);
  DelayInit(pbFreq);
  putsUART2(CLEAR_VT);
  p_imu = &imu;
  imu_res = ImuInit(p_imu,
      TEST_I2C_BUS_ID,
      pbFreq,
      TEST_I2C_BUS_SPEED,
      ACCEL_RANGE_4G,
      ACCEL_BW_100,
      GYRO_DLPF_LPF_42HZ,
      9,
      GYRO_PWR_MGM_CLK_SEL_X);

  if (imu_res == IMU_FAIL) {
      printf("IMU fail\n");
      while(1);
  }

  Kalman_MahonyInit(&kmah);
  Kalman_MadgwickInit(&kmad);
  printf("PB speed = %u\n", pbFreq);
  printf("Kalmans initialized\n");
  DelayS(2);
  ImuCalibrate(p_imu, TRUE, TRUE, 128, TEST_UPDATE_FREQ);
  printf("Finished IMU calibration.\nBeginning acquisition and filtering...\n");
  DelayS(2);
  printf("Clearing VT in 2, then wait 5 for output\n");
  DelayS(2);
  putsUART2(CLEAR_VT);
  DelayS(5);
  t5 = DelayUtilGetUs();
  while(1) {
    DelayMs(TEST_UPDATE_FREQ);
#ifndef FILE_SAVE_TEST
    putsUART2(CLEAR_VT);
#endif
    t1 = DelayUtilGetUs();
    imu_res = ImuUpdate(p_imu);
    if (imu_res == IMU_FAIL) {
        printf("IMU update failure\n");
        while(1);
    }
    t2 = DelayUtilGetUs();
    Kalman_MadgwickUpdate(p_imu, &kmad, 1.0f / (float) TEST_UPDATE_FREQ * 1000.0f);
    t3 = DelayUtilGetUs();
    Kalman_MahonyUpdate(p_imu, &kmah, 1.0f / (float) TEST_UPDATE_FREQ * 1000.0f);
    t4 = DelayUtilGetUs();
#ifndef FILE_SAVE_TEST
    printf("Update rate = %d ms\n", TEST_UPDATE_FREQ);
    printf("Times follow (in us):\n");
    printf("IMU Update:  %u\n", DelayUtilElapsedUs(t2,t1));
    printf("Madg Update: %u\n", DelayUtilElapsedUs(t3,t2));
    printf("Mah Update:  %u\n\n", DelayUtilElapsedUs(t4,t3));

    printf("Ax = %7.3f, Ay = %7.3f, Az = %7.3f\n", ImuGetAccelX(p_imu), ImuGetAccelY(p_imu), ImuGetAccelZ(p_imu));
    printf("Gx = %7.3f, Gy = %7.3f, Gz = %7.3f, Gt = %7.3f\n\n", ImuGetGyroX(p_imu), ImuGetGyroY(p_imu), ImuGetGyroZ(p_imu), ImuGetGyroTemp(p_imu));
    q = &(kmad.q);
    printf("\nMadg variables: q0=%6.2f q1=%6.2f q2=%6.2f q3=%6.2f\n", q->q0, q->q1, q->q2, q->q3);
    MHelpers_QuaternionToEuler(q, &e);
    printf("In Euler      : psi=%6.2f theta=%6.2f phi=%6.2f\n", e.psi, e.theta, e.phi);
    q = &(kmah.q);
    printf("\nMah  variables: q0=%6.2f q1=%6.2f q2=%6.2f q3=%6.2f\n", q->q0, q->q1, q->q2, q->q3);
    MHelpers_QuaternionToEuler(q, &e);
    printf("In Euler      : psi=%6.2f theta=%6.2f phi=%6.2f\n", e.psi, e.theta, e.phi);
#else
    q = &(kmad.q);
    printf("%f,%f,%f,%f\n", q->q0, q->q1, q->q2, q->q3);
    if (i++ == FILE_SAVE_LINES) {
      break;
    }

#endif

  }

  // Never exit
  while(1);
  return 0;
}