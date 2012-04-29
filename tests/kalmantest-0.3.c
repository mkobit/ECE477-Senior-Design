#include <plib.h>
#include <p32xxxx.h>
#include <stdio.h>
#include "imu.h"
#include "delay.h"
#include "i2c_shared.h"
#include "kalman.h"
#include "math_helpers.h"
#include "xbee.h"

#define TEST_UPDATE_FREQ 100

#define KALMAN_SELECT 2

typedef struct TRANSMIT_PACKAGE {
  UINT8 n_bytes;
  imu_id id;
  QUATERNION q;
} TRANSMIT_PACKAGE;

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

#define TEST_I2C_BUS_ID1              I2C1
#define TEST_I2C_BUS_ID2              I2C2
#define TEST_I2C_BUS_SPEED           (100000)
#define BAUDRATE 57600

#define CLEAR_VT "\033[2J"
#define NEW_LINE_MODE "\033[20h"

#define ADDR_I2C_ACCEL1 ACCEL_DEFAULT_ADDR
#define ADDR_I2C_ACCEL2 ACCEL_DEFAULT_ADDR
#define ADDR_I2C_GYRO1 GYRO_DEFAULT_ADDR
#define ADDR_I2C_GYRO2 GYRO_DEFAULT_ADDR

#define TEST_XBEE UART1

/**************************************************************************************************
  Title: 
    Kalman Filter Test
    
  Version: 
    0.3
    
  Filename: 
    kalmantest-0.3.c
    
  Author(s): 
    mkobit
    ahong
    
  Purpose of Program: 
    Test the kalman filter with 2 IMUS with transmission of the packets
    
  How to build: 
    delay.c
    imu.c
    i2c_shared.c
    math_helpers.c
    kalman.c
    xbee.c
    
  Update History: 
    4/27/12: Changed IMU library to use a specific address because I2C on Microchip is awful
    *4/29/12: After several attempts at testing I2C, we have given up with this test, for now
    
**************************************************/
int main() {
  imu_t imu1, imu2;
  imu_t *p_imu1, *p_imu2;
  IMU_RESULT imu_res1 = IMU_FAIL, imu_res2 = IMU_FAIL;
  unsigned int pbFreq;
  us_t t1, t2;
  KALMAN_STATE_MADGWICK kmad1, kmad2;
  KALMAN_STATE_MAHONY kmah1, kmah2;
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
  //OPENDEBUG();
  //OpenUART2(UART_EN | UART_NO_PAR_8BIT | UART_1STOPBIT, UART_RX_ENABLE | UART_TX_ENABLE,
            //(pbFreq/16/BAUDRATE) - 1);
  DelayInit(pbFreq);
  //putsUART2(CLEAR_VT);
  
  p_imu1 = &imu1;
  p_imu2 = &imu2;
  imu_res1 = ImuInit(p_imu1,
      TEST_I2C_BUS_ID1,
      pbFreq,
      TEST_I2C_BUS_SPEED,
      ADDR_I2C_ACCEL1, 
      ACCEL_RANGE_2G,
      ACCEL_BW_100,
      ADDR_I2C_GYRO1,
      GYRO_DLPF_LPF_20HZ,
      9,
      GYRO_PWR_MGM_CLK_SEL_X);

  //printf("IMU 1 finished init\n");
  imu_res2 = ImuInit(p_imu2,
      TEST_I2C_BUS_ID2,
      pbFreq,
      TEST_I2C_BUS_SPEED,
      ADDR_I2C_ACCEL2,
      ACCEL_RANGE_2G,
      ACCEL_BW_100,
      ADDR_I2C_GYRO2,
      GYRO_DLPF_LPF_20HZ,
      9,
      GYRO_PWR_MGM_CLK_SEL_X);
  //printf("IMU 2 finished init\n");


  ImuSetID(p_imu1, 0x00);
  ImuSetID(p_imu2, 0x01);

  if (imu_res1 == IMU_FAIL) {
      //printf("IMU1 fail\n");
      while(1);
  }
  if (imu_res2 == IMU_FAIL) {
      //printf("IMU2 fail\n");
      while(1);
  }

  Kalman_MahonyInit(&kmah1);
  Kalman_MadgwickInit(&kmad1);
  Kalman_MahonyInit(&kmah2);
  Kalman_MadgwickInit(&kmad2);
  //printf("PB speed = %u\n", pbFreq);
  //printf("Kalmans initialized\n");
  XBeeConfigure(TEST_XBEE, pbFreq, XBEE_BAUDRATE);
  //printf("XBee UART initialized\n");
  DelayS(2);
  imu_res1 = ImuCalibrate(p_imu1, TRUE, TRUE, 128, TEST_UPDATE_FREQ);
  //printf("Finished IMU1 calibration.\n");
  if (imu_res1 == IMU_FAIL) {
      //printf("IMU1 calibration fail\n");
      while(1);
  }
  imu_res2 = ImuCalibrate(p_imu2, TRUE, TRUE, 128, TEST_UPDATE_FREQ);
  if (imu_res2 == IMU_FAIL) {
      printf("IMU2 calibration fail\n");
      while(1);
  }
  //printf("Update rate = %d ms\n", TEST_UPDATE_FREQ);
  //printf("Size of TRANSMIT PACKAGE: %u\n", sizeof(TRANSMIT_PACKAGE)); // debug
  DelayS(3);
  //printf("Clearing VT in 2, then wait 1 for acquisition and filtering...\n");
  DelayS(2);
  //putsUART2(CLEAR_VT);
  DelayS(1);
  
  while(1) {
    DelayMs(TEST_UPDATE_FREQ);
#ifndef FILE_SAVE_TEST
    //putsUART2(CLEAR_VT);
#endif
    t1 = DelayUtilGetUs();
    imu_res1 = ImuUpdate(p_imu1);
    if (imu_res1 == IMU_FAIL) {
        //printf("IMU1 update failure\n");
        ImuResetI2CBus(p_imu1);
        DelayMs(50);
        continue;
    }
    imu_res2 = ImuUpdate(p_imu2);
    if (imu_res2 == IMU_FAIL) {
        //printf("IMU2 update failure\n");
        ImuResetI2CBus(p_imu2);
        DelayMs(50);
        continue;
    }
    updateRate = 1.0f / (float) TEST_UPDATE_FREQ * 1000.0f;
    //printf("Update time = %u\n", DelayUtilElapsedUs(DelayUtilGetUs(), t1));
#if (KALMAN_SELECT == 1)

    Kalman_MadgwickUpdate(p_imu1, &kmad1, updateRate);
    Kalman_MadgwickUpdate(p_imu2, &kmad2, updateRate);
    //t3 = DelayUtilGetUs();
#elif (KALMAN_SELECT == 2)

    Kalman_MahonyUpdate(p_imu1, &kmah1, updateRate);
    Kalman_MahonyUpdate(p_imu2, &kmah2, updateRate);
#endif

    //printf("Ax = %7.3f, Ay = %7.3f, Az = %7.3f\n", ImuGetAccelX(p_imu), ImuGetAccelY(p_imu), ImuGetAccelZ(p_imu));
    //printf("Gx = %7.3f, Gy = %7.3f, Gz = %7.3f, Gt = %7.3f\n\n", ImuGetGyroX(p_imu), ImuGetGyroY(p_imu), ImuGetGyroZ(p_imu), ImuGetGyroTemp(p_imu));
#if (KALMAN_SELECT == 1)
    q = &(kmad.q);
    //printf("\nMadg variables: q0=%6.2f q1=%6.2f q2=%6.2f q3=%6.2f\n", q->q0, q->q1, q->q2, q->q3);
    MHelpers_QuaternionToEuler(q, &e);
    //printf("In Euler      : psi=%6.2f theta=%6.2f phi=%6.2f\n", e.psi, e.theta, e.phi);
#elif (KALMAN_SELECT == 2)
    q = &(kmah1.q);
    //printf("\n1: Mah  variables: q0=%6.2f q1=%6.2f q2=%6.2f q3=%6.2f\n", q->q0, q->q1, q->q2, q->q3);
    MHelpers_QuaternionToEuler(q, &e);
    //printf("1: In Euler      : psi=%6.2f theta=%6.2f phi=%6.2f\n", e.psi, e.theta, e.phi);
    q = &(kmah2.q);
    //printf("\n2: Mah  variables: q0=%6.2f q1=%6.2f q2=%6.2f q3=%6.2f\n", q->q0, q->q1, q->q2, q->q3);
    MHelpers_QuaternionToEuler(q, &e);
    //printf("2: In Euler      : psi=%6.2f theta=%6.2f phi=%6.2f\n", e.psi, e.theta, e.phi);
#endif

    /*if (reInitKalman++ == 25000) {
      Kalman_MahonyInit(&kmah);
      Kalman_MadgwickInit(&kmad);
      reInitKalman = 0;
      //printf("\nREINITIALIZED\n");
      DelayMs(TEST_UPDATE_FREQ);
    }*/
    // Test xbee sending
    package.id = ImuGetId(p_imu1);
    package.q.q0 = q->q0;
    package.q.q1 = q->q1;
    package.q.q2 = q->q2;
    package.q.q3 = q->q3;
    XBeeSendDataBuffer(TEST_XBEE, (char *) &package, package.n_bytes);
    package.id = ImuGetId(p_imu2);
    package.q.q0 = q->q0;
    package.q.q1 = q->q1;
    package.q.q2 = q->q2;
    package.q.q3 = q->q3;
    XBeeSendDataBuffer(TEST_XBEE, (char *) &package, package.n_bytes);
    //printf("total transfer time = %u\n", DelayUtilElapsedUs(DelayUtilGetUs(), t1));
    t2 = DelayUtilElapsedUs(DelayUtilGetUs(), t1);
  }

  // Never exit
  while(1);
  return 0;
}