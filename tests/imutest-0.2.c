#include <plib.h>
#include <p32xxxx.h>
#include <stdio.h>
#include "imu.h"
#include "delay.h"
#include "i2c_shared.h"


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

#define UPDATE_DELAY 15

#define CLEAR_VT "\033[2J"
#define NEW_LINE_MODE "\033[20h"

void OPENDEBUG(unsigned int pbFreq);


/**************************************************************************************************
  Title: 
    IMUTest
    
  Version: 
    0.2
    
  Filename: 
    imutest-0.2.c
    
  Author(s): 
    mkobit
    
  Purpose of Program: 
    Tests major functionality of initializing, reading  from, writing to, calibrating, and other functionality. Through this, the accelerometer and gyroscope
    were also tested
    
  How to build: 
    delay.c
    imu.c
    i2c_shared.c
    
  Update History: 
    4/26/12: Added calibration
    
**************************************************/
int main() {
  long int pbFreq;
  imu_t imu;
  imu_t *p_imu;
  IMU_RESULT imu_res = IMU_FAIL;
  us_t start, end;
  gyro_t *gyro;
  // initialize debug messages
  
  pbFreq = SYSTEMConfigPerformance(GetSystemClock());
  
  //OPENDEBUG();
  OpenUART2(UART_EN | UART_NO_PAR_8BIT | UART_1STOPBIT, UART_RX_ENABLE | UART_TX_ENABLE,
            (pbFreq/16/BAUDRATE) - 1);
  
  DelayInit(pbFreq);
  putsUART2(CLEAR_VT);
  printf("Max PB speed: %u, what pbfreq is: %d", PB_BUS_MAX_FREQ_HZ, pbFreq);
  DelayS(2);
  //DBINIT();
  p_imu = &imu;
  //UARTSendData(UART2, CLEAR_VT);
  imu_res = ImuInit(p_imu,
          TEST_I2C_BUS_ID,
          pbFreq,
          TEST_I2C_BUS_SPEED,
          ACCEL_RANGE_4G,
          ACCEL_BW_100,
          GYRO_DLPF_LPF_10HZ,
          9,
          GYRO_PWR_MGM_CLK_SEL_X);
  //init_res = I2CShared_Init(TEST_I2C_BUS_ID, pbFreq, TEST_I2C_BUS_SPEED);
  DelayS(2);
  while (imu_res == IMU_FAIL) {
    printf("\nFail, retrying...\n");
    DelayS(3);
    putsUART2(CLEAR_VT);
    printf("Resetting here\n");
    ImuResetI2CBus(p_imu);
    DelayS(2);
    printf("\n");
    imu_res = ImuInit(p_imu,
          TEST_I2C_BUS_ID,
          pbFreq,
          TEST_I2C_BUS_SPEED,
          ACCEL_RANGE_4G,
          ACCEL_BW_100,
          GYRO_DLPF_LPF_10HZ,
          9,
          GYRO_PWR_MGM_CLK_SEL_X);
  }
  printf("\nSuccess: calculating offsets\n");
  ImuCalibrate(p_imu, TRUE, TRUE, 128, UPDATE_DELAY);
  gyro = ImuGetGyro(p_imu);
  printf("ImuCalibration complete: offsets are: %d, %d, %d\n", gyro->xOffset, gyro->yOffset, gyro->zOffset);
  DelayS(3);
  while(1) {
    start = DelayUtilGetUs();
    imu_res = ImuUpdate(p_imu);
    end = DelayUtilGetUs();
    if (imu_res == IMU_FAIL) {
        printf("Error, IMU fail, delaying for 1s and trying to reset I2C bus\n");
        ImuResetI2CBus(p_imu);
        DelayS(2);
        continue;
    }
    putsUART2(CLEAR_VT);
    printf("Ax = %7.3f, Ay = %7.3f, Az = %7.3f\n", ImuGetAccelX(p_imu), ImuGetAccelY(p_imu), ImuGetAccelZ(p_imu));
    printf("Gx = %7.3f, Gy = %7.3f, Gz = %7.3f, Gt = %7.3f\n", ImuGetGyroX(p_imu), ImuGetGyroY(p_imu), ImuGetGyroZ(p_imu), ImuGetGyroTemp(p_imu));
    //printf("Us for update = %d\n", DelayUtilElapsedUs(end, start));
    DelayMs(UPDATE_DELAY);
  }
  while(1);
  return 0;
}

void OPENDEBUG(unsigned int pbFreq) {
  UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART2, pbFreq, BAUDRATE);
  UARTEnable(UART2, UART_ENABLE | UART_TX_ENABLE);
}