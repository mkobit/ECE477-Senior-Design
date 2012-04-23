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
#pragma config FPBDIV = DIV_2

// Clock Constants
#define SYS_CLOCK (80000000L)
#define GetSystemClock()            (SYS_CLOCK)
#define GetPeripheralClock()        (SYS_CLOCK/2)
#define GetInstructionClock()       (SYS_CLOCK)

#define TEST_I2C_BUS_ID              I2C1
#define TEST_I2C_BUS_SPEED           (100000)
#define BAUDRATE 57600

#define CLEAR_VT "\033[2J"
#define NEW_LINE_MODE "\033[20h"

void OPENDEBUG();

int main() {
  long int pbFreq;
  long int delayed = 0;
  imu_t imu;
  imu_t *p_imu;
  IMU_RESULT imu_res = IMU_SUCCESS;
  BOOL result;
  BOOL init_res;
  char data;
  // initialize debug messages
  
  pbFreq = SYSTEMConfigPerformance(GetSystemClock());
  //OPENDEBUG();
  OpenUART2(UART_EN | UART_NO_PAR_8BIT | UART_1STOPBIT, UART_RX_ENABLE | UART_TX_ENABLE,
            (pbFreq/16/BAUDRATE) - 1);
  
  DelayInit(GetSystemClock());
  putsUART2(CLEAR_VT);
  //DBINIT();
  p_imu = &imu;
  //UARTSendData(UART2, CLEAR_VT);
  imu_res = ImuInit(p_imu,
          TEST_I2C_BUS_ID,
          GetPeripheralClock(),
          TEST_I2C_BUS_SPEED,
          ACCEL_RANGE_4G,
          ACCEL_BW_100,
          GYRO_DLPF_LPF_98HZ,
          9,
          GYRO_PWR_MGM_CLK_SEL_X);
  init_res = I2CShared_Init(TEST_I2C_BUS_ID, pbFreq, TEST_I2C_BUS_SPEED);
  if (imu_res == IMU_FAIL) {
    printf("Fail\n");
    while(1) {}
  }
  printf("Success\n");
  while(1) {
      ImuUpdate(p_imu);
      putsUART2(CLEAR_VT);
      printf("Ax = %7.3f, Ay = %7.3f, Az = %7.3f\n", ImuGetAccelX(p_imu), ImuGetAccelY(p_imu), ImuGetAccelZ(p_imu));
      printf("Gx = %7.3f, Gy = %7.3f, Gz = %7.3f, Gt = %7.3f\n", ImuGetGyroX(p_imu), ImuGetGyroY(p_imu), ImuGetGyroZ(p_imu), ImuGetGyroTemp(p_imu));
      DelayMs(200);
  }
  while(1);
  return 0;
}

void OPENDEBUG() {
  UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART2, GetPeripheralClock(), BAUDRATE);
  UARTEnable(UART2, UART_ENABLE | UART_TX_ENABLE);
}