#include <p32xxxx.h>
#include <plib.h>
#include <stdio.h>
#include "delay.h"
#include "i2c_shared.h"
#include "lcd_16x2.h"

#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FWDTEN   = OFF           // Watchdog Timer

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
  IMU_RESULT imu_res = IMU_SUCCESS;
  BOOL result;
  BOOL init_res;
  char data = 0xFF;

  pbFreq = SYSTEMConfigPerformance(GetSystemClock());
  OPENDEBUG();

  DelayInit(GetSystemClock());
  ///UARTSendData(UART2, CLEAR_VT);
  init_res = I2CShared_Init(TEST_I2C_BUS_ID, pbFreq, TEST_I2C_BUS_SPEED);
  printf("Starting\n");
  printf("derp\n");
  while(1) {
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA7, 0xA6, 0x00, &data);
      DelayMs(500);
      printf("Hi\n");
  }
  if (imu_res == IMU_FAIL) {
    while(1) {}
  }
  while(1) {}
  return 0;
}

void OPENDEBUG() {
  UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART2, GetPeripheralClock(), BAUDRATE);
  UARTEnable(UART2, UART_ENABLE | UART_TX_ENABLE);
}