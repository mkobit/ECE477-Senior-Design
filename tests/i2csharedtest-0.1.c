#include <plib.h>
#include <p32xxxx.h>
//#include <stdio.h>
#include "delay.h"
#include "i2c_shared.h"
//#include "i2c_shared.h"
//#include "lcd_16x2.h"

#pragma config FPLLMUL = MUL_20
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLODIV = DIV_1
#pragma config POSCMOD = HS
#pragma config FNOSC = PRIPLL
#pragma config FWDTEN = OFF // watchdog off
#pragma config FPBDIV = DIV_2

#define SYS_CLOCK (80000000L)
#define GetSystemClock()            (SYS_CLOCK)
#define GetPeripheralClock()        (SYS_CLOCK/2)
#define GetInstructionClock()       (SYS_CLOCK)

#define TEST_I2C_BUS_ID              I2C1
#define TEST_I2C_BUS_SPEED           (100000)
#define BAUDRATE 9600

#define CLEAR_VT "\033[2J"
#define NEW_LINE_MODE "\033[20h"

void OPENDEBUG();

int main() {
  int pbFreq;
  long int delayed = 0;
  BOOL result = TRUE;
  BOOL init_res;
  char data = 0xFF;

  SYSTEMConfig(GetSystemClock(), SYS_CFG_ALL);
  pbFreq = SYSTEMConfigPerformance(GetSystemClock());
  
  //OPENDEBUG();
  OpenUART2(UART_EN | UART_NO_PAR_8BIT | UART_1STOPBIT, UART_RX_ENABLE | UART_TX_ENABLE,
            (pbFreq/16/BAUDRATE) - 1);

  DelayInit(GetSystemClock());
  ///UARTSendData(UART2, CLEAR_VT);
  putsUART2(CLEAR_VT);
  init_res = I2CShared_Init(TEST_I2C_BUS_ID, GetPeripheralClock(), TEST_I2C_BUS_SPEED);
  
  printf("Starting -> init_res = %d\n", (int) init_res);
  // accelerometer ID
  result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x00, &data);
  if (!result) {
      printf("Error in result: Delay to see if it fixes bus\n");
      DelayUs(500);
  } else {
      printf("Result: accel ID = 0x%x\n", (unsigned char) data);
  }
  // gyroscope ID
  result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x00, &data);
  if (!result) {
      printf("Error in result: Delay to see if it fixes bus\n");
      DelayUs(500);
  } else {
      printf("Result: gyro ID = 0x%x\n", (unsigned char) data);
  }
  // accelerometer default baud
  result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x2C, &data);
  if (!result) {
      printf("Error in result: Delay to see if it fixes bus\n");
      DelayUs(500);
  } else {
      printf("Result: accel default buad = 0x%x\n", (unsigned char) data);

  }
  printf("Testing writing\n");
  // write to config register and say if success or not
  // accel configs
  result = I2CShared_WriteByte(TEST_I2C_BUS_ID, 0xA6, 0x2C, (1 << 3));  // put accel in measure mode
  if (!result) {
      printf("Could not write to accel and put it in measure mode\n");
      DelayUs(500);
  } else {
      printf("Success: put accel in measure mode\n");
  }
  result = I2CShared_WriteByte(TEST_I2C_BUS_ID, 0xA6, 0x31, 0x01);  // put accel in 4g mode
  if (!result) {
      printf("Could not write to accel and put it in measure mode\n");
      DelayUs(500);
  } else {
      printf("Success: put accel in 4g mode\n");
  }
  result = I2CShared_WriteByte(TEST_I2C_BUS_ID, 0xA6, 0x2C, 0x0B);  // set bandwidth to 100 hz
  if (!result) {
      printf("Could not write to accel and put it in measure mode\n");
      DelayUs(500);
  } else {
      printf("Success: set accel bandwidth to 100 Hz\n");
  }
  // gyro configs
  DelayS(1);
  while(1) {}
  return 0;
}

void OPENDEBUG() {
  UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART2, GetPeripheralClock(), BAUDRATE);
  UARTEnable(UART2, UART_ENABLE | UART_TX_ENABLE);
}