#include <plib.h>
#include <p32xxxx.h>
//#include <stdio.h>
#include "delay.h"
//#include "i2c_shared.h"
//#include "lcd_16x2.h"

#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2
#pragma config FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL

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
  int pbFreq;
  long int delayed = 0;
  BOOL result = TRUE;
  BOOL init_res;
  char data = 0xFF;

  pbFreq = SYSTEMConfigPerformance(GetSystemClock());
  
  //OPENDEBUG();
  OpenUART2(UART_EN | UART_NO_PAR_8BIT | UART_1STOPBIT, UART_RX_ENABLE | UART_TX_ENABLE,
            (pbFreq/16/BAUDRATE) - 1);

  DelayInit(GetSystemClock());
  ///UARTSendData(UART2, CLEAR_VT);
  init_res = I2CShared_Init(TEST_I2C_BUS_ID, pbFreq, TEST_I2C_BUS_SPEED);
  putsUART2(CLEAR_VT);
  printf("Starting -> init_res = %d, lowest enum status for i2c I2C_TRANSMITTER_FULL = %d\r\n", (int) init_res, I2C_TRANSMITTER_FULL);
  while(1) {
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA7, 0xA6, 0x00, &data);
      DelayMs(500);
      printf("Result = %d\r\n", (int)result);
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