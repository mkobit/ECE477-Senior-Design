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
#define BAUDRATE 57600

#define CLEAR_VT "\033[2J"
#define NEW_LINE_MODE "\033[20h"

#ifndef int16_t
typedef short int int16_t;
#endif

void OPENDEBUG();

int main() {
  int pbFreq;
  long int delayed = 0;
  BOOL result = TRUE;
  BOOL init_res;
  unsigned char data = 0xFF;
  int16_t ax,ay,az;
  int16_t gx,gy,gz,gt;

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
      printf("Error in result: could not read accel ID\n");
      DelayUs(500);
  } else {
      printf("Result: accel ID = 0x%x\n", (unsigned char) data);
  }
  // gyroscope ID
  result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x00, &data);
  if (!result) {
      printf("Error in result: could not read gyro ID\n");
      DelayUs(500);
  } else {
      printf("Result: gyro ID = 0x%x\n", (unsigned char) data);
  }
  // accelerometer default baud
  result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x2C, &data);
  if (!result) {
      printf("Error in result: could not read default accel baud\n");
      DelayUs(500);
  } else {
      printf("Result: accel default buad = 0x%x\n", (unsigned char) data);

  }
  printf("Testing writing\n");
  // write to config register and say if success or not
  // accel configs
  result = I2CShared_WriteByte(TEST_I2C_BUS_ID, 0xA6, 0x2D, (1 << 3));  // put accel in measure mode
  if (!result) {
      printf("Could not write to accel and put it in measure mode\n");
      DelayUs(500);
  } else {
      printf("Success: put accel in measure mode\n");
  }
  // check measure register
  result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x31, &data);
  if (!result) {
      printf("Could not read accel ACCEL_DATA_FORMAT\n");
      DelayUs(500);
  } else {
      printf("Success: read ACCEL_DATA_FORMAT, value = 0x%x\n", (unsigned char) data);
  }
  result = I2CShared_WriteByte(TEST_I2C_BUS_ID, 0xA6, 0x31, 0x01);  // put accel in 4g mode
  if (!result) {
      printf("Could not write to accel and put it in 4g mode\n");
      DelayUs(500);
  } else {
      printf("Success: put accel in 4g mode\n");
  }
  result = I2CShared_WriteByte(TEST_I2C_BUS_ID, 0xA6, 0x2C, 0x0B);  // set bandwidth to 100 hz
  if (!result) {
      printf("Could not write to accel and set accel bandwidth\n");
      DelayUs(500);
  } else {
      printf("Success: set accel bandwidth to 100 Hz\n");
  }
  // gyro configs
  result = I2CShared_WriteByte(TEST_I2C_BUS_ID, 0xD0, 0x16, 0x18 | 3);  // turn gyro on, let lpf
  if (!result) {
      printf("Could not write to gyro and set its LPF\n");
      DelayUs(500);
  } else {
      printf("Success: set gyro LPF to 42 Hz, and 2000 degrees/s\n");
  }
  result = I2CShared_WriteByte(TEST_I2C_BUS_ID, 0xD0, 0x15, 9);  // turn gyro on, let lpf
  if (!result) {
      printf("Could not write to gyro and set its sample rate\n");
      DelayUs(500);
  } else {
      printf("Success: set gyro sample rate to 1kHz/(9+1) == 100 Hz/s\n");
  }
  result = I2CShared_WriteByte(TEST_I2C_BUS_ID, 0xD0, 0x3E, 1);  // turn gyro on, let lpf
  if (!result) {
      printf("Could not write to gyro and set GYRO_PWR_MGM X-axis (roll)\n");
      DelayUs(500);
  } else {
      printf("Success: set gyro GYRO_PWR_MGM X-axis (roll)\n");
  }
  // end configs, read some data and test ReadMultBytes
  printf("Done reading device ID and setting configs, time to read...\n");
  DelayS(4);
  do {
      ax = 0; ay = 0; az = 0; gx = 0; gy = 0; gz = 0; gt = 0;
      DelayMs(200);
      putsUART2(CLEAR_VT);
      // clear terminal before reading and updating
      // accel
      I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x33, &data);   // X MSB
      ax = ax | (data << 8);
      I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x32, &data);   // X LSB
      ax = ax | data;
      I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x35, &data);   // Y MSB
      ay = ay | (data << 8);
      I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x34, &data);   // Y LSB
      ay = ay | data;
      I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x37, &data);   // Z MSB
      az = az | (data << 8);
      I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x36, &data);   // Z LSB
      az = az | data;
      // gyro
      I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x1B, &data);   // T MSB
      gx = gx | (data << 8);
      I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x1C, &data);   // T LSB
      gx = gx | data;
      I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x1D, &data);   // X MSB
      gx = gx | (data << 8);
      I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x1E, &data);   // X LSB
      gx = gx | data;
      I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x1F, &data);   // Y MSB
      gy = gy | (data << 8);
      I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x20, &data);   // Y LSB
      gy = gy | data;
      I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x21, &data);   // Z MSB
      gz = gz | (data << 8);
      I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x22, &data);   // Z LSB
      gz = gz | data;
      // print results
      printf("ax = %5d, ay = %5d, az = %5d\n", ax,ay,az);
      printf("gx = %5d, gy = %5d, gz = %5d, gt = %5d\n", gx, gy, gz, gt);
      delayed++;
  } while(delayed < 300);

  while(1) {}
  return 0;
}

void OPENDEBUG() {
  UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART2, GetPeripheralClock(), BAUDRATE);
  UARTEnable(UART2, UART_ENABLE | UART_TX_ENABLE);
}