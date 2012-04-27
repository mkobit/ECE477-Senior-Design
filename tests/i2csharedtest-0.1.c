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
#pragma config FPBDIV = DIV_1

#define SYS_CLOCK (80000000L)
#define GetSystemClock()            (SYS_CLOCK)
#define GetInstructionClock()       (SYS_CLOCK)

#define TEST_I2C_BUS_ID              I2C1
#define TEST_I2C_BUS_SPEED           (400000)
#define BAUDRATE 57600

#define CLEAR_VT "\033[2J"
#define NEW_LINE_MODE "\033[20h"

#ifndef int16_t
typedef short int int16_t;
#endif

void OPENDEBUG(unsigned int pbFreq);

/**************************************************************************************************
  Title: 
    I2C Test
    
  Version: 
    mkobit
    
  Filename: 
    i2csharedtest-0.1.c
    
  Author(s): 
    mkobit
    
  Purpose of Program: 
    Test various functions of the I2CShared library that we built. Initializing, writing a byte, and reading a byte. Also tests Resetting if bus failure detected
    
  How to build: 
    delay.c
    i2c_shared.c
    
  Update History: 
    4/23/12: Added testing to reset a bus after timeouts or errors
    
**************************************************/
int main() {
  unsigned int pbFreq;
  long int delayed = 0;
  BOOL result = TRUE;
  BOOL init_res;
  unsigned char data = 0xFF;
  us_t t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14;
  int16_t ax,ay,az;
  int16_t gx,gy,gz,gt;

  SYSTEMConfig(GetSystemClock(), SYS_CFG_ALL);
  pbFreq = SYSTEMConfigPerformance(GetSystemClock());
  
  //OPENDEBUG(pbFreq);
  OpenUART2(UART_EN | UART_NO_PAR_8BIT | UART_1STOPBIT, UART_RX_ENABLE | UART_TX_ENABLE,
            (pbFreq/16/BAUDRATE) - 1);

  DelayInit(pbFreq);
  ///UARTSendData(UART2, CLEAR_VT);

  putsUART2(CLEAR_VT);
  init_res = I2CShared_Init(TEST_I2C_BUS_ID, pbFreq, TEST_I2C_BUS_SPEED);
  
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
      DelayMs(400);
      putsUART2(CLEAR_VT);
      // clear terminal before reading and updating
      // accel
      t1 = DelayUtilGetUs();
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x33, &data);   // X MSB
      t2 = DelayUtilGetUs();
      if (!result) {
        I2CShared_ResetBus(TEST_I2C_BUS_ID);
      }
      ax = ax | (data << 8);
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x32, &data);   // X LSB
      t3 = DelayUtilGetUs();
      if (!result) {
        I2CShared_ResetBus(TEST_I2C_BUS_ID);
      }
      ax = ax | data;
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x35, &data);   // Y MSB
      t4 = DelayUtilGetUs();
      if (!result) {
        I2CShared_ResetBus(TEST_I2C_BUS_ID);
      }
      ay = ay | (data << 8);
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x34, &data);   // Y LSB
      t5 = DelayUtilGetUs();
      if (!result) {
        I2CShared_ResetBus(TEST_I2C_BUS_ID);
      }
      ay = ay | data;
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x37, &data);   // Z MSB
      t6 = DelayUtilGetUs();
      if (!result) {
        I2CShared_ResetBus(TEST_I2C_BUS_ID);
      }
      if (!result) {
        I2CShared_ResetBus(TEST_I2C_BUS_ID);
      }
      az = az | (data << 8);
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xA6, 0xA7, 0x36, &data);   // Z LSB
      t7 = DelayUtilGetUs();
      if (!result) {
        I2CShared_ResetBus(TEST_I2C_BUS_ID);
      }
      az = az | data;
      // gyro
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x1B, &data);   // T MSB
      t8 = DelayUtilGetUs();
      if (!result) {
        I2CShared_ResetBus(TEST_I2C_BUS_ID);
      }
      gx = gx | (data << 8);
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x1C, &data);   // T LSB
      t9 = DelayUtilGetUs();
      if (!result) {
        I2CShared_ResetBus(TEST_I2C_BUS_ID);
      }
      gx = gx | data;
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x1D, &data);   // X MSB
      t10 = DelayUtilGetUs();
      if (!result) {
        I2CShared_ResetBus(TEST_I2C_BUS_ID);
      }
      gx = gx | (data << 8);
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x1E, &data);   // X LSB
      t11 = DelayUtilGetUs();
      if (!result) {
        I2CShared_ResetBus(TEST_I2C_BUS_ID);
      }
      gx = gx | data;
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x1F, &data);   // Y MSB
      t11 = DelayUtilGetUs();
      if (!result) {
        I2CShared_ResetBus(TEST_I2C_BUS_ID);
      }
      gy = gy | (data << 8);
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x20, &data);   // Y LSB
      t12 = DelayUtilGetUs();
      if (!result) {
        I2CShared_ResetBus(TEST_I2C_BUS_ID);
      }
      gy = gy | data;
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x21, &data);   // Z MSB
      t13 = DelayUtilGetUs();
      if (!result) {
        I2CShared_ResetBus(TEST_I2C_BUS_ID);
      }
      gz = gz | (data << 8);
      result = I2CShared_ReadByte(TEST_I2C_BUS_ID, 0xD0, 0xD1, 0x22, &data);   // Z LSB
      t14 = DelayUtilGetUs();
      if (!result) {
        I2CShared_ResetBus(TEST_I2C_BUS_ID);
      }
      gz = gz | data;
      // print results
      printf("ax = %5d, ay = %5d, az = %5d\ntimes per byte = %u, %u, %u, %u, %u, %u\n", ax,ay,az, DelayUtilElapsedUs(t2,t1), \
              DelayUtilElapsedUs(t3,t2), DelayUtilElapsedUs(t4,t3), DelayUtilElapsedUs(t5,t4), DelayUtilElapsedUs(t6,t5), DelayUtilElapsedUs(t7,t6));
      printf("\ngx = %5d, gy = %5d, gz = %5d, gt = %5d\ntimes per byte= %u, %u, %u\n", gx, gy, gz, gt, DelayUtilElapsedUs(t8, t7), \
              DelayUtilElapsedUs(t9,t8), DelayUtilElapsedUs(t10,t9), DelayUtilElapsedUs(t11,t10), DelayUtilElapsedUs(t12,t11), \
              DelayUtilElapsedUs(t13,t12),DelayUtilElapsedUs(t14,t13));
      printf("Total time =%u\n", DelayUtilElapsedUs(t14,t1));
      delayed++;
  } while(delayed < 800);

  while(1) {}
  return 0;
}

void OPENDEBUG(unsigned int pbFreq) {
  UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART2, pbFreq, BAUDRATE);
  UARTEnable(UART2, UART_ENABLE | UART_TX_ENABLE);
}