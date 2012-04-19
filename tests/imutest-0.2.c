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

#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider

/*#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select*/

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
  IMU_RESULT imu_res = IMU_SUCCESS;
  BOOL result;
  BOOL init_res;
  char data;
  // initialize debug messages
  
  pbFreq = SYSTEMConfigPerformance(GetSystemClock());
  OPENDEBUG();
  
  DelayInit(GetSystemClock());
  //DBINIT();
  UARTSendData(UART2, CLEAR_VT);
  imu_res = ImuInit(&imu,
          TEST_I2C_BUS_ID,
          pbFreq,
          TEST_I2C_BUS_SPEED,
          ACCEL_SCALE_4G,
          ACCEL_BW_100,
          GYRO_DLPF_LPF_98HZ,
          9,
          GYRO_PWR_MGM_CLK_SEL_X);
  init_res = I2CShared_Init(TEST_I2C_BUS_ID, pbFreq, TEST_I2C_BUS_SPEED);
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