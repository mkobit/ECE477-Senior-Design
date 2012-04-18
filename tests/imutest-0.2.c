#include <plib.h>
#include <p32xxxx.h>
#include <sys/appio.h>
#include <stdio.h>
#include "imu.h"
#include "delay.h"


// Configuration Bit settings
//
// SYSCLK = 80 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 40 MHz
// WDT OFF

#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer 
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select

// Clock Constants
#define SYS_CLOCK (80000000L)
#define GetSystemClock()            (SYS_CLOCK)
#define GetPeripheralClock()        (SYS_CLOCK/2)
#define GetInstructionClock()       (SYS_CLOCK)

#define TEST_I2C_BUS_ID              I2C1
#define TEST_I2C_BUS_SPEED           (400000)

#define CLEAR_VT "\033[2J"
#define NEW_LINE_MODE "\033[20h"

int main() {
	long int pbFreq;
	long int delayed = 0;
  imu_t imu;
  IMU_RESULT imu_res;

  // initialize debug messages
  
  pbFreq = SYSTEMConfigPerformance(GetSystemClock());
  //DBINIT();
  printf("Yo\n");
  imu_res = ImuInit(&imu, 
          TEST_I2C_BUS_ID,
          pbFreq,
          TEST_I2C_BUS_SPEED,
          ACCEL_SCALE_4G,
          ACCEL_BW_100,
          GYRO_DLPF_LPF_98HZ,
          9,
          GYRO_PWR_MGM_CLK_SEL_X);
  if (imu_res == IMU_FAIL) {
    while(1) {}
  }
  while(1);
  return 0;
}