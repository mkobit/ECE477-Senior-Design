#include <plib.h>
#include <p32xxxx.h>
#include <string.h>

#include <stdio.h>


#include "imu.h"
//#include "battery_monitor.h"  DOES NOT WORK CURRENTLY
#include "delay.h"
#include "i2c_shared.h"
#include "kalman.h"
#include "lcd_16x2.h"
#include "math_helpers.h"


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


/*****Constants that are used in initializing and execution of the main program*****/
/* IMU DEFINES AND CONSTANTS */
// IDs and corresponding I2C modules to be used with the IMUs
// These same IDs should be used on the base station
#define ID_UPPER_ARM 0
#define I2C_UPPER_ARM I2C1
#define ID_FORE_ARM 1
#define I2C_FORE_ARM I2C2   // FIXME these i2c are probably wrong from diagram
#define ID_HAND 2
#define I2C_HAND I2C3
// I2C bus frequency to communicate with IMUs
#define I2C_FREQUENCY 400000

/* BATTERY MONITOR DEFINES AND CONSTANTS */
// *not incorporated

/* KALMAN DEFINES AND CONSTANTS */
// TODO need update rate

/* LCD DEFINES AND CONSTANTS */
#define LCD_INPUTS 11

// Settings
#define CURSOR_VAL LCD_CURSOR_OFF
#define CURSOR_BLINK LCD_CURSOR_BLINK_OFF

typedef struct LCD_PAIR {
    unsigned int bitnum;
    IoPortId port_id;
} LCD_PAIR;

// pins and ports to be used for testing
#define INIT_MESSAGE1 "Initializing"
#define INIT_MESSAGE2 "Initializing."
#define INIT_MESSAGE3 "Initializing.."
#define INIT_MESSAGE4 "Initializing..."
#define IMU_ERROR "IMU ERROR"
#define BATT_PRE "Battery: "
#define RSSI_PRE "RSSI: "
#define BATT_INVAL "NO BATT. SIGNAL"
#define RSS_INVAL "NO RSSI. SIGNAL"
LCD_PAIR lcd_pairs[LCD_INPUTS] = {  // TODO fix these for the actual project
    {BIT_7, IOPORT_F}, \
    {BIT_6, IOPORT_F}, \
    {BIT_9, IOPORT_E}, \
    {BIT_14, IOPORT_D}, \
    {BIT_8, IOPORT_E}, \
    {BIT_15, IOPORT_D}, \
    {BIT_2, IOPORT_A}, \
    {BIT_3, IOPORT_A}, \
    {BIT_13, IOPORT_G}, \
    {BIT_14, IOPORT_G}, \
    {BIT_4, IOPORT_C}};

// Pins and ports used for button reset TODO


// Utility functions used in here
BOOL ButtonReset();
void UpdateLCDStatus(int signalPercent, int batteryPercent);

int main() {
  imu_t imu_upper_arm;
  imu_t imu_fore_arm;
  imu_t imu_hand;
  unsigned int pbFreq;  // peripheral bus frequency


  // Initialize components needed
  pbFreq = SYSTEMConfigPerformance(GetSystemClock());

  // Initialize Delay module first
  DelayInit(GetSystemClock());

  // Initialize LCD module
  LcdInit(lcd_pairs[0].bitnum, lcd_pairs[0].port_id,
    lcd_pairs[1].bitnum, lcd_pairs[1].port_id,
    lcd_pairs[2].bitnum, lcd_pairs[2].port_id,
    lcd_pairs[3].bitnum, lcd_pairs[3].port_id,
    lcd_pairs[4].bitnum, lcd_pairs[4].port_id,
    lcd_pairs[5].bitnum, lcd_pairs[5].port_id,
    lcd_pairs[6].bitnum, lcd_pairs[6].port_id,
    lcd_pairs[7].bitnum, lcd_pairs[7].port_id,
    lcd_pairs[8].bitnum, lcd_pairs[8].port_id,
    lcd_pairs[9].bitnum, lcd_pairs[9].port_id,
    lcd_pairs[10].bitnum, lcd_pairs[10].port_id,
    LCD_DOTS_5x8);
  LcdInstrSetDisplayMode(LCD_DISPLAY_ON, CURSOR_VAL, CURSOR_BLINK);
  // Display initializing
  LcdDisplayData(INIT_MESSAGE1);

  // Initialize IMUs
  // Set IMU IDs


  

  return 0;
}

// TODO
BOOL ButtonRest() {
    // TODO
  static BOOL past = 0;
  static BOOL current = 0;

  return FALSE;
}

// DONE doc this
void UpdateLCDStatus(int signalPercent, int batteryPercent) {
  char buffer[17];
  char percentageval[5];

  // Return to home address without instruction
  LcdInstrSetDDRAMAddress(LINE_1);

  // Display signal message
  if (signalPercent <= 0) {
    // invalid signal strength
    LcdDisplayData(RSS_INVAL);
  } else {
    strcpy(buffer, BATT_PRE);
    // Format battery string
    itoa(percentageval, signalPercent, 10);
    strcat(buffer, percentageval);
    strcat(buffer, percentageval);
    strcat(buffer, "%");
    // Display formatted signal string to LCD
    LcdDisplayData(buffer);
  }

  // Move to second line
  LcdInstrSetDDRAMAddress(LINE_2);
  
  // Display battery message
  if (batteryPercent <= 0) {
    LcdDisplayData(RSS_INVAL);
  } else {
    strcpy(buffer, RSSI_PRE);
    // Format battery string
    itoa(percentageval, batteryPercent, 10);
    strcat(buffer, percentageval);
    strcat(buffer, percentageval);
    strcat(buffer, "%");
    // Display formatted battery string to LCD
    LcdDisplayData(buffer);
  }

}