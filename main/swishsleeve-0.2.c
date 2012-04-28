#include <plib.h>
#include <p32xxxx.h>
#include <string.h>

#include <stdio.h>

#include "delay.h"
#include "imu.h"
//#include "battery_monitor.h"  DOES NOT WORK CURRENTLY
#include "i2c_shared.h"
#include "kalman.h"
#include "lcd_16x2.h"
#include "math_helpers.h"
#include "xbee.h"

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



/* KALMAN FILTER DEFINES AND CONSTANTS */
// TODO need update rate
#define SAMPLE_FREQ 100.0f  // 1 / 10 ms
#define USE_MADGWICK 1
#define USE_MAHONY 2
#define FILTERING_ALG USE_MADGWICK


/* IMU DEFINES AND CONSTANTS */
// I2C bus frequency to communicate with IMUs
#define I2C_CLOCK 400000
// IMU acceleration and gyroscope settings being used
#define ACC_RANG ACCEL_RANGE_2G         // accelerometer range
#define ACC_BW ACCEL_BW_100             // accelerometer bandwidth
#define GYR_DLPF GYRO_DLPF_LPF_20HZ     // gyro digital low pass filter setting
#define GYR_SAMP_DIV 9                  // sample divider being used
#define GYR_POW_SEL GYRO_PWR_MGM_CLK_SEL_X  // gyro clock selector
// IMU calibration settings
#define CALIBRATE_SAMPLES 128
#define CALIBRATE_DELAY ((UINT)((1 / SAMPLE_FREQ) * 1000))


/* BATTERY MONITOR DEFINES AND CONSTANTS */
// *not currently supported


/* XBEE DEFINES AND CONSTANTS */
// TODO all xbee settings here
#define UART_PORT_XBEE UART1
typedef struct TRANSMIT_PACKAGE {
  UINT8 n_bytes;
  imu_id id;
  QUATERNION q;
} TRANSMIT_PACKAGE;


/* LCD DEFINES AND CONSTANTS */
#define LCD_INPUTS 11
// Settings that are being used
#define CURSOR_VAL LCD_CURSOR_OFF
#define CURSOR_BLINK LCD_CURSOR_BLINK_OFF

#define LCD_DISP_THRESH 1000
// Simple structure for organizing LCD_DATA
typedef struct port_pin_pair {
    unsigned int bitnum;
    IoPortId port_id;
} PP_PAIR;
// LCD pins/ports
// rs, rw,  en, d0-d7
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


/* BUTTON DEFINES AND CONSTANTS */    
// Pins and ports used for button reset FIXME: might need to be changed for diagram/working
#define BUTTON1_A_PORT IOPORT_G
#define BUTTON1_A_PIN BIT_2
#define BUTTON1_B_PORT IOPORT_G
#define BUTTON1_B_PIN BIT_3

/* TIMER1 DEFINES AND CONSTANTS */
// Settings for timer interrupt, 10 ms interrupt
#define TIM1_SETTINGS (T1_ON | T1_SOURCE_INT | T1_PS_1_256)
#define TIM1_PERIOD 3124  // Done by hand and tested for 10 ms, SYS_CLOCK / PBFREQ / TIM1_PS / 1000 / (1 / 10 ms)
volatile BOOL UPDATE = FALSE;     // flag for update that timer will change



/**************************************************************************************************
  Title: 
    Swish Sleeve Main Application
    
  Version: 
    0.2
    
  Filename: 
    swishsleeve-0.2.c
    
  Author(s): 
    mkobit
    
  Purpose of Program: 
    Main program for the Swish Sleeve application
    
  How to build: 
    delay.c
    imu.c
    *battery_monitor.c  *NOT CURRENTLY SUPPORTED
		i2c_shared.c
		kalman.c
		lcd_16x2.c
		math_helpers.c
		xbee.c
    
  Update History: 
    
    
**************************************************/
/*** MAIN SWISH SLEEVE PROGRAM ***/
int main() {
  return 0;
}