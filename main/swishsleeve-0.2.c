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