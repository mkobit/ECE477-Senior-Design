#include <plib.h>
#include <p32xxxx.h>

#include "configs.h"

#pragma config DEBUG = ON
#pragma config ICESEL = ICS_PGx1

#define SYS_CLOCK (80000000L)
#define GetSystemClock()            (SYS_CLOCK)
#define GetInstructionClock()       (SYS_CLOCK)

#define BUTTONA_PORT IOPORT_G
#define BUTTONA_PIN BIT_3
#define BUTTONB_PORT IOPORT_G
#define BUTTONB_PIN BIT_2

#define TIM1_SETTINGS (T1_ON | T1_SOURCE_INT | T1_PS_1_256)
#define TIM1_PERIOD 3124  // Done by hand and tested for 10 ms, SYS_CLOCK / PBFREQ / TIM1_PS / 1000

#define TEST_DIGITAL_TOGGLE_PORT IOPORT_D
#define TEST_DIGITAL_TOGGLE_PIN BIT_1

#define SetTimer1Intrs(on) mT1IntEnable(on) // simple macro expansion to the mT1IntEnable, {on} should be TRUE or FALSE

volatile int cnt = 0;

volatile BOOL UPDATE = FALSE;

void ButtonConfig();
BOOL ButtonCheck();

void ConfigTimer1Intrs();

/**************************************************************************************************
  Title: 
    Button Test
    
  Version: 
    0.21
    
  Filename: 
    buttontest-0.21.c
    
  Author(s): 
    mkobit
    
  Purpose of Program: 
    Test button protocol with timer interrupts on our PCB with a
    simple digital out toggle instead of printf. Timer interrupt settings are
    taken from timer1inttest.c. Testing with PCB configurations, and in the style of the main application
    
  How to build: 
    delay.c
    
  Update History: 
    
    
**************************************************/
int main() {
  unsigned int pbFreq;
  
  pbFreq = SYSTEMConfigPerformance(GetSystemClock());
  ButtonConfig();

  PORTSetPinsDigitalOut(TEST_DIGITAL_TOGGLE_PORT, TEST_DIGITAL_TOGGLE_PIN);
  PORTClearBits(TEST_DIGITAL_TOGGLE_PORT, TEST_DIGITAL_TOGGLE_PIN);

  INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

  

  // Enable interrupts
  INTEnableSystemMultiVectoredInt();
  // Enable Timer1 interrupts
  
  // Set timer for interrupts based on how often IMUUpdate and KalmanUpdate occur
  ConfigTimer1Intrs();
  SetTimer1Intrs(TRUE);

  while(1) {
    while (!UPDATE);
    
    UPDATE = FALSE;
    if (ButtonCheck()) {
      PORTToggleBits(TEST_DIGITAL_TOGGLE_PORT, TEST_DIGITAL_TOGGLE_PIN);
    }
  }
  
  return 0;
}

void ButtonConfig() {
  PORTSetPinsDigitalIn(BUTTONA_PORT, BUTTONA_PIN);
  PORTSetPinsDigitalIn(BUTTONB_PORT, BUTTONB_PIN);
}

BOOL ButtonCheck() {
  static BOOL down = FALSE;
  BOOL b1, b2;
  BOOL res = FALSE;;
  
  b1 = PORTReadBits(BUTTONA_PORT, BUTTONA_PIN) ? TRUE : FALSE;
  b2 = PORTReadBits(BUTTONB_PORT, BUTTONB_PIN) ? TRUE : FALSE;

  if (down && (b1 && !b2)) {
    // button relased
    cnt++;  // debug
    res = TRUE;
  }
  down = !b1 && b2;

  return (res);
}

void ConfigTimer1Intrs() {
  // Configure timers for our target update rate
  ConfigIntTimer1(T1_INT_OFF | T1_INT_PRIOR_1);
  OpenTimer1(TIM1_SETTINGS, TIM1_PERIOD);
  WriteTimer1(0);
}

void __ISR(_TIMER_1_VECTOR, ipl1) Timer1IntrHandler() {
  // Clear interrupt flag
  mT1ClearIntFlag();
  // Handle timer 1 interrupt
  UPDATE = TRUE;
}