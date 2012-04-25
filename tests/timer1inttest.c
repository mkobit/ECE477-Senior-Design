#include <plib.h>
#include <p32xxxx.h>

#include <stdio.h>
#include "delay.h"

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


#define TIM1_SETTINGS (T1_ON | T1_SOURCE_INT | T1_PS_1_256)
#define TIM1_PERIOD 3124  // Done by hand and tested for 10 ms, SYS_CLOCK / PBFREQ / TIM1_PS / 1000

// Timer interrupt functions
void ConfigTimer1Intrs();
void SetTimer1Intrs(BOOL on);
#define BAUDRATE 57600

#define CLEAR_VT "\033[2J"

volatile us_t start;

int main() {
  unsigned int pbFreq;
  
  pbFreq = SYSTEMConfigPerformance(GetSystemClock());

  OpenUART2(UART_EN | UART_NO_PAR_8BIT | UART_1STOPBIT, UART_RX_ENABLE | UART_TX_ENABLE,
            (pbFreq/16/BAUDRATE) - 1);

  putsUART2(CLEAR_VT);
  printf("Starting timer1inttest.c\n\n");
  // Initialize Delay module first
  DelayInit(pbFreq);
  DelayS(2);

  // Configure interrupts for the system as single vector
  INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

  // Set timer for interrupts based on how often IMUUpdate and KalmanUpdate occur
  ConfigTimer1Intrs();

  // Enable interrupts
  INTEnableSystemMultiVectoredInt();
  // Enable Timer1 interrupts
  start = DelayUtilGetUs();
  SetTimer1Intrs(TRUE);

  while(1);
  return 0;
}

void ConfigTimer1Intrs() {
  // Configure timers for our target update rate
  ConfigIntTimer1(T1_INT_OFF | T1_INT_PRIOR_1);
  OpenTimer1(TIM1_SETTINGS, TIM1_PERIOD);
}

// TODO
void SetTimer1Intrs(BOOL on) {
  // Turn timer intrs on or off
  mT1IntEnable(on);
}

// TODO
void __ISR(_TIMER_1_VECTOR, ipl1) Timer1IntrHandler() {
  // Clear interrupt flag
  static unsigned int i = 0;
  static us_t this, last = 0;
  mT1ClearIntFlag();
  // Handle timer 1 interrupt
  this = DelayUtilGetUs();
  printf("Last handler: %u us passed, total time = %u\n", DelayUtilElapsedUs(this, last), DelayUtilElapsedUs(this, start));
  last = this;
  i++;
  if (i == 200) {
    SetTimer1Intrs(FALSE);
    printf("\nTurned off intrs\n");
  }
}
