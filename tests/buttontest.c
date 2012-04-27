#include <plib.h>
#include <p32xxxx.h>
#include "delay.h"
#include <stdlib.h>

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

#define BUTTONA_PORT IOPORT_A
#define BUTTONA_PIN BIT_1
#define BUTTONB_PORT IOPORT_A
#define BUTTONB_PIN BIT_5

#define BAUDRATE 57600
#define CLEAR_VT "\033[2J"

volatile int cnt = 0;

void ButtonConfig();
BOOL ButtonCheck();


/**************************************************************************************************
  Title: 
    Button Test
    
  Version: 
    0.1
    
  Filename: 
    buttontest.c
    
  Author(s): 
    mkobit
    
  Purpose of Program: 
    Figure out protocol for the button pressing to only react on button press and then liftup
    
  How to build: 
    delay.c
    
  Update History: 
    4/26/12: Figured out which values were static and which variables should be determined each button check
    
**************************************************/
int main() {
  unsigned int pbFreq;
  us_t start;
  
  pbFreq = SYSTEMConfigPerformance(GetSystemClock());
  OpenUART2(UART_EN | UART_NO_PAR_8BIT | UART_1STOPBIT, UART_RX_ENABLE | UART_TX_ENABLE,
            (pbFreq/16/BAUDRATE) - 1);
  DelayInit(pbFreq);
  putsUART2(CLEAR_VT);
  printf("Starting button test\n");
  DelayS(1);
  printf("Go!\n");
  ButtonConfig();
  start = DelayUtilGetUs();
  while(1) {
    DelayMs(20);
    if (ButtonCheck()) {
      printf("Button released at %u, %d\n", DelayUtilElapsedUs(DelayUtilGetUs(), start), cnt);
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