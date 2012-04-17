#include <p32xxxx.h>
#include <plib.h>
#include "lcd_16x2.h"
#include "delay.h"

#pragma config POSCMOD=XT, FNOSC=PRIPLL 
#pragma config FPLLIDIV=DIV_2, FPLLMUL=MUL_20, FPLLODIV=DIV_1
#pragma config FPBDIV=DIV_2, FWDTEN=OFF, CP=OFF, BWP=OFF

#define SYSTEM_FREQUENCY 80000000L
int main(void)
{
  int pbFreq;
  
  pbFreq = SYSTEMConfigPerformance(SYSTEM_FREQUENCY);
  DelayInit(SYSTEM_FREQUENCY);
  DelayS(1);  // simple delay to get ready for program
  LcdInit(BIT_2, IOPORT_F,
        BIT_3, IOPORT_F,
        BIT_5, IOPORT_F,
        BIT_3, IOPORT_G,
        BIT_1, IOPORT_B,
        BIT_4, IOPORT_B,
        BIT_8, IOPORT_E,
        BIT_15, IOPORT_D,
        BIT_0, IOPORT_F,
        BIT_1, IOPORT_F,
        BIT_4, IOPORT_F,
        LCD_DOTS_5x8);   // Using a wide selection of pins in lab with these ports connected
  // this test will just test writing
  LcdInstrSetDisplayMode(LCD_DISPLAY_ON, LCD_CURSOR_OFF, LCD_CURSOR_BLINK_ON);
  
  while(1) {
  }
  return 0;
}