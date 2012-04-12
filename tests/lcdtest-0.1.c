#include <p32xxxx.h>
#include <plib.h>
#include "lcd_16x2.h"
#include "delay.h"

#pragma config FNOSC = PRIPLL	// Primary Osc w/PLL (XT+,HS+,EC+PLL)
#pragma config POSCMOD = HS		// HS osc mode 
#pragma config FPLLMUL = MUL_18	// PLL Multiplier
#pragma config FPLLIDIV = DIV_2	// PLL Input Divider
#pragma config FPBDIV = DIV_2
#pragma config FPLLODIV = DIV_1

#define SYSTEM_FREQUENCY 72000000L
int main(void)
{
  int pbFreq;

  pbFreq = SYSTEMConfigPerformance(SYSTEM_FREQUENCY);
  DelayInit(SYSTEM_FREQUENCY);
  DelayS(2);  // delay to allow for stability of LCD
  LcdInitPins(BIT_2, IOPORT_F,
        BIT_3, IOPORT_F,
        BIT_5, IOPORT_F,
        BIT_3, IOPORT_G,
        BIT_1, IOPORT_B,
        BIT_4, IOPORT_B,
        BIT_8, IOPORT_E,
        BIT_15, IOPORT_D,
        BIT_0, IOPORT_F,
        BIT_1, IOPORT_F,
        BIT_4, IOPORT_F);   // Using a custom header in lab with these ports connected
  LcdSetFunction(LCD_INTERFACE_LENGTH_8BIT, LCD_LINES_2, LCD_DOTS_5x8);
  LcdSetDisplayMode(LCD_DISPLAY_OFF, LCD_CURSOR_OFF, LCD_CURSOR_BLINK_OFF);
  LcdClearDisplay();
  LcdSetEntryMode(LCD_DDRAM_ADDRESS_INCR, LCD_SHIFT_DISPLAY_OFF);
  LcdSetDisplayMode(LCD_DISPLAY_ON, LCD_CURSOR_ON, LCD_CURSOR_BLINK_ON);
  
  
  while(1) {
    LcdDisplayData("Hello\nThere!");
    DelayS(3);
    LcdReturnHome();
    LcdDisplayData("Hi again!\n Testing LCD");
    LcdReturnHome();
    DelayS(3);
  }

  return 0;
}