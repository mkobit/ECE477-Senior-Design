#include <p32xxxx.h>
#include <plib.h>
#include "lcd_16x2.h"
#include "delay.h"

#define SYSTEM_FREQUENCY 72000000L
int main(void)
{
    int pbFreq;

    pbFreq = SYSTEMConfigPerformance(SYSTEM_FREQUENCY);
    DelayInit(SYSTEM_FREQUENCY);
    DelayS(5);    // delay to allow for stability of LCd
    LcdInitPins(IOPORT_F, BIT_2,
                IOPORT_F, BIT_3,
                IOPORT_G, BIT_2,
                IOPORT_G, BIT_3,
                IOPORT_B, BIT_1,
                IOPORT_B, BIT_4,
                IOPORT_E, BIT_8,
                IOPORT_D, BIT_15,
                IOPORT_F, BIT_0,
                IOPORT_F, BIT_1,
                IOPORT_F, BIT_4);   // Using a custom header in lab with these ports connected
    LcdSetFunction(LCD_INTERFACE_LENGTH_8BIT, LCD_LINES_2, LCD_DOTS_5x8);
    LcdSetEntryMode(LCD_DDRAM_ADDRESS_INCR, LCD_SHIFT_DISPLAY_OFF);
    LcdSetDisplayMode(LCD_DISPLAY_ON, LCD_CURSOR_ON, LCD_CURSOR_BLINK_ON);
    
    while(1) {
        //LcdDisplayData("Hello\nThere!");
        DelayS(5);
        //LcdDisplayData("Hi again!\n Testing LCD, runoff here");
        DelayS(5);
    }

    return 0;
}