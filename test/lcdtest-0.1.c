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
    DelayS(10);    // delay to allow for stability of LCd
    // LcdInitPins
    // LcdSetEntryMode
    // LcdSetDisplayMode
    // LcdSetShiftCursorOrDisplay
    // LcdSetFunction
    // while(1) {
    // LcdDisplayData
    //}

    return 0;
}