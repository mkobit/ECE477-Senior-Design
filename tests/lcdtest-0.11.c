#include <p32xxxx.h>
#include <plib.h>
#include "lcd_16x2.h"
#include "delay.h"

#include "configs.h"

#pragma config DEBUG = ON
#pragma config ICESEL = ICS_PGx1


#define SYSTEM_FREQUENCY 80000000L
#define LCDS_IN 11


typedef struct TEST_PAIR {
    unsigned int bitnum;
    IoPortId port_id;
} TEST_PAIR;

// pins and ports to be used for testing
// rs, rw,  en, d0-d7
TEST_PAIR lcd_pairs[LCDS_IN] = {
        {BIT_1, IOPORT_D}, \
        {BIT_5, IOPORT_D}, \
        {BIT_6, IOPORT_D}, \
        {BIT_7, IOPORT_D}, \
        {BIT_0, IOPORT_F}, \
        {BIT_1, IOPORT_F}, \
        {BIT_0, IOPORT_E}, \
        {BIT_1, IOPORT_E}, \
        {BIT_2, IOPORT_E}, \
        {BIT_3, IOPORT_E}, \
        {BIT_4, IOPORT_E}};


/**************************************************************************************************
  Title: 
    LCD test
    
  Version: 
    0.11
    
  Filename: 
    lcdtest-0.11.c
    
  Author(s): 
    mkobit
    ahong
    
  Purpose of Program: 
    Test basic initialization and displaying of data with the LCD on the PCB
    
  How to build: 
    delay.c
    lcd_16x2.c - delay.c
    
  Update History: 
    
    
**************************************************/
int main(void)
{
  int pbFreq;

  pbFreq = SYSTEMConfigPerformance(SYSTEM_FREQUENCY);
  DelayInit(SYSTEM_FREQUENCY);
  DelayMs(1000);  // simple delay to get ready for program
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
        LCD_DOTS_5x8);   // Using a wide selection of pins in lab with these ports connected
  // this test will just test writing to the LCD
  LcdInstrSetDisplayMode(LCD_DISPLAY_ON, LCD_CURSOR_OFF, LCD_CURSOR_BLINK_OFF);
  
  while(1) {
    LcdInstrReturnHome();
    LcdDisplayData("Hello\nThere!");

    DelayMs(5000);
    LcdInstrClearDisplay();
    DelayS(5);
    LcdDisplayData("Just cleared it.\nYou see that?");
    DelayS(5);
    LcdInstrClearDisplay();
    LcdDisplayData("Cleared it again but now to test a long string to check lines");
    DelayS(5);
    LcdInstrClearDisplay();
    LcdDisplayData("Return H");
    LcdInstrSetDDRAMAddress(14);
    LcdDisplayData("END!");
    DelayS(5);
  }

  return 0;
}