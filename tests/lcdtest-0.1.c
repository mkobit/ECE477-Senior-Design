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
#pragma config FWDTEN = OFF


#define SYSTEM_FREQUENCY 72000000L
#define LCDS_IN 11


typedef struct TEST_PAIR {
    unsigned int bitnum;
    IoPortId port_id;
} TEST_PAIR;

// pins and ports to be used for testing
TEST_PAIR lcd_pairs[LCDS_IN] = {
        {BIT_2, IOPORT_B}, \
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


/**************************************************************************************************
  Title: 
    LCD test
    
  Version: 
    0.1
    
  Filename: 
    lcdtest-0.1.c
    
  Author(s): 
    mkobit
    
  Purpose of Program: 
    Test basic initialization and displaying of data with the LCD
    
  How to build: 
    delay.c
    lcd_16x2.c
    
  Update History: 
    
    
**************************************************/
int main(void)
{
  int pbFreq;

  pbFreq = SYSTEMConfigPerformance(SYSTEM_FREQUENCY);
  DelayInit(SYSTEM_FREQUENCY);
  DelayS(1);  // simple delay to get ready for program
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
    DelayS(3);
    LcdInstrClearDisplay();
    DelayS(2);
    LcdDisplayData("Just cleared it.\nYou see that?");
    DelayS(2);
    LcdInstrClearDisplay();
    LcdDisplayData("Cleared it again but now to test a long string to check lines");
    DelayS(2);
    LcdInstrClearDisplay();
    LcdDisplayData("Return H");
    LcdInstrSetDDRAMAddress(14);
    LcdDisplayData("END!");
    DelayS(2);
  }

  return 0;
}