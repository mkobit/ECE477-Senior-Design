#include <p32xxxx.h>
#include <plib.h>
#include "lcd_16x2.h"
#include "delay.h"

#pragma config FNOSC = PRIPLL	// Primary Osc w/PLL (XT+,HS+,EC+PLL)
#pragma config POSCMOD = HS		// HS osc mode
#pragma config FPLLMUL = MUL_20	// PLL Multiplier
#pragma config FPLLIDIV = DIV_2	// PLL Input Divider
// 8 MHz * 20 / 2 = 80 MHz
#pragma config FPBDIV = DIV_2
#pragma config FPLLODIV = DIV_1
#pragma config FWDTEN = OFF


#define SYSTEM_FREQUENCY 80000000L
#define LCDS_IN 11

void CrazyCursor(char offset);

typedef struct TEST_PAIR {
    unsigned int bitnum;
    IoPortId port_id;
} TEST_PAIR;

// pins and ports to be used for testing
TEST_PAIR lcd_pairs[LCDS_IN] = {
        {BIT_7, IOPORT_F}, \
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


int main(void)
{
  int i;
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
  // Test shiting and other options
  LcdInstrSetDisplayMode(LCD_DISPLAY_ON, LCD_CURSOR_OFF, LCD_CURSOR_BLINK_ON);
  LcdDisplayData("LEFT SIDE!\non left");
  LcdInstrSetDDRAMAddress(LINE_1 + 21);
  LcdDisplayData("RIGHT SIDE!");
  LcdInstrSetDDRAMAddress(LINE_2 + 24);
  LcdDisplayData("on right");
  DelayS(2);
  LcdInstrReturnHome();
  LcdInstrSetDisplayMode(LCD_DISPLAY_ON, LCD_CURSOR_OFF, LCD_CURSOR_BLINK_OFF);
  
  while(1) {
    for (i = 0; i < 16; i++) {
      LcdInstrShiftCursorOrDisplay(LCD_SHIFT_SELECT_DISPLAY, LCD_SHIFT_DIRECTION_LEFT);
      DelayMs(175);
    }
    // at the right side of the screen
    CrazyCursor(16);
    for (i = 0; i < 16; i++) {
      LcdInstrShiftCursorOrDisplay(LCD_SHIFT_SELECT_DISPLAY, LCD_SHIFT_DIRECTION_RIGHT);
      DelayMs(175);
    }
    // back on left side of screen
    CrazyCursor(0);
  }
  return 0;
}

void CrazyCursor(char offset) {
    int i;
    LcdInstrSetDisplayMode(LCD_DISPLAY_ON, LCD_CURSOR_ON, LCD_CURSOR_BLINK_ON);

    // Send cursor across top and bottom going right 2 times
    LcdInstrSetDDRAMAddress(offset);
    for (i = 0; i < 16; i++) {
        LcdInstrShiftCursorOrDisplay(LCD_SHIFT_SELECT_CURSOR, LCD_SHIFT_DIRECTION_RIGHT);
        DelayMs(95);
    }
    LcdInstrSetDDRAMAddress(LINE_2 + offset);
    for (i = 0; i < 16; i++) {
        LcdInstrShiftCursorOrDisplay(LCD_SHIFT_SELECT_CURSOR, LCD_SHIFT_DIRECTION_RIGHT);
        DelayMs(95);
    }
    LcdInstrSetDDRAMAddress(offset);
    for (i = 0; i < 16; i++) {
        LcdInstrShiftCursorOrDisplay(LCD_SHIFT_SELECT_CURSOR, LCD_SHIFT_DIRECTION_RIGHT);
        DelayMs(95);
    }
    LcdInstrSetDDRAMAddress(LINE_2 + offset);
    for (i = 0; i < 16; i++) {
        LcdInstrShiftCursorOrDisplay(LCD_SHIFT_SELECT_CURSOR, LCD_SHIFT_DIRECTION_RIGHT);
        DelayMs(95);
    }

    // send cursor from bottom to top left 2 times
     LcdInstrSetDDRAMAddress(LINE_2 + offset + 16);
    for (i = 0; i < 16; i++) {
        LcdInstrShiftCursorOrDisplay(LCD_SHIFT_SELECT_CURSOR, LCD_SHIFT_DIRECTION_LEFT);
        DelayMs(95);
    }
    LcdInstrSetDDRAMAddress(offset + 16);
    for (i = 0; i < 16; i++) {
        LcdInstrShiftCursorOrDisplay(LCD_SHIFT_SELECT_CURSOR, LCD_SHIFT_DIRECTION_LEFT);
        DelayMs(95);
    }
    LcdInstrSetDDRAMAddress(LINE_2 + offset + 16);
    for (i = 0; i < 16; i++) {
        LcdInstrShiftCursorOrDisplay(LCD_SHIFT_SELECT_CURSOR, LCD_SHIFT_DIRECTION_LEFT);
        DelayMs(95);
    }
    LcdInstrSetDDRAMAddress(offset + 16);
    for (i = 0; i < 16; i++) {
        LcdInstrShiftCursorOrDisplay(LCD_SHIFT_SELECT_CURSOR, LCD_SHIFT_DIRECTION_LEFT);
        DelayMs(95);
    }
    LcdInstrSetDisplayMode(LCD_DISPLAY_ON, LCD_CURSOR_OFF, LCD_CURSOR_BLINK_OFF);
}