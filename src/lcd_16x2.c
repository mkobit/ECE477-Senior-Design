#include <p32xxxx.h>
#include <plib.h>
#include "lcd_16x2.h"
#include "delay.h"

static unsigned int _rs, _rw, _en;	// pin numbers
static unsigned int _d0, _d1, _d2, _d3, _d4, _d5, _d6, _d7;		// pin numbers
static IoPortId _rs_port, _rw_port, _en_port;	// port numbers
static IoPortId _d0_port, _d1_port, _d2_port, _d3_port, _d4_port, _d5_port, _d6_port, _d7_port;	// port numbers

static void LcdSetOutputs(int rs, int rw, unsigned char c);
static inline void LcdConfigAllAsOutputs();
static void LcdEnableOn();
static void LcdEnableOff();
static void LcdSetDataOutputs(unsigned char data);

void LcdInitPins(unsigned int rs, IoPortId rs_port,
            unsigned int rw, IoPortId rw_port,
            unsigned int en, IoPortId en_port,
            unsigned int d0, IoPortId d0_port,
            unsigned int d1, IoPortId d1_port,
            unsigned int d2, IoPortId d2_port,
            unsigned int d3, IoPortId d3_port,
            unsigned int d4, IoPortId d4_port,
            unsigned int d5, IoPortId d5_port,
            unsigned int d6, IoPortId d6_port,
            unsigned int d7, IoPortId d7_port) {
  _rs = rs;
  _rw = rw;
  _en = en;
  _d0 = d0;
  _d1 = d1;
  _d2 = d2;
  _d3 = d3;
  _d4 = d4;
  _d5 = d5;
  _d6 = d6;
  _d7 = d7;

  _rs_port = rs_port;
  _rw_port = rw_port;
  _en_port = en_port;
  _d0_port = d0_port;
  _d1_port = d1_port;
  _d2_port = d2_port;
  _d3_port = d3_port;
  _d4_port = d4_port;
  _d5_port = d5_port;
  _d6_port = d6_port;
  _d7_port = d7_port;
  LcdConfigAllAsOutputs();
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, LCD_BOOT);
  LcdEnableOn();
  DelayUs(1);
  LcdEnableOff();
  DelayMs(5);
  LcdEnableOn();
  DelayUs(1);
  LcdEnableOff();
  DelayUs(200);
  LcdEnableOn();
  DelayUs(1);
  LcdEnableOff();
  DelayUs(200);
}

void LcdClearDisplay() {
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, LCD_INSTR_CLEAR);
  LcdEnableOn();
  DelayMs(2);	// 1.52 ms on datasheet
  LcdEnableOff();
}

void LcdReturnHome() {
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, LCD_INSTR_RETHOME);
  LcdEnableOn();
  DelayMs(2);	// 1.52 ms on datasheet
  LcdEnableOff();
}

void LcdSetEntryMode(unsigned char ddram_address_gain, unsigned char shift_display) {
  unsigned char disp_out;
  disp_out = 0x04 | ddram_address_gain | shift_display;
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);
  LcdEnableOn();
  DelayUs(50);	// 37 us on datasheet
  LcdEnableOff();
}

void LcdSetDisplayMode(unsigned char display_control, unsigned char cursor_control, unsigned char cursor_blink_control) {
  unsigned char disp_out;
  disp_out = 0x08 | display_control | cursor_control | cursor_blink_control;
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);
  LcdEnableOn();
  DelayUs(50);	// 37 us on datasheet
  LcdEnableOff();
}

void LcdShiftCursorOrDisplay(unsigned char shift_select, unsigned char shift_direction) {
  unsigned char disp_out;
  disp_out = 0x10 | shift_select << 3 | shift_direction;
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);
  LcdEnableOn();
  DelayUs(50);	// 37 us on datasheet
  LcdEnableOff();
}

void LcdSetFunction(unsigned char interface_length_control, unsigned char line_number_control, unsigned char dots_display_control) {
  unsigned char disp_out;
  disp_out = 0x20 | interface_length_control | line_number_control | dots_display_control;
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);
  LcdEnableOn();
  DelayUs(50);	// 37 us on datasheet
  LcdEnableOff();
}

void LcdSetCGRAMAddress(unsigned char address) {
  unsigned char disp_out;
  disp_out = address & LCD_CGRAM_MASK;
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);
  LcdEnableOn();
  DelayUs(50);  // 37 us on datasheet
  LcdEnableOff();
}

void LcdSetDDRAMAddress(unsigned char address) {
  unsigned char disp_out;
  disp_out = address & LCD_DDRAM_MASK;
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);
  LcdEnableOn();
  DelayUs(50);	// 37 us on datasheet
  LcdEnableOff();
}

static void LcdSetDataOutputs(unsigned char data) {
	// set each output pin or clear it
  data & 0x80 ? PORTSetBits(_d7_port, _d7) : PORTClearBits(_d7_port, _d7);
  data & 0x40 ? PORTSetBits(_d6_port, _d6) : PORTClearBits(_d6_port, _d6);
  data & 0x20 ? PORTSetBits(_d5_port, _d5) : PORTClearBits(_d5_port, _d5);
  data & 0x10 ? PORTSetBits(_d4_port, _d4) : PORTClearBits(_d4_port, _d4);
  data & 0x08 ? PORTSetBits(_d3_port, _d3) : PORTClearBits(_d3_port, _d3);
  data & 0x04 ? PORTSetBits(_d2_port, _d2) : PORTClearBits(_d2_port, _d2);
  data & 0x02 ? PORTSetBits(_d1_port, _d1) : PORTClearBits(_d1_port, _d1);
  data & 0x01 ? PORTSetBits(_d0_port, _d0) : PORTClearBits(_d0_port, _d0);
}

void LcdDisplayData(unsigned char *data) {
  while(*data) {
        switch (*data)
        {
        case '\n':          // move to second line
            LcdSetDDRAMAddress(LINE_2);
            break;
        case '\r':          // home, point to first line
            LcdSetDDRAMAddress(LINE_1);	// may use return home here?
            break;
        default:            // print character
            LcdSetOutputs(LCD_DATA, LCD_WRITE, *data);  // this requires delay
            LcdEnableOn();
            DelayUs(1); // datasheet says 400 ns = .4 us
            LcdEnableOff();
            break;
        }
        data++;
    }
}
static void LcdSetOutputs(int rs, int rw, unsigned char c) {
  rs ? PORTSetBits(_rs_port, _rs) : PORTClearBits(_rs_port, _rs);
  rw ? PORTSetBits(_rw_port, _rw) : PORTClearBits(_rw_port, _rw);
  LcdSetDataOutputs(c);
}

static inline void LcdConfigAllAsOutputs() {
  PORTSetPinsDigitalOut(_rs_port, _rs);
  PORTSetPinsDigitalOut(_rw_port, _rw);
  PORTSetPinsDigitalOut(_en_port, _en);
  PORTSetPinsDigitalOut(_d0_port, _d0);
  PORTSetPinsDigitalOut(_d1_port, _d1);
  PORTSetPinsDigitalOut(_d2_port, _d2);
  PORTSetPinsDigitalOut(_d3_port, _d3);
  PORTSetPinsDigitalOut(_d4_port, _d4);
  PORTSetPinsDigitalOut(_d5_port, _d5);
  PORTSetPinsDigitalOut(_d6_port, _d6);
  PORTSetPinsDigitalOut(_d7_port, _d7);
}

static void LcdEnableOn() {
  PORTSetBits(_en_port, _en);
}

static void LcdEnableOff() {
  PORTClearBits(_en_port, _en);
}