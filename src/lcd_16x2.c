#include lcd_16x2.h
#include <p32xxxx.h>
#include <plib.h>

static int _rs, _rw, _en;
static int _d0, _d1, _d2, _d3, _d4, _d5, _d6, _d7;
static int _d0_port, _d1_port, _d2_port, _d3_port, _d4_port, _d5_port, _d6_port, _d7_port;

void LCDInit(int rs, int rs_port,
             int rw, int rw_port,
             int en, int en_port,
             int d0, int d0_port,
             int d1, int d1_port,
             int d2, int d2_port,
             int d3, int d3_port,
             int d4, int d4_port,
             int d5, int d5_port,
             int d6, int d6_port,
             int d7, int d7_port) {
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
  LcdConfigAsOutputs();
  LcdSetDDRAMAddress(LINE_1);
}

void LcdClearDisplay() {
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, 0x01);
  // TODO enable and delay
}

void LcdReturnHome() {
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, (0x01<<1));
  // TODO enable and delay
}

void LcdSetEntryMode(int ddram_address_gain, int shift_display) {
  char disp_out;
  disp_out = 0x04 | ddram_address_gain | shift_display;
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);
  // TODO enable and delay
}

void LcdSetDisplayMode(int display_control, int cursor_control, int cursor_blink_control) {
  char disp_out;
  disp_out = 0x08 | display_control | cursor_control | cursor_blink_control;
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);
  // TODO enable and delay
}

void LcdSetShiftCursorOrDisplay(int shift_select, int shift_direction) {
  char disp_out;
  disp_out = 0x10 | shift_select << 3 | shift_direction;
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);
  // TODO enable and delay
}

void LcdSetFunction(int interface_length_control, int line_number_control, int dots_display_control) {
  char disp_out;
  disp_out = 0x20 | interface_length_control | line_number_control | dots_display_control;
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);
  // TODO enable and delay
}
void LcdSetCGRAMAddress(int address) {
}
void LcdSetDDRAMAddress(int address) {
}
int LcdReadBusyFlagAndAddress(int address) {
}
void LcdWriteRAM(char data) {
}
int LcdReadRAM(char address) {
}
void LcdDisplayData(char *data) {
}
static void LcdSetOutputs(int rs, int rw, char c) {
}
static void LcdConfigAllAsOutputs() {
  PORTSetPinsDigitalOut()(_rs_port, _rs);
  PORTSetPinsDigitalOut()(_rw_port, _rw);
  PORTSetPinsDigitalOut()(_en_port, _en);
  PORTSetPinsDigitalOut()(_d0_port, _d0);
  PORTSetPinsDigitalOut()(_d1_port, _d1);
  PORTSetPinsDigitalOut()(_d2_port, _d2);
  PORTSetPinsDigitalOut()(_d3_port, _d3);
  PORTSetPinsDigitalOut()(_d4_port, _d4);
  PORTSetPinsDigitalOut()(_d5_port, _d5);
  PORTSetPinsDigitalOut()(_d6_port, _d6);
  PORTSetPinsDigitalOut()(_d7_port, _d7);
}
static void LcdConfigAsInputs() {
}
