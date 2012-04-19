#include <p32xxxx.h>
#include <plib.h>
#include "lcd_16x2.h"
#include "delay.h"

// Pin numbers
static unsigned int _rs, _rw, _en;  
static unsigned int _d0, _d1, _d2, _d3, _d4, _d5, _d6, _d7;

// port IDs
static IoPortId _rs_port, _rw_port, _en_port;  
static IoPortId _d0_port, _d1_port, _d2_port, _d3_port, _d4_port, _d5_port, _d6_port, _d7_port;


static void LcdSetOutputs(int rs, int rw, unsigned char c);
static inline void LcdConfigAllAsOutputs();
static void LcdToggleEnable();
static void LcdSetDataOutputs(unsigned char data);

/************************************************************************************************** 
  Function: 
    void LcdInit(unsigned int rs, IoPortId rs_port,
            unsigned int rw, IoPortId rw_port,
            unsigned int en, IoPortId en_port,
            unsigned int d0, IoPortId d0_port,
            unsigned int d1, IoPortId d1_port,
            unsigned int d2, IoPortId d2_port,
            unsigned int d3, IoPortId d3_port,
            unsigned int d4, IoPortId d4_port,
            unsigned int d5, IoPortId d5_port,
            unsigned int d6, IoPortId d6_port,
            unsigned int d7, IoPortId d7_port,
            unsigned char dots_display_control)
  
  Author(s):
    mkobit
  
  Summary: 
    Sets static variables in LCD module and initalizes the LCD
  
  Description: 
    This should be called before any other LCD functions are called
    Uses the boot-up sequence described by the data sheet to initialize the LCD
  
  Preconditions: 
    DelayInit called to setup Delay library. This is used for almost all functions, so it must be initialized.
  
  Parameters: 
    unsigned int rs   - rs pin
    IoPortId rs_port  - rs port
    unsigned int rw   - rw pin
    IoPortId rw_port  - rw port
    unsigned int en   - en pin
    IoPortId en_port  - en port
    unsigned int d0   - data 0 pin
    IoPortId d0_port  - data 0 port
    unsigned int d1   - *
    IoPortId d1_port  - *
    unsigned int d2   - *
    IoPortId d2_port  - *
    unsigned int d3   - *
    IoPortId d3_port  - *
    unsigned int d4   - *
    IoPortId d4_port  - *
    unsigned int d5   - *
    IoPortId d5_port  - *
    unsigned int d6   - *
    IoPortId d6_port  - *
    unsigned int d7   - *
    IoPortId d7_port  - *
    unsigned char dots_display_control - constant from library to determine how many dots to display
      LCD_DOTS_5x11
      LCD_DOTS_5x8
  
  Returns: 
    void
  
  Example: 
    <code>
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
        LCD_DOTS_5x8);
    </code>
  
  Conditions at Exit: 
    All pins configured as outputs
    LCD initialized and ready to be written to
    Static variables set
  
**************************************************************************************************/
void LcdInit(unsigned int rs, IoPortId rs_port,
            unsigned int rw, IoPortId rw_port,
            unsigned int en, IoPortId en_port,
            unsigned int d0, IoPortId d0_port,
            unsigned int d1, IoPortId d1_port,
            unsigned int d2, IoPortId d2_port,
            unsigned int d3, IoPortId d3_port,
            unsigned int d4, IoPortId d4_port,
            unsigned int d5, IoPortId d5_port,
            unsigned int d6, IoPortId d6_port,
            unsigned int d7, IoPortId d7_port,
            unsigned char dots_display_control) {
  
  unsigned char disp_out;

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
  
  // Pins initalized, now do bootup sequence for LCD
  DelayMs(50);  // delay for LCD to be fully powered
  disp_out = 0x20 | LCD_INTERFACE_LENGTH_8BIT | LCD_LINES_2 | dots_display_control; // Command "Set Interface"
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);  // Instruction - SetFunction
  LcdToggleEnable();
  DelayMs(5); // > 4.1 ms
  LcdToggleEnable();
  DelayUs(200); // > 100 us
  LcdToggleEnable();
  DelayUs(200); // > 100 us, last time
  // Initialization commands
  // Write Command "Set Interface", already on bus
  LcdToggleEnable();  
  DelayUs(43);  // 37 us + 15%
  // Write Command "Enable Display/Cursor"
  LcdInstrSetDisplayMode(LCD_DISPLAY_OFF, LCD_CURSOR_OFF, LCD_CURSOR_BLINK_OFF);
  // Write Command "Clear and Home"
  LcdInstrClearDisplay();
  // Write Command "Set Entry Mode"
  LcdInstrSetEntryMode(LCD_DDRAM_ADDRESS_INCR, LCD_SHIFT_DISPLAY_OFF);
}

/************************************************************************************************** 
  Function: 
    void LcdInstrClearDisplay()
  
  Author(s):
    mkobit
  
  Summary: 
    Clears the LCD display
  
  Description: 
    Sends the LCD_INSTR_CLEAR to the LCD as an instruction
    This instruction takes about 1.52 ms, so the delay is set to 2 ms
  
  Preconditions: 
    LcdInit called prior
    None of the pins have been reconfigured or used for anything else
  
  Parameters: 
    None
  
  Returns: 
    void
  
  Example: 
    <code>
    LcdInstrClearDisplay()
    </code>
  
  Conditions at Exit: 
    LCD is cleared and the cursor is in the home position
  
**************************************************************************************************/
void LcdInstrClearDisplay() {
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, LCD_INSTR_CLEAR);
  LcdToggleEnable();
  DelayMs(2);  // 1.52 ms on datasheet
}

/************************************************************************************************** 
  Function: 
    void LcdInstrReturnHome()
  
  Author(s):
    mkobit
  
  Summary: 
    Returns the cursor in the LCD to the Home position
  
  Description: 
    Sends the LCD_INSTR_RETHOME to the LCD as an instruction
    This instruction takes about 1.52 ms, so the delay is set to 2 ms
  
  Preconditions: 
    LcdInit called prior
    None of the pins have been reconfigured or used for anything else
  
  Parameters: 
    None
  
  Returns: 
    void
  
  Example: 
    <code>
    LcdInstrReturnHome()
    </code>
  
  Conditions at Exit: 
    Cursor is in the home position
  
**************************************************************************************************/
void LcdInstrReturnHome() {
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, LCD_INSTR_RETHOME);
  LcdToggleEnable();
  DelayMs(2);  // 1.52 ms on datasheet
}

/************************************************************************************************** 
  Function: 
    void LcdInstrSetEntryMode(unsigned char ddram_address_gain, unsigned char shift_display)
  
  Author(s):
    mkobit
  
  Summary:
    Sets cursor move direction and specifies display shift
  
  Description: 
    The shift/move wirection will happen on a read/write
  
  Preconditions: 
    LcdInit called prior
    None of the pins have been reconfigured or used for anything else
  
  Parameters: 
    unsigned char ddram_address_gain - movement direction of DDRAM on read/write
      LCD_DDRAM_ADDRESS_INCR - increments DDRAM
      LCD_DDRAM_ADDRESS_DECR - decrements DDRAM
    unsigned char shift_display - shifts entire screen on read/write according to (ddram_address_gain)
      LCD_SHIFT_DISPLAY_ON    - shifts display left when (ddram_address_gain) is LCD_DDRAM_ADDRESS_INCR, shifts display right when 0 or LCD_DDRAM_ADDRESS_DECR
      LCD_SHIFT_DISPLAY_OFF   - do not shift display

  Returns: 
    void
  
  Example: 
    <code>
    LcdInstrSetEntryMode(LCD_DDRAM_ADDRESS_INCR, LCD_SHIFT_DISPLAY_OFF)
    </code>
  
  Conditions at Exit: 
    Entry mode of the LCD is set to shift when data is written/read
  
**************************************************************************************************/
void LcdInstrSetEntryMode(unsigned char ddram_address_gain, unsigned char shift_display) {
  unsigned char disp_out;
  disp_out = 0x04 | ddram_address_gain | shift_display;
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);
  LcdToggleEnable();
  DelayUs(43);  // 37 us + 15%
}

/************************************************************************************************** 
  Function: 
    void LcdInstrSetDisplayMode(unsigned char display_control, unsigned char cursor_control, unsigned char cursor_blink_control)
  
  Author(s):
    mkobit
  
  Summary: 
    Sets various settings of the display
  
  Description: 
    Sets LCD to turn on/off display, turn on/off cursor, and turn on/off cursor blinking
  
  Preconditions: 
    LcdInit called prior
    None of the pins have been reconfigured or used for anything else
  
  Parameters: 
    unsigned char display_control - turn display on/off
      LCD_DISPLAY_ON
      LCD_DISPLAY_OFF
    unsigned char cursor_control - turn on/off cursor
      LCD_CURSOR_ON
      LCD_CURSOR_OFF
    unsigned char cursor_blink_control - turn on/off cursor blinking
      LCD_CURSOR_BLINK_ON
      LCD_CURSOR_BLINK_OFF
  
  Returns: 
    void
  
  Example: 
    <code>
    LcdInstrSetDisplayMode(LCD_DISPLAY_OFF, LCD_CURSOR_OFF, LCD_CURSOR_BLINK_OFF)
    </code>
  
  Conditions at Exit: 
    All display options set as per what the description and parameter states
  
**************************************************************************************************/
void LcdInstrSetDisplayMode(unsigned char display_control, unsigned char cursor_control, unsigned char cursor_blink_control) {
  unsigned char disp_out;
  disp_out = 0x08 | display_control | cursor_control | cursor_blink_control;
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);
  LcdToggleEnable();
  DelayUs(43);  // 37 us + 15%
}

/************************************************************************************************** 
  Function: 
    void LcdInstrShiftCursorOrDisplay(unsigned char shift_select, unsigned char shift_direction)
  
  Author(s):
    mkobit
  
  Summary: 
    Set cursor moving and display shift control bit, and the direction, without changing DDRAM data
  
  Description: 
    Shifts the cursor or the display without changing the DDRAM data.
  
  Preconditions: 
    LcdInit called prior
    None of the pins have been reconfigured or used for anything else
  
  Parameters: 
    unsigned char shift_select  - shift display/cursor
      LCD_SHIFT_SELECT_DISPLAY - shifts display, cursor follows shift
      LCD_SHIFT_SELECT_CURSOR
    unsigned char shift_direction - direction to shift cursor/display
      LCD_SHIFT_DIRECTION_RIGHT
      LCD_SHIFT_DIRECTION_LEFT
      
  Returns: 
    void
  
  Example: 
    <code>
    LcdInstrShiftCursorOrDisplay(LCD_SHIFT_SELECT_DISPLAY, LCD_SHIFT_DIRECTION_RIGHT)
    </code>
  
  Conditions at Exit: 
    Shifts LCD/cursor in a certain direction
  
**************************************************************************************************/
void LcdInstrShiftCursorOrDisplay(unsigned char shift_select, unsigned char shift_direction) {
  unsigned char disp_out;
  disp_out = 0x10 | shift_select | shift_direction;
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);
  LcdToggleEnable();
  DelayUs(43);  // 37 us + 15%
}

/************************************************************************************************** 
  Function: 
    void LcdInstrSetCGRAMAddress(unsigned char address)
  
  Author(s):
    mkobit
  
  Summary: 
    Sets the CGRAM address in the LCD
  
  Description: 
    Writes an instruction to the LCD telling to change its CGRAM address to (address)
  
  Preconditions: 
    LcdInit called prior
    None of the pins have been reconfigured or used for anything else
  
  Parameters: 
    unsigned char address - address for the CGRAM to be changed to
  
  Returns: 
    void
  
  Example: 
    <code>
    LcdInstrSetCGRAMAddress(0x64)
    </code>
  
  Conditions at Exit: 
    LCD CGRAM address set to (address)
  
**************************************************************************************************/
void LcdInstrSetCGRAMAddress(unsigned char address) {
  unsigned char disp_out;
  disp_out = address & LCD_CGRAM_MASK;
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);
  LcdToggleEnable();
  DelayUs(43);  // 37 us + 15%
}

/************************************************************************************************** 
  Function: 
    void LcdInstrSetDDRAMAddress(unsigned char address)
  
  Author(s):
    mkobit
  
  Summary: 
    Sets the DDRAM address in the LCD
  
  Description: 
    Writes an instruction to the LCD telling to change its DDRAM address to (address)
  
  Preconditions: 
    LcdInit called prior
    None of the pins have been reconfigured or used for anything else
  
  Parameters: 
    unsigned char address - address for the DDRAM to be changed to
  
  Returns: 
    void
  
  Example: 
    <code>
    LcdInstrSetDDRAMAddress(LINE_2)
    </code>
  
  Conditions at Exit: 
    LCD DDRAM address set to (address)
    
  Notes:
    4/17/12 - There is no protection guaranteed here from trying to set the address to some place that is out of bounds. User's responsibility for now
  
**************************************************************************************************/
void LcdInstrSetDDRAMAddress(unsigned char address) {
  unsigned char disp_out;
  disp_out = address & LCD_DDRAM_MASK;
  LcdSetOutputs(LCD_INSTR, LCD_WRITE, disp_out);
  LcdToggleEnable();
  DelayUs(43);  // 37 us + 15%
}

/************************************************************************************************** 
  Function: 
    static void LcdSetDataOutputs(unsigned char data)
  
  Author(s):
    mkobit
  
  Summary: 
    Sets/clears digital outs for (d0-d7)
  
  Description: 
    Does logical operations on each bit to determine which pins should be set/cleared
  
  Preconditions: 
    LcdInit called prior
    None of the pins have been reconfigured or used for anything else
    Static function, used by internal library
  
  Parameters: 
    unsigned char data - character data to be used to set/clear digital output pins
  
  Returns: 
    void
  
  Example: 
    <code>
    char c = 0x38
    LcdSetDataOutputs(c)
    </code>
  
  Conditions at Exit: 
    (d0-d7) set/cleared based on input
  
**************************************************************************************************/
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

/************************************************************************************************** 
  Function: 
    
  
  Author(s):
    mkobit
  
  Summary: 
    Writes a string of data to the LCD
  
  Description: 
    Cycles through string and writes the characters to the LCD until it hits a null character -> '/0'
  
  Preconditions: 
    LcdInit called prior
    None of the pins have been reconfigured or used for anything else
  
  Parameters: 
    unsigned char *data
  
  Returns: 
    void
  
  Example: 
    <code>
    LcdDisplayData("Hello\nThere!")
    </code>
    
  Conditions at Exit: 
    LCD DDRAM address will be increased by strlen(data)
    LCD data displayed on LCD
    
  Note:
    4/17/12 - It is the user's responsibility to make sure that the string is well formed
    
**************************************************************************************************/
void LcdDisplayData(unsigned char *data) {
  while(*data) {
    switch (*data)
    {
      case '\n':          
        // move to second line
        LcdInstrSetDDRAMAddress(LINE_2);
        break;
      case '\r':          
        // Home, point to first line
        LcdInstrSetDDRAMAddress(LINE_1);  // may use return home here?
        break;
      default:            
        // print character to LCD
        LcdDisplayChar(*data);
        break;
    }
    data++;
  }
}

/************************************************************************************************** 
  Function: 
    void LcdDisplayChar(unsigned char c)
  
  Author(s):
    mkobit
  
  Summary: 
    Writes a single character to the LCD
  
  Description: 
    Sets the digital outputs and waits for a time period specified by the data sheet
  
  Preconditions: 
    LcdInit called prior
    None of the pins have been reconfigured or used for anything else
  
  Parameters: 
    unsigned char c - character to write to the LCD
  
  Returns: 
    void
  
  Example: 
    <code>
    LcdDisplayChar('!')
    </code>
    
  Conditions at Exit: 
    Character written to LCD
    DDRAM of LCD increased by 1
  
**************************************************************************************************/
void LcdDisplayChar(unsigned char c) {
  LcdSetOutputs(LCD_DATA, LCD_WRITE, c);  // this requires delay
  LcdToggleEnable();
  DelayUs(50);// 43 us + 15%
}

/************************************************************************************************** 
  Function: 
    static void LcdSetOutputs(int rs, int rw, unsigned char c)
  
  Author(s):
    mkobit
  
  Summary: 
    Sets/clears the digital outputs
  
  Description: 
    Static function, used by internal library
  
  Preconditions: 
    LcdInit called prior
    None of the pins have been reconfigured or used for anything else
  
  Parameters: 
    int rs - if register select should be enabled
    int rw - if rw should be enabled 
    unsigned char c - character to be written to LCD
  
  Returns: 
    void
  
  Example: 
    <code>
    LcdSetOutputs(LCD_DATA, LCD_WRITE, data)
    </code>
  
  Conditions at Exit: 
    Digital output pins set/cleared for rs, rw, and d0-d7
  
**************************************************************************************************/
static void LcdSetOutputs(int rs, int rw, unsigned char c) {
  rs ? PORTSetBits(_rs_port, _rs) : PORTClearBits(_rs_port, _rs);
  rw ? PORTSetBits(_rw_port, _rw) : PORTClearBits(_rw_port, _rw);
  LcdSetDataOutputs(c);
}

/************************************************************************************************** 
  Function: 
    static inline void LcdConfigAllAsOutputs()
  
  Author(s):
    mkobit
  
  Summary: 
    Configures all static pins/ports as digital outputs
  
  Description: 
    Static function, used by internal library
  
  Preconditions: 
    None
  
  Parameters: 
    void
  
  Returns: 
    None
  
  Example: 
    None, used in LcdInit
  
  Conditions at Exit: 
    All static pins/ports configured as digital outputs
  
**************************************************************************************************/
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

/************************************************************************************************** 
  Function: 
    static void LcdToggleEnable()
  
  Author(s):
    mkobit
  
  Summary: 
    Toggles enable to send a pulse to the LCD
  
  Description:
    1 us delay used because 400 nanosecond minimum time.
    Static function, used by internal library
  
  Preconditions: 
    LcdInit called prior
    None of the pins have been reconfigured or used for anything else
  
  Parameters:
    None
  
  Returns: 
    void
  
  Example: 
    <code>
    LcdToggleEnable()
    </code>
  
  Conditions at Exit: 
    (_en) pin cleared
  
**************************************************************************************************/
static void LcdToggleEnable() {
  PORTSetBits(_en_port, _en);
  DelayUs(1); // 1 us delay for hold time of enable
  PORTClearBits(_en_port, _en);
}