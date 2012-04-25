#ifndef LCD_16X2_H
#define LCD_16X2_H

/* Display character address code:
Display position:  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16
DDRAM Address:     00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F ... 27  -- shift mode goes this far
DDRAM Address:     40 41 42 43 44 45 46 47 48 49 4A 4B 4C 4D 4E 4F ... 67
*/

#define LCD_BOOT (0x30)

// Entry Mode Set 
#define LCD_DDRAM_ADDRESS_INCR (1 << 1)
#define LCD_DDRAM_ADDRESS_DECR 0
#define LCD_SHIFT_DISPLAY_ON 1
#define LCD_SHIFT_DISPLAY_OFF 0

// Display ON/OFF
#define LCD_DISPLAY_ON (1 << 2)
#define LCD_DISPLAY_OFF 0
#define LCD_CURSOR_ON (1 << 1)
#define LCD_CURSOR_OFF 0
#define LCD_CURSOR_BLINK_ON 1
#define LCD_CURSOR_BLINK_OFF 0

// Cursor or Display Shift 
#define LCD_SHIFT_SELECT_DISPLAY (1 << 3)
#define LCD_SHIFT_SELECT_CURSOR 0
#define LCD_SHIFT_DIRECTION_RIGHT (1 << 2)
#define LCD_SHIFT_DIRECTION_LEFT 0

// Function Set
#define LCD_INTERFACE_LENGTH_8BIT (1 << 4)
#define LCD_INTERFACE_LENGTH_4BIT 0
#define LCD_LINES_2 (1 << 3)
#define LCD_LINES_1 0
#define LCD_DOTS_5x11 (1 << 2)
#define LCD_DOTS_5x8 0

// Set CGRAM address 
#define LCD_CGRAM_MASK 0x3F

// Set DDRAM address 
#define LCD_DDRAM_MASK 0x7F

// Control variables for setting output pins
#define LCD_DATA 1
#define LCD_INSTR 0
#define LCD_READ 1
#define LCD_WRITE 0
#ifndef BYTE_MASK
  #define BYTE_MASK 0xFF
#endif

#define LCD_INSTR_CLEAR (0x01)
#define LCD_INSTR_RETHOME (0x02)

#define LINE_1 0x00
#define LINE_2 0x40


void LcdInit(const UINT rs, const IoPortId rs_port,
            const UINT rw, const IoPortId rw_port,
            const UINT en, const IoPortId en_port,
            const UINT d0, const IoPortId d0_port,
            const UINT d1, const IoPortId d1_port,
            const UINT d2, const IoPortId d2_port,
            const UINT d3, const IoPortId d3_port,
            const UINT d4, const IoPortId d4_port,
            const UINT d5, const IoPortId d5_port,
            const UINT d6, const IoPortId d6_port,
            const UINT d7, const IoPortId d7_port,
            const UINT8 dots_display_control);
void LcdInstrClearDisplay();
void LcdInstrReturnHome();
void LcdInstrSetEntryMode(const UINT8 ddram_address_gain, const UINT8 shift_display); // TODO comeback to this
void LcdInstrSetDisplayMode(const UINT8 display_control, const UINT8 cursor_control, const UINT8 cursor_blink_control);
void LcdInstrShiftCursorOrDisplay(const UINT8 shift_select, const UINT8 shift_direction);
void LcdInstrSetCGRAMAddress(const UINT8 address);
void LcdInstrSetDDRAMAddress(const UINT8 address);
void LcdDisplayData(UINT8 *data);
void LcdClearAndDisplayData(UINT8 *data);
void LcdDisplayChar(const UINT8 c);

#endif
