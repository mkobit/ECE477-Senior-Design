#ifndef LCD_16X2_H
#define LCD_16X2_H

/* Display character address code:
Display position:  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16
DDRAM Address:     00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F ... 27	-- shift mode for this use
DDRAM Address:     40 41 42 43 44 45 46 47 48 49 4A 4B 4C 4D 4E 4F ... 67
*/

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
#define LCD_SHIFT_SELECT_NONE 0
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
#define BYTE_MASK 0xFF


#define LINE_1 0x00
#define LINE_2 0x40


void LcdInitPins(int rs, int rs_port,
             int rw, int rw_port,
             int en, int en_port,
             int d0, int d0_port,
             int d1, int d1_port,
             int d2, int d2_port,
             int d3, int d3_port,
             int d4, int d4_port,
             int d5, int d5_port,
             int d6, int d6_port,
             int d7, int d7_port);
void LcdClearDisplay();
void LcdReturnHome();
void LcdSetEntryMode(char ddram_address_gain, char shift_display); // TODO comeback to this
void LcdSetDisplayMode(char display_control, char cursor_control, char cursor_blink_control);
void LcdShiftCursorOrDisplay(char shift_select, char shift_direction);
void LcdSetFunction(char interface_length_control, char line_number_control, char dots_display_control);
void LcdSetCGRAMAddress(char address);
void LcdSetDDRAMAddress(char address);
void LcdDisplayData(char *data);

#endif
