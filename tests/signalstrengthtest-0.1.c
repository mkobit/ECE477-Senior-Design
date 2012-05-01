#include <p32xxxx.h>
#include <plib.h>
#include "lcd_16x2.h"
#include "delay.h"
#include "xbee.h"

#include "configs.h"

#pragma config DEBUG = ON
#pragma config ICESEL = ICS_PGx1


#define SYSTEM_FREQUENCY 80000000L
#define LCDS_IN 11

#define LCD_DISP_THRESH 500

#define RSSI_PRE "RSSI: "
#define RSSI_INVAL "NO RSSI. SIGNAL"

#define TEST_UART_XBEE UART3
#define TEST_UART_BAUDRATE 57600

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


void Send2LineDisplay(char *line_1, char *line_2, const unsigned char bottomLineStartOffset);
void UpdateLCDStatus(const int signalPercent);

/**************************************************************************************************
  Title: 
    Signal Strength on LCD test
    
  Version: 
    0.1
    
  Filename: 
    signalstrengthtest-0.1.c
    
  Author(s): 
    mkobit
    
  Purpose of Program: 
    Test displaying of signal strength on the LCD as well as some functions in the main Swish Sleeve application
    
  How to build: 
    delay.c
    lcd_16x2.c - delay.c
    xbee.c
    
  Update History: 
    
    
**************************************************/
int main(void)
{
  int pbFreq;
  int signalPercent = -1;
  int updatePercent = 0;

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
  LcdInstrSetDisplayMode(LCD_DISPLAY_ON, LCD_CURSOR_OFF, LCD_CURSOR_BLINK_ON);
  XBeeConfigure(TEST_UART_XBEE, pbFreq, TEST_UART_BAUDRATE);
  
  while(1) {
    LcdInstrReturnHome();
    if (signalPercent == -1) {
      signalPercent = XBeeCaptureSignalStrength();
    } else {
      updatePercent = XBeeCaptureSignalStrength();
      if (updatePercent == 100) {
        signalPercent = 100;
      }
      signalPercent = (signalPercent + updatePercent) / 2;
    }
    UpdateLCDStatus(signalPercent);
    DelayMs(500);
  }

  return 0;
}

void Send2LineDisplay(char *line_1, char *line_2, const unsigned char bottomLineStartOffset) {
  LcdInstrClearDisplay();
  LcdDisplayData(line_1);
  LcdInstrSetDDRAMAddress(LINE_2 + bottomLineStartOffset);
  LcdDisplayData(line_2);
}

void UpdateLCDStatus(const int signalPercent) {
  char line1[40];
  char percentageval[5];

  // Display signal message
  if (signalPercent < 0) {
    // invalid signal strength, copy to buffer
    strcpy(line1, RSSI_INVAL);
    // Append newline to data for battery message
  } else {
    strcpy(line1, RSSI_PRE);
    // Format battery string
    itoa(percentageval, signalPercent, 10);
    strcat(line1, percentageval);
    strcat(line1, "%");
  }
  Send2LineDisplay(line1, "N/A here", 4);
}