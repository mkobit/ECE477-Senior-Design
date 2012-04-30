#include <p32xxxx.h>
#include <plib.h>

#pragma config FNOSC = PRIPLL	// Primary Osc w/PLL (XT+,HS+,EC+PLL)
#pragma config POSCMOD = HS		// HS osc mode 
#pragma config FPLLMUL = MUL_18	// PLL Multiplier
#pragma config FPLLIDIV = DIV_2	// PLL Input Divider
#pragma config FPBDIV = DIV_2
#pragma config FPLLODIV = DIV_1
#pragma config FWDTEN = OFF
#pragma config DEBUG = ON
#pragma config ICESEL = ICS_PGx1

#define DELAY (1000)
#define SQUARE(x) (x*x)
#define SYSTEM_FREQUENCY 72000000L

#define TEST_SIZE 11

typedef struct TEST_PAIR {
    unsigned int bitnum;
    IoPortId port_id;
} TEST_PAIR;


// pins and ports to be used for testing
// rs, rw,  en, d0-d7
TEST_PAIR pin_pairs[TEST_SIZE] = {
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
    Multiple Pin Test
    
  Version: 
    0.11
    
  Filename: 
    multpintest.c
    
  Author(s): 
    mkobit
    
  Purpose of Program: 
    Tests multiple pins to see if they can be configured as digital outs and be toggled.
    Used on PCB
    
  How to build: 
    
    
  Update History: 
    
    
**************************************************/
int main(void)
{
    int pbFreq;
    int i, j;
    TEST_PAIR *currentpair;

    pbFreq = SYSTEMConfigPerformance(SYSTEM_FREQUENCY);
    for (j = 0; j < TEST_SIZE; j++) {
        currentpair = &pin_pairs[j];
        PORTSetPinsDigitalOut(currentpair->port_id, currentpair->bitnum);
    }
    while(1) {
        for (j = 0; j < TEST_SIZE; j++) {
            currentpair = &pin_pairs[j];
            PORTSetBits(currentpair->port_id, currentpair->bitnum);
            i = SQUARE(DELAY);
            while (i >= 0) {
              i--;
            }
        }
        for (j = 0; j < TEST_SIZE; j++) {
            currentpair = &pin_pairs[j];
            PORTClearBits(currentpair->port_id, currentpair->bitnum);
            i = SQUARE(DELAY);
            while (i >= 0) {
              i--;
            }
        }
    }

    return 0;
}