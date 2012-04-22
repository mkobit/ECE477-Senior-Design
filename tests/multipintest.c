#include <p32xxxx.h>
#include <plib.h>

#pragma config FNOSC = PRIPLL	// Primary Osc w/PLL (XT+,HS+,EC+PLL)
#pragma config POSCMOD = HS		// HS osc mode 
#pragma config FPLLMUL = MUL_18	// PLL Multiplier
#pragma config FPLLIDIV = DIV_2	// PLL Input Divider
#pragma config FPBDIV = DIV_2
#pragma config FPLLODIV = DIV_1

#define DELAY 2048
#define SQUARE(x) (x*x)
#define SYSTEM_FREQUENCY 72000000L

#define TEST_PIN BIT_11
#define TEST_PORT IOPORT_B

#define TEST_SIZE 11

typedef struct TEST_PAIR {
    unsigned int bitnum;
    IoPortId port_id;
} TEST_PAIR;


TEST_PAIR pin_pairs[TEST_SIZE] = {
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
            while(i--);
        }
        for (j = 0; j < TEST_SIZE; j++) {
            currentpair = &pin_pairs[j];
            PORTClearBits(currentpair->port_id, currentpair->bitnum);
            i = SQUARE(DELAY);
            while(i--);
        }
    }

    return 0;
}