#include <p32xxxx.h>
#include <plib.h>

#pragma config FNOSC = PRIPLL	// Primary Osc w/PLL (XT+,HS+,EC+PLL)
#pragma config POSCMOD = HS		// HS osc mode
#pragma config FPLLMUL = MUL_18	// PLL Multiplier
#pragma config FPLLIDIV = DIV_2	// PLL Input Divider
#pragma config FPBDIV = DIV_2
#pragma config FPLLODIV = DIV_1
#pragma config FWDTEN = OFF

#define DELAY 4096
#define SQUARE(x) (x*x)
#define SYSTEM_FREQUENCY 72000000L

#define TEST_PIN BIT_2
#define TEST_PORT IOPORT_B

int main(void)
{
    int pbFreq;
    int i, j;

    pbFreq = SYSTEMConfigPerformance(SYSTEM_FREQUENCY);

    PORTSetPinsDigitalOut(TEST_PORT, TEST_PIN);

    while(1) {
        PORTSetBits(TEST_PORT, TEST_PIN);
        i = SQUARE(DELAY);
        while(i--);
        PORTClearBits(TEST_PORT, TEST_PIN);
        i = SQUARE(DELAY);
        while(i--);
    }

    return 0;
}