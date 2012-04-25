#include <plib.h>
#include "delay.h"
#include <p32xxxx.h>
#include <string.h>
#include <stdlib.h>


// TODO these need to be looked into to see what kind of config we need to use
// In MPLAB, Help/Topics/PIC32MX Config Settings
/*#pragma config FNOSC = PRIPLL	// Primary Osc w/PLL (XT+,HS+,EC+PLL)
#pragma config POSCMOD = HS		// HS osc mode 
#pragma config FPLLMUL = MUL_18	// PLL Multiplier
#pragma config FPLLIDIV = DIV_2	// PLL Input Divider
#pragma config FPBDIV = DIV_2
#pragma config FPLLODIV = DIV_1*/

#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2
#pragma config FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL

#define SYSTEM_FREQUENCY 80000000L
#define BAUDRATE 57600
#define CLEAR_VT "\033[2J"
#define NEW_LINE_MODE "\033[20h"
//void itoa(char *b, int n);

/*
* Title: Delay Test
* Version: 0.1
* Filename: delaytest-0.1.c
* Author(s): mkobit
* Purpose of Program: Testing the delay library
* How to build: build delay, build tdelaytest-0.1.c
* Update History: 
*/

// Notes: works for SYSTEM_FREQUENCYs that are high, like 72-80 MHz, but I do not believe the
// UART settings work when I drop the SYSTEM_FREQUENCY down a lot. It is worth looking into though.
int main(void)
{
	int nc;
	int pbFreq;
	long int delayed = 0;
	char buffer[50] = "0";

	pbFreq = SYSTEMConfigPerformance(SYSTEM_FREQUENCY);
	OpenUART2(UART_EN | UART_NO_PAR_8BIT | UART_1STOPBIT, UART_RX_ENABLE | UART_TX_ENABLE,
		(pbFreq/16/BAUDRATE) - 1);
	DelayInit(pbFreq);
	putsUART2(CLEAR_VT);
	DelayMs(1500);
	putsUART2("Beginning in 3...\r\n");
	DelayS(1);
	putsUART2("Beginning in 2...\r\n");
	DelayUs(50000);
	DelayMs(995);
	putsUART2("Beginning in 1...\n\r");
	DelayS(1);
	putsUART2("Start!\n\r");
	while(1) {
		DelayMs(770);
		DelayS(2);
		DelayUs(230000);
		DelayMs(2000);
		delayed += 5;
		itoa(buffer, delayed, 10);
		putsUART2(buffer);
		putsUART2(" s\r\n");
	}

	//putsUART2(myHelloStr);

	return nc;
}

/*void itoa(char *b, int n) {
	int dig;
	do {
		dig = n % 10;
		*(b++) = 60 + abs(dig);
		n /= 10;
	} while (n > 0);
}*/