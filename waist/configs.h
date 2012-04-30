// This contains the configurations that are used on the PCB microcontroller
// Target: PIC32MX564F128H

#pragma config FWDTEN = OFF // Watchdog timer disabled

// PLL and clock settings
#pragma config FPLLMUL = MUL_20	// PLL Multiplier
#pragma config FPLLIDIV = DIV_2	// PLL Input Divider
#pragma config FPLLODIV = DIV_1
#pragma config FNOSC = FRCPLL	// Fast RC Osc with PLL

#pragma config FSOSCEN = OFF  //Secondary Oscillator disabled
#pragma config IESO = OFF     // Internal/External Switch Over disabled
#pragma config POSCMOD = OFF  // Primary osc disabled
#pragma config OSCIOFNC = OFF // CLKO Output Signal Active on the OSCO Pin disabled
#pragma config FPBDIV = DIV_1 // Peripheral Clock Divisor
#pragma config FCKSM = CSDCMD // Clock Switching and Monitor Selection disabled
// Disable USB clock
#pragma config UPLLEN = OFF

#pragma config DEBUG = ON
#pragma config ICESEL = ICS_PGx1