#include "delay.h"
#include <p32xxxx.h>
#include <plib.h>

static unsigned int core_hz;
static unsigned int core_ticks_in_us;	// stored locally so that these delays will be more accurate
static unsigned int core_ticks_in_ms;

/* DelayInit
* Parameters: 
	unsigned int clock_ticks - number of ticks in a second by the main system clock
* Author(s): mkobit
* Called by: main program
* Purpose/Function Performed: inits variables to be used in other delay functions
* Calling Convention: DelayInit(80000000L)
* Conditions at Exit: Nothing else changes core timer period or uses it
* Date Started: 
* Update History: 
*/
void DelayInit(unsigned int clock_hz) {
	core_hz = clock_hz / 2;	// runs at half-speed of system clock
	core_ticks_in_us = DELAY_US_TO_CT_TICKS(core_hz);
	core_ticks_in_ms = DELAY_MS_TO_CT_TICKS(core_hz);
	WriteCoreTimer(0);
}

/* DelayS
* Parameters: 
	unsigned int s - seconds to delay: should not be an excessively high number
* Author(s): mkobit
* Called by: any function that wants a fairly precise delay
* Purpose/Function Performed: delays for amount of seconds that user specifies
* Calling Convention: DelayS(2)
* Conditions at Exit: Nothing else changes core timer period or uses it
* Date Started: 3/29/12
* Update History: 
*/
void DelayS(unsigned int s) {
	unsigned int start;
	unsigned int ticks_to_wait;

	ticks_to_wait = s * core_hz;
	WriteCoreTimer(0);	// clear timer
	start = ReadCoreTimer();
	while ((unsigned int) (ReadCoreTimer() - start) < ticks_to_wait) {};
}

/* DelayMs
	unsigned int ms - milliseconds to delay
* Author(s): mkobit
* Called by: any function that wants a fairly precise delay
* Purpose/Function Performed: delays for amount of milliseconds that user specifies
* Calling Convention: DelayMs(250)
* Conditions at Exit: Nothing else changes core timer period or uses it
* Date Started: 3/29/12
* Update History: 
*/
void DelayMs(unsigned int ms) {
	unsigned int start;
	unsigned int ticks_to_wait;

	ticks_to_wait = ms * core_ticks_in_ms;
	WriteCoreTimer(0);	// clear timer
	start = ReadCoreTimer();
	while ((unsigned int) (ReadCoreTimer() - start) < ticks_to_wait) {};
}

/* DelayUs
	unsigned int us - milliseconds to delay
* Author(s): mkobit
* Called by: any function that wants a fairly precise delay
* Purpose/Function Performed: delays for amount of microseconds that user specifies
* Calling Convention: DelayUs(2500)
* Conditions at Exit: Nothing else changes core timer period or uses it
* Date Started: 3/29/12
* Update History: 
*/
void DelayUs(unsigned int us) {
	unsigned int start;
	unsigned int ticks_to_wait;

	ticks_to_wait = us * core_ticks_in_us;
	WriteCoreTimer(0);	// clear timer
	start = ReadCoreTimer();
	while ((unsigned int) (ReadCoreTimer() - start) < ticks_to_wait) {};
}