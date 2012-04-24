#include <p32xxxx.h>
#include <plib.h>
#include "delay.h"

static UINT _core_hz;
static UINT _core_ticks_in_us;  // stored locally so that these delays will be more accurate
static UINT _core_ticks_in_ms;

/************************************************************************************************** 
  Function: 
    void DelayInit(const UINT clock_hz)
  
  Author(s): 
    mkobit
  
  Summary: 
    Sets internal variables to be used with other delays
  
  Description: 
    Calculates how many core timer clock ticks are in a microseconds and a millisecond to be used later with delays
  
  Preconditions: 
    Core timer configured
  
  Parameters: 
    const UINT clock_hz - clock speed of microcontroller
  
  Returns: 
    void
  
  Example: 
    <code>
    DelayInit(80000000L)
    </code>
  
  Conditions at Exit: 
    Core timer reset, internal static variables set
  
**************************************************************************************************/
void DelayInit(const UINT clock_hz) {
  _core_hz = clock_hz / 2;  // runs at half-speed of system clock
  _core_ticks_in_us = DELAY_US_TO_CT_TICKS(_core_hz);
  _core_ticks_in_ms = DELAY_MS_TO_CT_TICKS(_core_hz);
  WriteCoreTimer(0);
}

/************************************************************************************************** 
  Function: 
    void DelayS(const UINT s)
  
  Author(s): 
    mkobit
  
  Summary: 
    Delays for s seconds
  
  Description: 
    Blocking routine, sets core timer to 0, and calculates how many core timer ticks it needs to wait to delay for s seconds and delays
  
  Preconditions: 
    DelayInit called previously, core timer configured
  
  Parameters: 
    const UINT s - seconds to delay: should not be an excessively high number
  
  Returns: 
    void
  
  Example: 
    DelayS(2)
  
  Conditions at Exit: 
    Core timer overwritten to 0 before delay, could be any number after
  
**************************************************************************************************/
void DelayS(const UINT s) {
  UINT start;
  UINT ticks_to_wait;
  UINT core_time;

  ticks_to_wait = s * _core_hz;
  WriteCoreTimer(0);  // clear timer
  start = ReadCoreTimer();
  do {
    core_time = ReadCoreTimer();
  } while((core_time - start) < ticks_to_wait);
}

/************************************************************************************************** 
  Function: 
    void DelayMs(const UINT ms)
  
  Author(s): 
    mkobit
  
  Summary: 
    Delays for ms seconds
  
  Description: 
    Blocking routine, calculates how many core timer ticks required to delay for ms milliseconds and delays
  
  Preconditions: 
    DelayInit called previously, core timer configured
  
  Parameters: 
    const UINT ms - milliseconds to delay
  
  Returns: 
    void
  
  Example: 
    <code>
    DelayMs(250)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
void DelayMs(const UINT ms) {
  UINT start;
  UINT ticks_to_wait;
  UINT core_time;

  ticks_to_wait = ms * _core_ticks_in_ms;
  //WriteCoreTimer(0);  // clear timer
  start = ReadCoreTimer();
  do {
    core_time = ReadCoreTimer();
  } while((core_time - start) < ticks_to_wait);
  //while ((UINT) (ReadCoreTimer() - start) < ticks_to_wait) {};
}

/************************************************************************************************** 
  Function: 
    void DelayUs(const UINT us)
  
  Author(s): 
    mkobit
  
  Summary: 
    Delays for us microseconds
  
  Description: 
    Blocking routine, calculates how many core timer ticks required to delay for us microseconds and delays
  
  Preconditions: 
    DelayInit called previously, core timer configured
  
  Parameters: 
    const UINT us - milliseconds to delay
  
  Returns: 
    void
  
  Example: 
    <code>
    DelayUs(800)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
void DelayUs(const UINT us) {
  UINT start;
  UINT ticks_to_wait;
  UINT core_time;

  ticks_to_wait = us * _core_ticks_in_us;
  //WriteCoreTimer(0);  // clear timer
  start = ReadCoreTimer();
  do {
    core_time = ReadCoreTimer();
  } while((core_time - start) < ticks_to_wait);
}

/************************************************************************************************** 
  Function: 
    us_t DelayUtilGetUs()
  
  Author(s): 
    mkobit
  
  Summary: 
    Gets a time in microseconds of what the core timer currently is at
  
  Description: 
    Gets the micrseconds, used in DelayUtilElapsedUs to determine how many microseconds have passed since a few operations
  
  Preconditions: 
    DelayInit called previously, core timer configured
  
  Parameters: 
    void
  
  Returns: 
    us_t currentUs - current microsecond count of the core timer
  
  Example: 
    <code>
    us = DelayUtilGetUs()
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
us_t DelayUtilGetUs(void) {
  us_t currentUs;
  currentUs = ReadCoreTimer() / _core_ticks_in_us;
  return currentUs;
}

/************************************************************************************************** 
  Function: 
    us_t DelayUtilElapsedUs(const us_t start, const us_t end)
  
  Author(s): 
    mkobit
  
  Summary: 
    Calculates the elapsed time in from two calls of DelayUtilGetUs
  
  Description: 
    Simply subtracts the start from the end using the datatype us_t
  
  Preconditions: 
    Core timer must not have been reset in between calls of DelayUtilGetUs. DelayS must not have been called because it overwrites the core timer
    Must not have been an excessive amount of time in between start and end that results in an overlap
  
  Parameters: 
    const us_t start - start time in microseconds
    const us_t end - end time in microseconds
  
  Returns: 
    unsigned int elapsedUs - us between start and end
  
  Example: 
    <code>
    us_t us1 = DelayUtilGetUs()
    us_t us2 = DelayUtilGetUs()
    elapsed = DelayUtilElapsedUs(us1, us2)
    </code>
  
  Conditions at Exit: 
    None
  
**************************************************************************************************/
us_t DelayUtilElapsedUs(const us_t start, const us_t end) {
  return end - start;
}