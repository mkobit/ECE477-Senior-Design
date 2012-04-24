#ifndef DELAY_H
#define DELAY_H

#define DELAY_US_TO_CT_TICKS(core_timer) (core_timer / 1000000UL)    // uS to core_timer ticks 
#define DELAY_MS_TO_CT_TICKS(core_timer) (core_timer / 1000UL)

typedef unsigned int us_t;

void DelayInit(UINT clock_hz);
void DelayS(UINT s);
void DelayMs(UINT ms);
void DelayUs(UINT us);
us_t DelayUtilGetUs(void);
us_t DelayUtilElapsedUs(us_t start, us_t end);

#endif