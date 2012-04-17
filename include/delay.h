#ifndef DELAY_H
#define DELAY_H

#define DELAY_US_TO_CT_TICKS(core_timer) (core_timer / 1000000UL)    // uS to core_timer ticks 
#define DELAY_MS_TO_CT_TICKS(core_timer) (core_timer / 1000UL)

typedef unsigned int us_t;

void DelayInit(unsigned int clock_hz);
void DelayS(unsigned int s);
void DelayMs(unsigned int ms);
void DelayUs(unsigned int us);
us_t DelayUtilGetUs(void);
us_t DelayUtilElapsedUs(us_t start, us_t end);

#endif