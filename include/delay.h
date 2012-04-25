#ifndef DELAY_H
#define DELAY_H

#define DELAY_US_TO_CT_TICKS(core_timer) (core_timer / 1000000UL)    // uS to core_timer ticks 
#define DELAY_MS_TO_CT_TICKS(core_timer) (core_timer / 1000UL)

typedef unsigned int us_t;

void DelayInit(const UINT pbFreq);
void DelayS(const UINT s);
void DelayMs(const UINT ms);
void DelayUs(const UINT us);
us_t DelayUtilGetUs(void);
us_t DelayUtilElapsedUs(const us_t end, const us_t start);

#endif