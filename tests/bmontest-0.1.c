#include <p32xxxx.h>
#include <plib.h>
#include "delay.h"
#include "battery_monitor.h"

#pragma config FNOSC = PRIPLL	// Primary Osc w/PLL (XT+,HS+,EC+PLL)
#pragma config POSCMOD = HS		// HS osc mode 
#pragma config FPLLMUL = MUL_18	// PLL Multiplier
#pragma config FPLLIDIV = DIV_2	// PLL Input Divider
#pragma config FPBDIV = DIV_2
#pragma config FPLLODIV = DIV_1

#define TEST_PIN BIT_11
#define TEST_PORT IOPORT_B

#define SYSTEM_FREQUENCY 72000000L

int main(void)
{
  int pbFreq;
  unsigned char presence;
  char buffer[10] = {'a', 'b', 'c', 'd', 'e', 'f'};

  pbFreq = SYSTEMConfigPerformance(SYSTEM_FREQUENCY);
  DelayInit(SYSTEM_FREQUENCY);
  BatteryMonitorInit(TEST_PIN, IOPORT_B);
  while(1) {
    presence = BatteryMonitorWriteBytes(BATTERY_MONITOR_NET_SKIP_ADDR, 
                  BATTERY_MONITOR_PARAM_VCHG, buffer, 3);
    DelayUs(1000);
  }  
  pbFreq = SYSTEMConfigPerformance(SYSTEM_FREQUENCY);
  return 0;
}