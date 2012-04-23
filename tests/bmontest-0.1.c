#include <p32xxxx.h>
#include <plib.h>
#include "delay.h"
#include "battery_monitor.h"

#pragma config FNOSC = PRIPLL	// Primary Osc w/PLL (XT+,HS+,EC+PLL)
#pragma config POSCMOD = HS		// HS osc mode 
#pragma config FPLLMUL = MUL_20	// PLL Multiplier
#pragma config FPLLIDIV = DIV_2	// PLL Input Divider
#pragma config FPBDIV = DIV_2
#pragma config FPLLODIV = DIV_1
#pragma config FWDTEN = OFF

#define TEST_PIN BIT_0
#define TEST_PORT IOPORT_F

#define SYSTEM_FREQUENCY 80000000L

int main(void)
{
  int pbFreq;
  BATTMON_RESULT presence;
  char buffer[10] = {'a', 'b', 'c', 'd', 'e', 'f'};

  pbFreq = SYSTEMConfigPerformance(SYSTEM_FREQUENCY);
  DelayInit(SYSTEM_FREQUENCY);
  BatteryMonitorInit(TEST_PIN, TEST_PORT);
  while(1) {
    presence = BatteryMonitorWriteBytes(BATTERY_MONITOR_NET_SKIP_ADDR, 
                  BATTERY_MONITOR_PARAM_VCHG, buffer, 3);
    DelayUs(1000);
  }  
  pbFreq = SYSTEMConfigPerformance(SYSTEM_FREQUENCY);
  return 0;
}