#include <p32xxxx.h>
#include <plib.h>
#include "battery_monitor.h"
#include "delay.h"

static inline void BatteryMonitor_drive_low;();
static inline void BatteryMonitor_drive_high();
static inline char BatteryMonitor_read_bit();
static inline void BatteryMonitor_write_bit(char addr);

