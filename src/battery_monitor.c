#include <p32xxxx.h>
#include <plib.h>
#include "battery_monitor.h"
#include "delay.h"

static inline void OW_drive_low;();
static inline void OW_drive_high();
static inline char OW_read_bit();
static inline void OW_write_bit(char);

