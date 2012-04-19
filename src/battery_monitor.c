#include <p32xxxx.h>
#include <plib.h>
#include "battery_monitor.h"
#include "delay.h"

static unsigned int _bmon_pin;
static IoPortId _bmon_port;

static unsigned char BatteryMonitorReset();
static void BatteryMonitor_drive_low();
static void BatteryMonitor_drive_high();
static unsigned char BatteryMonitor_read_onewire();
static unsigned char BatteryMonitor_read_bit();
static void BatteryMonitor_write_bit(unsigned char bit);
static unsigned char BatteryMonitorReset();
static unsigned char BatteryMonitorReadByte();
static void BatteryMonitorWriteByte(unsigned char c);

/************************************************************************************************** 
  Function: 
    void BatteryMonitorInit(unsigned int batt_mon_pin, IoPortId batt_mon_port)
  
  Author(s): 
    mkobit
  
  Summary: 
    Configures pin as open drain and stores the parameters in here for future use
  
  Description: 
    Sets pin as open drain and stores the pin and port into the static variables (_bmon_port)
    and (_bmon_pin)
  
  Preconditions: 
    Pin and port provided not being used for anything else
  
  Parameters: 
    unsigned int batt_mon_pin - pin number of the battery monitor pin, should only be 1 pin
    IoPortId batt_mon_port - port of the battery monitor pin
  
  Returns: 
    None
  
  Example: 
    <code>
    BatteryMonitorInit(BIT_11, IOPORT_B);
    </code>
  
  Conditions at Exit: 
    Port (_bmon_port), pin (_bmon_pin) configured as open-drain pin
  
**************************************************************************************************/
void BatteryMonitorInit(unsigned int batt_mon_pin, IoPortId batt_mon_port) {
  _bmon_port = batt_mon_port;
  _bmon_pin = batt_mon_pin;
  switch(_bmon_port) {
    case IOPORT_A: mPORTAOpenDrainOpen(_bmon_pin); break;
    case IOPORT_B: mPORTBOpenDrainOpen(_bmon_pin); break;
    case IOPORT_C: mPORTCOpenDrainOpen(_bmon_pin); break;
    case IOPORT_D: mPORTDOpenDrainOpen(_bmon_pin); break;
    case IOPORT_E: mPORTEOpenDrainOpen(_bmon_pin); break;
    case IOPORT_F: mPORTFOpenDrainOpen(_bmon_pin); break;
    case IOPORT_G: mPORTGOpenDrainOpen(_bmon_pin); break;
  }
}

/************************************************************************************************** 
  Function: 
    BATTMON_RESULT BatteryMonitorReadBytes(unsigned char net_address_command, 
                    unsigned char start_addr, 
                    unsigned char *data, 
                    int nitems)
  
  Author(s):    mkobit
  
  Summary: 
    Reads (nitems) bytes into (data) from the battery monitor
  
  Description: 
    Reads several bytes from the battery monitor using the one wire protocol provided by other functions
    in this library
  
  Preconditions: 
    BatteryMonitorInit called prior
    One wire bus is idle
    (_bmon_pin) and (_bmon_port) have not been reconfigured or other use
  
  Parameters: 
    unsigned char net_address_command - command for how the bus master will interact with the slaves
    unsigned char start_addr - starting address for function
    char *data - pointer for data to be read into
    int nitems - number of bytes to read from the battery monitor
  
  Returns: 
    BATTMON_SUCCESS - If transaction was successful
    BATTMON_FAIL - If device not detected or transaction unsuccessful
  
  Example: 
    <code>
    BATTMON_RESULT presence
    presence = BatteryMonitorReadBytes(BATTERY_MONITOR_NET_SKIP_ADDR, 
                  BATTERY_MONITOR_PARAM_VCHG, buffer, 3)
    </code>
  
  Conditions at Exit: 
    One-wire bus idle
    
  Notes:
    4/17/12 - the only (net_address_command) that is currently supported is BATTERY_MONITOR_NET_SKIP_ADDR
      which means that only 1 device can be on the 1 wire bus
  
**************************************************************************************************/
BATTMON_RESULT BatteryMonitorReadBytes(unsigned char net_address_command, 
                    unsigned char start_addr, 
                    unsigned char *data, 
                    int nitems) {
  int i;
  BATTMON_RESULT presence_detect;
  unsigned char data_read;
  if (net_address_command == BATTERY_MONITOR_NET_SKIP_ADDR) {
    // saves time by not using net_address, assuming only 1 one-wire device on the bus
    presence_detect = BatteryMonitorReset();
    if (!presence_detect) return presence_detect;  // not detected, return a failure
    BatteryMonitorWriteByte(net_address_command);
    BatteryMonitorWriteByte(BATTERY_MONITOR_FUNC_READ_DATA);
    BatteryMonitorWriteByte(start_addr);
  }
  //else if (net_address_command == BATTERY_MONITOR_NET_RESUME {} *NOT IMPLEMENTED*
  //else if (net_address_command == BATTERY_MONITOR_NET_MATCH_ADDR) {} *NOT IMPLEMENTED*
  //else if (net_address_command == BATTERY_MONITOR_NET_SEARCH_ADDR) {} *NOT IMPLEMENTED*
  //else if (net_address_command == BATTERY_MONITOR_NET_READ_ADDR) {} *NOT IMPLEMENTED*
  else {
    return BATTMON_FAIL;  // not a valid net_address_command
  }

  for (i = 0; i < nitems; i++) {
    data_read = BatteryMonitorReadByte();
    *(data++) = data_read;
  }
  presence_detect = BatteryMonitorReset();  // reset at end of function
  return presence_detect;
}

/************************************************************************************************** 
  Function: 
    BATTMON_RESULT BatteryMonitorWriteBytes(unsigned char net_address_command, 
                    unsigned char start_addr, 
                    unsigned char *data, 
                    int nitems)
  
  Author(s):    mkobit
  
  Summary: 
    Writes several bytes to the battery monitor 
  
  Description: 
    Writes several bytes from the battery monitor using the one wire protocol provided by other functions
    in this library
  
  Preconditions: 
    BatteryMonitorInit called prior
    One wire bus is idle
    (_bmon_pin) and (_bmon_port) have not been reconfigured or other use
  
  Parameters: 
    unsigned char net_address_command - command for how the bus master will interact with the slaves
    unsigned char function - the feature of the battery monitor to execute 
    unsigned char start_addr - starting address for function
  
  Returns: 
    BATTMON_SUCCESS - If transaction was successful
    BATTMON_FAIL - If device not detected or transaction unsuccessful
  
  Example: 
    <code>
    BATTMON_RESULT presence
    presence = BatteryMonitorWriteBytes(BATTERY_MONITOR_NET_SKIP_ADDR, 
                  BATTERY_MONITOR_RAAC_MSB, buffer, 2)
    </code>
  
  Conditions at Exit: 
    One-wire bus idle
  
**************************************************************************************************/
BATTMON_RESULT BatteryMonitorWriteBytes(unsigned char net_address_command, 
                    unsigned char start_addr, 
                    unsigned char *data, 
                    int nitems) {
  int i;
  BATTMON_RESULT presence_detect;
  if (net_address_command == BATTERY_MONITOR_NET_SKIP_ADDR) {
    // saves time by not using net_address, assuming only 1 one-wire device on the bus
    presence_detect = BatteryMonitorReset();
    if (!presence_detect) return presence_detect;  // not detected, return a failure
    BatteryMonitorWriteByte(net_address_command);
    BatteryMonitorWriteByte(BATTERY_MONITOR_FUNC_WRITE_DATA);
    BatteryMonitorWriteByte(start_addr);
  }
  //else if (net_address_command == BATTERY_MONITOR_NET_RESUME {} *NOT IMPLEMENTED*
  //else if (net_address_command == BATTERY_MONITOR_NET_MATCH_ADDR) {} *NOT IMPLEMENTED*
  //else if (net_address_command == BATTERY_MONITOR_NET_SEARCH_ADDR) {} *NOT IMPLEMENTED*
  //else if (net_address_command == BATTERY_MONITOR_NET_READ_ADDR) {} *NOT IMPLEMENTED*
  else {
    return 0;  // not a valid net_address_command
  }
  for (i = 0; i < nitems; i++) {
    BatteryMonitorWriteByte(*(data++));
  }
  presence_detect = BatteryMonitorReset();  // reset at end of function
  return presence_detect;
}

/************************************************************************************************** 
  Function: 
    static BATTMON_RESULT BatteryMonitorReset()
  
  Author(s):    mkobit
  
  Summary: 
    Resets the one wire bus and detects the presence
  
  Description: 
    Describes the protocol that resets the one-wire device which is used when starting a transaction on the bus
    Static function, used by internal library
  
  Preconditions: 
    BatteryMonitorInit called prior
    One wire bus is idle
    (_bmon_pin) and (_bmon_port) have not been reconfigured or other use
  
  Parameters: 
    None
  
  Returns: 
    BATTMON_SUCCESS - If device detected
    BATTMON_FAIL - If device not detected
  
  Example: 
    <code>
    BATTMON_RESULT presence
    BATTMON_RESULT presence = BatteryMonitorReset()
    // start transaction
    </code>
  
  Conditions at Exit: 
    One-wire bus responds and gets put into a state where it waits for the net address command
  
**************************************************************************************************/
static BATTMON_RESULT BatteryMonitorReset() {
  BATTMON_RESULT presence_detected;
  
  BatteryMonitor_drive_low();
  DelayUs(480);
  BatteryMonitor_drive_high();  // release bus
  DelayUs(100);
  // Read presence bit, 0 if present 1 if not present so value is inverted for logical clarity
  presence_detected = BatteryMonitor_read_onewire() == 0 ? BATTMON_SUCCESS : BATTMON_FAIL;
  DelayUs(380);
  BatteryMonitor_drive_high();  // release bus
  return presence_detected;
}

/************************************************************************************************** 
  Function: 
    static unsigned char BatteryMonitorReadByte()
  
  Author(s):
    mkobit
  
  Summary: 
    Reads an entire byte off of the one-wire bus
  
  Description: 
    Describes protocol to read a single byte off of the device
    Static function, used by internal library
  
  Preconditions: 
    BatteryMonitorInit called prior
    One wire bus is idle
    (_bmon_pin) and (_bmon_port) have not been reconfigured or other use
  
  Parameters: 
    None
  
  Returns: 
    unsigned char result - byte read from one-wire bus
  
  Example: 
    <code>
    unsigned char c
    c = BatteryMonitorReadByte()
    </code>
  
  Conditions at Exit: 
    One-wire bus still in transaction, waiting for next action
  
**************************************************************************************************/
static unsigned char BatteryMonitorReadByte() {
  unsigned char loop;
  unsigned char result = 0;
  
  for (loop = 0; loop < 8; loop++) {
    result >>= 1;
    if (BatteryMonitor_read_bit()) {
      result |= MSBIT_MASK;  // if bit read is 1, or in a 1
    }
  }
  return result;
}

/************************************************************************************************** 
  Function: 
    static void BatteryMonitorWriteByte(unsigned char c)
  
  Author(s):    mkobit
  
  Summary: 
    Writes an entire byte to the one wire battery monitor device
  
  Description: 
    Describes protocol to write a single byte to the device
    Static function, used by internal library
  
  Preconditions: 
    BatteryMonitorInit called prior
    One wire bus is idle
    (_bmon_pin) and (_bmon_port) have not been reconfigured or other use
  
  Parameters: 
    unsigned char c - character to be written to the one wire bus, least significant bit first
  
  Returns: 
    void
  
  Example: 
    <code>
    BatteryMonitorWriteByte(0xCD)
    </code>
  
  Conditions at Exit: 
    One-wire bus still in transaction, waiting for next action
  
**************************************************************************************************/
static void BatteryMonitorWriteByte(unsigned char c) {
  unsigned char loop;
  
  for (loop = 0; loop < 8; loop++) {
    BatteryMonitor_write_bit(c & LSBIT_MASK);
    c >>= 1;
  }
}

/************************************************************************************************** 
  Function: 
    static void BatteryMonitor_drive_low()
  
  Author(s):    mkobit
  
  Summary: 
    Configures the 1-wire port pin as an output and drives the port pin low

  Description: 
    Static function, used by internal library
  
  Preconditions: 
    BatteryMonitorInit called prior
    (_bmon_pin) and (_bmon_port) have not been reconfigured or other use
  
  Parameters: 
    None
  
  Returns: 
    void
  
  Example: 
    <code>
    BatteryMonitor_drive_low()
    </code>
  
  Conditions at Exit: 
    (_bmon_pin) and (_bmon_port) configured as output, driving low
  
**************************************************************************************************/
static void BatteryMonitor_drive_low() {
  PORTSetPinsDigitalOut(_bmon_port, _bmon_pin);
  PORTClearBits(_bmon_port, _bmon_pin);
}

/************************************************************************************************** 
  Function: 
    static void BatteryMonitor_drive_high()
  
  Author(s):    mkobit
  
  Summary: 
    Configures the 1-wire port pin as an output and drives the port pin high
  
  Description: 
    Static function, used by internal library
  
  Preconditions: 
    BatteryMonitorInit called prior
    (_bmon_pin) and (_bmon_port) have not been reconfigured or other use
  
  Parameters: 
    None
  
  Returns: 
    void
  
  Example: 
    <code>
    BatteryMonitor_drive_high()
    </code>
  
  Conditions at Exit: 
    (_bmon_pin) and (_bmon_port) configured as output, driving high
  
**************************************************************************************************/
static void BatteryMonitor_drive_high() {
  PORTSetPinsDigitalOut(_bmon_port, _bmon_pin);
  PORTSetBits(_bmon_port, _bmon_pin);
}

/************************************************************************************************** 
  Function: 
    static unsigned char BatteryMonitor_read_onewire()
  
  Author(s):    mkobit
  
  Summary: 
    Configures one wire interface as input and samples the line
  
  Description: 
    Static function, used by internal library
  
  Preconditions: 
    BatteryMonitorInit called prior
    (_bmon_pin) and (_bmon_port) have not been reconfigured or other use
  
  Parameters: 
    None
  
  Returns: 
    unsigned char read_data - 1 if pin read high, 0 if pin read low
  
  Example: 
    <code>
    unsigned char bit
    bit = BatteryMonitor_read_onewire()
    </code>
  
  Conditions at Exit: 
    (_bmon_pin) and (_bmon_port) configured as input
  
**************************************************************************************************/
static unsigned char BatteryMonitor_read_onewire(void) {
  unsigned char read_data = 0;
  
  PORTSetPinsDigitalIn(_bmon_port, _bmon_pin);
  if(PORTReadBits(_bmon_port, _bmon_pin)) {
    read_data = 1;
  } else {
    read_data = 0;
  }
  return read_data;
}

/************************************************************************************************** 
  Function: 
    static unsigned char BatteryMonitor_read_bit()
  
  Author(s):    mkobit
  
  Summary: 
    Describes read protocol for one-wire device
  
  Description: 
    Static function, used by internal library
  
  Preconditions: 
    BatteryMonitorInit called prior
    (_bmon_pin) and (_bmon_port) have not been reconfigured or other use
  
  Parameters: 
    void
  
  Returns: 
    unsigned char result - 0 or 1 for what was read on the data line
  
  Example: 
    <code>
    unsigned char c
    c = BatteryMonitor_read_bit()
    </code>
  
  Conditions at Exit: 
    BatteryMonitor_drive_high called to leave pin as an output and high
  
**************************************************************************************************/
static unsigned char BatteryMonitor_read_bit() {
  unsigned char result;
  
  BatteryMonitor_drive_low();
  DelayUs(3);  // minimum 1 us
  BatteryMonitor_drive_high();
  DelayUs(20);  // tRDV - max 15 us
  result = BatteryMonitor_read_onewire();
  BatteryMonitor_drive_high();
  DelayUs(57);  // time slot (min,max) = (60, 120) us
  return result;
}

/************************************************************************************************** 
  Function: 
    static void BatteryMonitor_write_bit(unsigned char bit)
  
  Author(s):    mkobit
  
  Summary: 
    Describes write protocol for one-wire device
  
  Description: 
    Protocol for writing a 1 or 0 on the one-wire bus
    Static function, used by internal library
  
  Preconditions: 
    BatteryMonitorInit called prior
    (_bmon_pin) and (_bmon_port) have not been reconfigured or other use
  
  Parameters: 
    unsigned char bit - 0 or !0, determines which protocol to use to write a 1 or 0 to one-wire bus
  
  Returns: 
    void
  
  Example: 
    <code>
    char c = 0x6B
    BatteryMonitor_write_bit(c & LSBIT_MASK)
    </code>
  
  Conditions at Exit: 
    BatteryMonitor_drive_high called to leave pin as an output and high
  
**************************************************************************************************/
static void BatteryMonitor_write_bit(unsigned char bit) {
  if (bit) {
    // write 1 bit
    BatteryMonitor_drive_low();
    DelayUs(7);  // Write-1 Low Time, (min, max) = (1, 15) us
    BatteryMonitor_drive_high();
    DelayUs(75);   //recovery time and slot time, min recovery is 1 us
  } else {
    // write 0 bit
    BatteryMonitor_drive_low();
    DelayUs(80);  // Write-0 Low Time, (min, max) = (60, 120) us
    BatteryMonitor_drive_high();
    DelayUs(2);   //recovery time, min is 1 us    
  }
}