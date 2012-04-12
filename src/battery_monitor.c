#include <p32xxxx.h>
#include <plib.h>
#include "battery_monitor.h"
#include "delay.h"

static int _bmon_pin, _bmon_port;

static unsigned char BatteryMonitorReset();
static void BatteryMonitor_drive_low();
static void BatteryMonitor_drive_high();
static unsigned char BatteryMonitor_read_bit();
static void BatteryMonitor_write_bit(unsigned char bit);
static unsigned char BatteryMonitorReset();
static unsigned char BatteryMonitorReadByte();
static void BatteryMonitorWriteByte(unsigned char c);

/* BatteryMonitorInit
* Parameters: 
	int batt_mon_pin - pin number of the battery monitor pin
	int batt_mon_port - port of the battery monitor pin
* Author(s): mkobit
* Called by: Program that wants to use the battery monitor. This will be called before any other battery monitor usage.
* Purpose/Function Performed: Configures pin as open drain and stores the parameters in here for future use.
* Calling Convention: Delay library is expected to be set up before hand
	BatteryMonitorInit(IOPORT_F, BIT_2)
* Conditions at Exit: pin is configured as an open drain pin and configured as an output and should have released the bus. Resets the bus to check and see if device is connected.
	return int, if 1-wire device detected, returns 1, if not, returns 0
* Date Started: 
* Update History: 
*/
void BatteryMonitorInit(int batt_mon_pin, int batt_mon_port) {
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

/* BatteryMonitorReadBytes
* Parameters:
	unsigned char net_address_command - command for how the bus master will interact with the slaves
	unsigned char start_addr - starting address for function
	char *data - pointer for data to be read into
	int nitems - number of bytes to read from the battery monitor
* Author(s): mkobit
* Called by: main program
* Purpose/Function Performed: reads several bytes from the battery monitor 
* Calling Convention: BatteryMonitorReadBytes(BATTERY_MONITOR_SKIP_NET_ADDR, BATTERY_MONITOR_RAAC_MSB, read_data, 2)
* Conditions at Exit: resets the slave devices and goes into an idle high
	return unsigned char - 1 for a device was detected, 0 for no device detected
* Date Started: 
* Update History: 
*/
unsigned char BatteryMonitorReadBytes(unsigned char net_address_command, 
										unsigned char start_addr, 
										char *data, 
										int nitems) {
	int i;
	unsigned char presence_detect;
	unsigned char data_read;
	if (net_address_command == BATTERY_MONITOR_SKIP_NET_ADDR) {
		// saves time by not using net_address, assuming only 1 one-wire device on the bus
		presence_detect = BatteryMonitorReset();
		if (!presence_detect) return presence_detect;	// not detected, return a failure
		BatteryMonitorWriteByte(net_address_command);
		BatteryMonitorWriteByte(BATTERY_MONITOR_READ_DATA);
		BatteryMonitorWriteByte(start_addr);
	}
	//else if (net_address_command == BATTERY_MONITOR_RESUME {} *NOT IMPLEMENTED*
	//else if (net_address_command == BATTERY_MONITOR_MATCH_NET_ADDR) {} *NOT IMPLEMENTED*
	//else if (net_address_command == BATTERY_MONITOR_SEARCH_NET_ADDR) {} *NOT IMPLEMENTED*
	//else if (net_address_command == BATTERY_MONITOR_READ_NET_ADDR) {} *NOT IMPLEMENTED*
	else {
		return 0;	// not a valid net_address_command
	}

	for (i = 0; i < nitems; i++) {
		data_read = BatteryMonitorReadByte();
		*(data++) = data_read;
	}
	presence_detect = BatteryMonitorReset();	// reset at end of function
	return presence_detect;
}


/* BatteryMonitorWriteBytes
* Parameters:
	unsigned char net_address_command - command for how the bus master will interact with the slaves
	unsigned char function - the feature of the battery monitor to execute 
	unsigned char start_addr - starting address for function
	char *data - pointer for write data
	int nitems - number of bytes to written to the battery monitor
* Author(s): mkobit
* Called by: main program
* Purpose/Function Performed: writes several bytes to the battery monitor
* Calling Convention: BatteryMonitorWriteBytes(BATTERY_MONITOR_SKIP_NET_ADDR, BATTERY_MONITOR_PARAM_AC, write_data, 2)
* Conditions at Exit: resets the slave devices and goes into an idle high
	return unsigned char - 1 for a device was detected, 0 for no device detected
* Date Started: 
* Update History: 
*/
unsigned char BatteryMonitorWriteBytes(unsigned char net_address, 
										unsigned char start_addr, 
										char *data, 
										int nitems) {
	int i;
	unsigned char presence_detect;
	if (net_address_command == BATTERY_MONITOR_SKIP_NET_ADDR) {
		// saves time by not using net_address, assuming only 1 one-wire device on the bus
		presence_detect = BatteryMonitorReset();
		if (!presence_detect) return presence_detect;	// not detected, return a failure
		BatteryMonitorWriteByte(net_address_command);
		BatteryMonitorWriteByte(BATTERY_MONITOR_WRITE_DATA);
		BatteryMonitorWriteByte(start_addr);
	}
	//else if (net_address_command == BATTERY_MONITOR_RESUME {} *NOT IMPLEMENTED*
	//else if (net_address_command == BATTERY_MONITOR_MATCH_NET_ADDR) {} *NOT IMPLEMENTED*
	//else if (net_address_command == BATTERY_MONITOR_SEARCH_NET_ADDR) {} *NOT IMPLEMENTED*
	//else if (net_address_command == BATTERY_MONITOR_READ_NET_ADDR) {} *NOT IMPLEMENTED*
	else {
		return 0;	// not a valid net_address_command
	}
	for (i = 0; i < nitems; i++) {
		BatteryMonitorWriteByte(*(data++));
	}
	presence_detect = BatteryMonitorReset();	// reset at end of function
	return presence_detect;
}

/* BatteryMonitorReset
* Author(s): mkobit
* Called by: internal functions
* Purpose/Function Performed: resets the one wire bus and detects the presence, also used when starting a transaction on the bus
* Calling Convention: unsigned char detected = BatteryMonitorReset()
* Conditions at Exit: bus is released
	return unsigned char - 1 for a device was detected, 0 for no device detected
* Date Started: 
* Update History: 
*/
static unsigned char BatteryMonitorReset() {
	unsigned char presence_detected;
	
	BatteryMonitor_drive_low();
	DelayUs(480);
	BatteryMonitor_drive_high();	// release bus
	// TODO
	DelayUs(100);
	presence_detected = !BatteryMonitor_read_onewire();	// read presence bit, 0 if present 1 if not present so value is inverted for logical clarity
	DelayUs(380);
	BatteryMonitor_drive_high();	// release bus
	return presence_detected;
}

/* BatteryMonitorReadByte
* Author(s): mkobit
* Called by: internal functions
* Purpose/Function Performed: reads an entire byte off of the one-wire
* Calling Convention: BatteryMonitorReadByte()
* Conditions at Exit: bus is released (from read bit, no need to call again
	return unsigned char - byte read from one-wire bus
* Date Started: 
* Update History: 
*/
static unsigned char BatteryMonitorReadByte() {
	unsigned char loop;
	unsigned char result = 0;
	
	for (loop = 0; loop < 8; loop++) {
		result >>= 1;
		if (BatteryMonitor_read_bit()) {
			result |= MSBIT_MASK;	// if bit read is 1, or in a 1
		}
	}
	return result;
}

/* BatteryMonitorWriteByte
* Parameters: 
	unsigned char c - character to be written to the one wire bus, least significant bit first
* Author(s): mkobit
* Called by: internal functions
* Purpose/Function Performed: writes an entire byte to the one wire device
* Calling Convention: BatteryMonitorWriteByte(BATTERY_MONITOR_PARAM_VCHG)
* Conditions at Exit: 
* Date Started: 
* Update History: 
*/
static void BatteryMonitorWriteByte(unsigned char c) {
	unsigned char loop;
	
	for (loop = 0; loop < 8; loop++) {
		BatteryMonitor_write_bit(c & LSBIT_MASK);
		c >>= 1;
	}
}

/* BatteryMonitor_drive_low
* Author(s): mkobit
* Called by: internal functions 
* Purpose/Function Performed: Configures the 1-wire port pin as an output and drives the port pin low
* Calling Convention: BatteryMonitor_drive_low()
* Conditions at Exit: 1-wire pin is a low output
* Date Started: 
* Update History: 
*/
static void BatteryMonitor_drive_low() {
	PORTSetPinsDigitalOut(_bmon_port, _bmon_pin);
	PORTClearBits(_bmon_port, _bmon_pin);
}

/* BatteryMonitor_drive_high
* Author(s): mkobit
* Called by: internal functions
* Purpose/Function Performed: Configures the 1-wire port pin as an output and drives the port pin high
* Calling Convention: BatteryMonitor_drive_high()
* Conditions at Exit: 1-wire pin is a high output
* Date Started: 
* Update History: 
*/
static void BatteryMonitor_drive_high() {
	PORTSetPinsDigitalOut(_bmon_port, _bmon_pin);
	PORTSetBits(_bmon_port, _bmon_pin);
}

/* BatteryMonitor_read_onewire
* Author(s): mkobit
* Called by: internal functions
* Purpose/Function Performed: configures one wire as input and samples the line. Returns if pin was high or not
* Calling Convention: unsigned char c = BatteryMonitor_read_onewire()
* Conditions at Exit: pin configured as input
	return unsigned char - 1 if pin read high, 0 if pin read low
* Date Started: 
* Update History: 
*/
static unsigned char BatteryMonitor_read_onewire() {
	unsigned char read_data = 0;
	
	PORTSetPinsDigitalIn(_bmon_port, _bmon_pin);
	if(PORTReadBits(_bmon_port, _bmon_pin)) {
		read_data = 1;
	} else {
		read_data = 0;
	}
	return read_data;
}

/* BatteryMonitor_read_bit
* Author(s): mkobit
* Called by: internal functions
* Purpose/Function Performed: read protocol for one-wire device
* Calling Convention: unisgned char bit = BatteryMonitor_read_bit()
* Conditions at Exit: 
	return unsigned char - 0 or 1 for what was read on the data line
* Date Started: 
* Update History: 
*/
static unsigned char BatteryMonitor_read_bit() {
	unsigned char result;
	
	BatteryMonitor_drive_low();
	DelayUs(3);	// minimum 1 us
	BatteryMonitor_drive_high();
	DelayUs(20);	// tRDV - max 15 us
	result = BatteryMonitor_read_onewire();
	BatteryMonitor_drive_high();
	DelayUs(57);	// time slot (min,max) = (60, 120) us
	return result;
}

/* BatteryMonitor_write_bit
* Parameters: 
	unsigned char bit - 0 or !0, determines which protocol to use to write a 1 or 0 to one-wire
* Author(s): mkobit
* Called by: internal functions
* Purpose/Function Performed: write a bit to the battery monitor using the one-wire protocl
* Calling Convention: BatteryMonitor_write_bit(input & MSBIT_MASK)
* Conditions at Exit: pin conifugred using BatteryMonitor_drive_high()
* Date Started: 
* Update History: 
*/
static void BatteryMonitor_write_bit(unsigned char bit) {
	if (bit) {
		// write 1 bit
		BatteryMonitor_drive_low();
		DelayUs(7);	// Write-1 Low Time, (min, max) = (1, 15) us
		BatteryMonitor_drive_high();
		DelayUs(75); 	//recovery time and slot time, min recovery is 1 us
	} else {
		// write 0 bit
		BatteryMonitor_drive_low();
		DelayUs(80);	// Write-0 Low Time, (min, max) = (60, 120) us
		BatteryMonitor_drive_high();
		DelayUs(2); 	//recovery time, min is 1 us		
	}
}