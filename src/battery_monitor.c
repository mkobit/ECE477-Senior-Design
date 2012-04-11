#include <p32xxxx.h>
#include <plib.h>
#include "battery_monitor.h"
#include "delay.h"

static int _bmon_pin, _bmon_port;

static unsigned char BatteryMonitorReset();
static void BatteryMonitor_drive_low();
static void BatteryMonitor_drive_high();
static unsigned int BatteryMonitor_read_bit();
static void BatteryMonitor_write_bit(char addr);

/* BatteryMonitorInit
* Parameters: 
	int batt_mon_pin - pin number of the battery monitor pin
	int batt_mon_port - port of the battery monitor pin
* Author(s): mkobit
* Called by: Program that wants to use the battery monitor. This will be called before any 
	other battery monitor usage.
* Purpose/Function Performed: Configures pin as open drain and stores the parameters in here for future use.
* Calling Convention: BatteryMonitorInit(IOPORT_F, BIT_2)
* Conditions at Exit: pin is configured as an open drain pin and configured as an output and should have released the bus. Resets the bus to check and see if device is connected.
	return int, if 1-wire device detected, returns 1, if not, returns 0
* Date Started: 
* Update History: 
*/
unsigned char BatteryMonitorInit(int batt_mon_pin, int batt_mon_port) {
	unsigned char presence_detected;
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
	// TODO detection of device
	presence_detected = BatteryMonitorReset();
	return presence_detected;
}

/* BatteryMonitorReset
* Author(s): mkobit
* Called by: 
* Purpose/Function Performed: 
* Calling Convention: 
* Conditions at Exit: 
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
	presence_detected = !BatteryMonitor_read_bit();	// read presence bit, 0 if present 1 if not present so value is inverted for logical clarity
	DelayUs(380);
	BatteryMonitor_drive_high();	// release bus
	return presence_detected;
}

/* BatteryMonitorReadByte
* Parameters: 
* Author(s): 
* Called by: 
* Purpose/Function Performed: 
* Calling Convention: 
* Conditions at Exit: 
* Date Started: 
* Update History: 
*/
unsigned char BatteryMonitorReadByte(char addr) {
	// TODO
	unsigned char loop;
	unsigned char result = 0;
	
	for (loop = 0; loop < 8; loop++) {
		result >>= 1;
		if (OW_read_bit()) {
			result |= 0x80;	// if bit read is 1, or in a 1
		}
	}
}

/* BatteryMonitorWriteByte
* Parameters: 
* Author(s): 
* Called by: 
* Purpose/Function Performed: 
* Calling Convention: 
* Conditions at Exit: 
	return char, read character from battery monitor
* Date Started: 
* Update History: 
*/
void BatteryMonitorWriteByte(char addr, char c) {
	// TODO
}

/* BatteryMonitor_drive_low
* Author(s): mkobit
* Called by: internal functions 
* Purpose/Function Performed: Configures the 1-wire port pin as an output and drives the port pin low
* Calling Convention: 
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
* Called by: 
* Purpose/Function Performed: Configures the 1-wire port pin as an output and drives the port pin high
* Calling Convention: 
* Conditions at Exit: 1-wire pin is a high output
* Date Started: 
* Update History: 
*/
static void BatteryMonitor_drive_high() {
	PORTSetPinsDigitalOut(_bmon_port, _bmon_pin);
	PORTSetBits(_bmon_port, _bmon_pin);
}

/* BatteryMonitor_read_bit
* Author(s): mkobit
* Called by: 
* Purpose/Function Performed: Configures the 1-wire port pin as an input and reads the port pin
* Calling Convention: 
* Conditions at Exit: 
* Date Started: 
* Update History: 
*/
static unsigned int BatteryMonitor_read_bit() {
	PORTSetPinsDigitalIn(_bmon_port, _bmon_pin);
	return PORTReadBits(_bmon_port, _bmon_pin);
}

/* BatteryMonitor_write_bit
* Parameters: 
* Author(s): 
* Called by: 
* Purpose/Function Performed: 
* Calling Convention: 
* Conditions at Exit: 
* Date Started: 
* Update History: 
*/
static void BatteryMonitor_write_bit(char addr) {
}