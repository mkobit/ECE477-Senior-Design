#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

// Net Address Commands
#define BATTERY_MONITOR_NET_READ_ADDR  0x33  // allows the bus master to read the DS2781’s 1-Wire net address
#define BATTERY_MONITOR_NET_MATCH_ADDR  0x55  // allows the bus master to specifically address one DS2781 on the 1-Wire bus
#define BATTERY_MONITOR_NET_SKIP_ADDR  0xCC  // command saves time when there is only one DS2781 on the bus by allowing the bus master to issue a function command without specifying the address of the slave
#define BATTERY_MONITOR_NET_SEARCH_ADDR  0xF0  // command allows the bus master to use a process of elimination to identify the 1-Wire net addresses of all slave devices on the bus
#define BATTERY_MONITOR_NET_RESUME      0xA5  // command increases data throughput in multidrop environments where the DS2781 needs to be accessed several times

// Function Commands
#define BATTERY_MONITOR_FUNC_READ_DATA    0x69  // Read Data [69h, XX]. This command reads data from the DS2781 starting at memory address XX. The Read Data command can be terminated by the bus master with a reset pulse at any bit boundary.
#define BATTERY_MONITOR_FUNC_WRITE_DATA    0x6C  // Write Data [6Ch, XX]. This command writes data to the DS2781 starting at memory address XX. The LSb of the data to be stored at address XX can be written immediately after the MSb of address has been entered.
#define BATTERY_MONITOR_FUNC_COPY_DATA    0x48
#define BATTERY_MONITOR_FUNC_RECALL_DATA    0xB8  // Recall Data [B8h, XX]. This command recalls the contents of the EEPROM cells to the EEPROM shadow memory for the EEPROM block containing address XX.
#define BATTERY_MONITOR_FUNC_LOCK      0x6A

// Battery monitor registers
#define BATTERY_MONITOR_STATUS     0x01  // Status Register
#define BATTERY_MONITOR_RAAC_MSB  0x02  // Remaining Active Absolute Capacity MSB
#define BATTERY_MONITOR_RAAC_LSB  0x03  // Remaining Active Absolute Capacity LSB
#define BATTERY_MONITOR_RSAC_MSB  0x04  // Remaining Standby Absolute Capacity MSB
#define BATTERY_MONITOR_RSAC_LSB  0x05  // Remaining Standby Absolute Capacity LSB
#define BATTERY_MONITOR_RARC    0x06  // Remaining Active Relative Capacity
#define BATTERY_MONITOR_RSRC    0x07  // Remaining Standby Relative Capacity
#define BATTERY_MONITOR_IAVG_MSB  0x08  // Average Current Register MSB
#define BATTERY_MONITOR_IAVG_LSB  0x09  // Average Current Register LSB
#define BATTERY_MONITOR_TEMP_MSB  0x0A  // Temperature Register MSB
#define BATTERY_MONITOR_TEMP_LSB  0x0B  // Temperature Register LSB
#define BATTERY_MONITOR_VOLT_MSB  0x0C  // Voltage Register MSB
#define BATTERY_MONITOR_VOLT    0x0D  // Voltage Register LSB
#define BATTERY_MONITOR_CURRENT_MSB  0x0E  // Current Register MSB
#define BATTERY_MONITOR_CURRENT_LSB  0x0F  // Current Register LSB
#define BATTERY_MONITOR_ACR_MSB    0x10  // Accumulated Current Register MSB
#define BATTERY_MONITOR_ACR_LSB    0x11  // Accumulated Current Register MSB
#define BATTERY_MONITOR_ACRL_MSB  0x12  // Low Accumulated Current Register MSB
#define BATTERY_MONITOR_ACRL_LSB  0x13  // Low Accumulated Current Register LSB
#define BATTERY_MONITOR_AS      0x14  // Age Scalar
#define BATTERY_MONITOR_SFR      0x15  // Special Feature Register
#define BATTERY_MONITOR_FULL_MSB  0x16  // Full Capacity MSB
#define BATTERY_MONITOR_FULL_LSB  0x17  // Full Capacity LSB
#define BATTERY_MONITOR_AE_MSB    0x18  // Active Empty MSB
#define BATTERY_MONITOR_AE_LSB    0x19  // Active Empty LSB
#define BATTERY_MONITOR_SE_MSB    0x1A  // Standby Empty MSB 
#define BATTERY_MONITOR_SE_LSB    0x1B  // Standby Empty LSB 

#define BATTERY_MONITOR_EEPROM_REGISTER  0x1F  // EEPROM Register
#define BATTERY_MONITOR_USER_EEPROM_LOW  0x20
#define BATTERY_MONITOR_USER_EEPROM_HGH  0x2F

// Application operating parameter addresses
#define BATTERY_MONITOR_PARAM_CONTROL  0x60  // Control Register
#define BATTERY_MONITOR_PARAM_AB    0x61  // Accumulation Bias
#define BATTERY_MONITOR_PARAM_AC_MSB    0x62  // Aging Capacity MSB
#define BATTERY_MONITOR_PARAM_AC_LSB    0x63  // Aging Capacity LSB
#define BATTERY_MONITOR_PARAM_VCHG    0x64  // Charge Voltage
#define BATTERY_MONITOR_PARAM_IMIN    0x65  // Minimum Charge Current
#define BATTERY_MONITOR_PARAM_VAE    0x66  // Active Empty Voltage
#define BATTERY_MONITOR_PARAM_IAE    0x67  // Active Empty Current
#define BATTERY_MONITOR_PARAM_ACT_EMPTY  0x68  // Active Empty 40 
#define BATTERY_MONITOR_PARAM_RSNSP    0x69  // Sense Resistor Prime
// 0x6A-0x77 a bunch of parameter EEPROM that does not seem useful
#define BATTERY_MONITOR_PARAM_RSGAIN_MSB  0x78  // Sense Resistor Gain MSB
#define BATTERY_MONITOR_PARAM_RSGAIN_LSB  0x79  // Sense Resistor Gain LSB
#define BATTERY_MONITOR_PARAM_RSTC      0x7A  // Sense Resistor Temp. Coeff.
#define BATTERY_MONITOR_PARAM_COB      0x7B  // Current Offset Bias
#define BATTERY_MONITOR_PARAM_TBP34      0x7C
#define BATTERY_MONITOR_PARAM_TBP23      0x7D
#define BATTERY_MONITOR_PARAM_TBP12      0x7E

// Masks
#define MSBIT_MASK   0x80
#define LSBIT_MASK  0x01

typedef enum {
  BATTMON_SUCCESS = 0,
  BATTMON_FAIL
} BATTMON_RESULT;

void BatteryMonitorInit(UINT batt_mon_pin, IoPortId batt_mon_port);
BATTMON_RESULT BatteryMonitorReadBytes(UINT8 net_address_command, 
                    UINT8 start_addr, 
                    UINT8 *data, 
                    int nitems);
BATTMON_RESULT BatteryMonitorWriteBytes(UINT8 net_address_command, 
                    UINT8 start_addr, 
                    UINT8 *data, 
                    int nitems);

#endif