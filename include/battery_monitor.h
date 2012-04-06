#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#define BATTERY_MONITOR_STATUS 		0x01
#define BATTERY_MONITOR_RAAC_MSB	0x02
#define BATTERY_MONITOR_RAAC_LSB	0x03
#define BATTERY_MONITOR_RSAC_MSB	0x04
#define BATTERY_MONITOR_RSAC_LSB	0x05
#define BATTERY_MONITOR_RARC_MSB	0x06
#define BATTERY_MONITOR_RSRC		0x07
#define BATTERY_MONITOR_IAVG_MSB	0x08
#define BATTERY_MONITOR_IAVG_LSB	0x09
#define BATTERY_MONITOR_TEMP_MSB	0x0A
#define BATTERY_MONITOR_TEMP_LSB	0x0B
#define BATTERY_MONITOR_VOLT_MSB	0x0C
#define BATTERY_MONITOR_VOLT		0x0D
#define BATTERY_MONITOR_CURRENT_MSB	0x0E
#define BATTERY_MONITOR_CURRENT_LSB	0x0F
#define BATTERY_MONITOR_ACR_MSB		0x10
#define BATTERY_MONITOR_ACR_LSB		0x11
#define BATTERY_MONITOR_ACRL_MSB	0x12
#define BATTERY_MONITOR_ACRL_LSB	0x13
#define BATTERY_MONITOR_AS			0x14
#define BATTERY_MONITOR_SFR			0x15
#define BATTERY_MONITOR_FULL_MSB	0x16
#define BATTERY_MONITOR_FULL_LSB	0x17
#define BATTERY_MONITOR_AE_MSB		0x18
#define BATTERY_MONITOR_AE_LSB		0x19
#define BATTERY_MONITOR_SE_MSB		0x1A
#define BATTERY_MONITOR_SE_LSB		0x1B

#define BATTERY_MONITOR_EEPROM_REG	0x1F
#define BATTERY_MONITOR_EEPROM_LOW	0x20
#define BATTERY_MONITOR_EEPROM_HGH	0x2F

void BatteryMonitorReset();
char BatteryMonitorReadByte();
void BatteryMonitorWriteByte(char c);

#endif