#ifndef GYRO3200_H
#define GYRO3200_H

// constants taken from https://github.com/a1ronzo/6DOF-Digital/
/* Register map for the GYRO3200 */
#define GYRO_READ	0xD1
#define GYRO_WRITE	0xD0

#define GYRO_WHO_AM_I	0x00
#define GYRO_SMPLRT_DIV	0x15
#define	GYRO_DLPF_FS		0x16
#define GYRO_INT_CFG		0x17
#define GYRO_INT_STATUS	0x1A
#define	GYRO_TEMP_OUT_H	0x1B
#define	GYRO_TEMP_OUT_L	0x1C
#define GYRO_XOUT_H	0x1D
#define	GYRO_XOUT_L	0x1E
#define GYRO_YOUT_H	0x1F
#define GYRO_YOUT_L	0x20
#define GYRO_ZOUT_H	0x21
#define GYRO_ZOUT_L	0x22
#define	GYRO_PWR_MGM	0x3E

//Sample Rate Divider
//Fsample = Fint / (divider + 1) where Fint is either 1kHz or 8kHz

//DLPF, Full Scale Register Bits
//FS_SEL must be set to 3 for proper operation
//Set DLPF_CFG to 3 for 1kHz Fint and 42 Hz Low Pass Filter
#define GYRO_DLPF_CFG_0	(1 << 0)
#define GYRO_DLPF_CFG_1	(1 << 1)
#define GYRO_DLPF_CFG_2	(1 << 2)
#define GYRO_DLPF_FS_SEL_0	(1 << 3)
#define GYRO_DLPF_FS_SEL_1	(1 << 4)

//Power Management Register Bits
//Recommended to set CLK_SEL to 1,2 or 3 at startup for more stable clock
#define GYRO_PWR_MGM_CLK_SEL_0	(1 << 0)
#define GYRO_PWR_MGM_CLK_SEL_1	(1 << 1)
#define GYRO_PWR_MGM_CLK_SEL_2	(1 << 2)
#define GYRO_PWR_MGM_STBY_Z	(1 << 3)
#define GYRO_PWR_MGM_STBY_Y	(1 << 4)
#define GYRO_PWR_MGM_STBY_X	(1 << 5)
#define GYRO_PWR_MGM_SLEEP	(1 << 6)
#define GYRO_PWR_MGM_H_RESET	(1 << 7)

//Interrupt Configuration Bits
#define GYRO_INT_CFG_ACTL			(1 << 7)
#define GYRO_INT_CFG_OPEN			(1 << 6)
#define GYRO_INT_CFG_LATCH_INT_EN	(1 << 5)
#define GYRO_INT_CFG_INT_ANYRD		(1 << 4)
#define GYRO_INT_CFG_GYRO_RDY_EN		(1 << 2)
#define GYRO_INT_CFG_RAW_RDY_EN		(1 << 0)

#define GYRO_FAIL -1
#define GYRO_SUCCESS 1

typedef enum {
  GYRO_SUCCESS = 0,
  GYRO_FAIL
} GYRO_RESULT;

typedef gyro_raw_t {
  short int x;
  short int y;
  short int z;
  int scale_ind;
} gyro_raw_t;

GYRO_RESULT GyroInitI2C(I2C_MODULE i2c, 
						unsigned int peripheral_clock_speed, 
						unsigned int i2c_speed, 
						char resolution, 
						char bandwidth, 
						accel_raw_readings *readings);
GYRO_RESULT GyroWrite(I2C_MODULE i2c, char i2c_reg, BYTE data);
GYRO_RESULT GyroRead(I2C_MODULE i2c, char i2c_reg, char *buffer);
GYRO_RESULT GyroReadAllAxes(I2C_MODULE i2c, accel_raw_readings *readings);
double GyroGetX(gyro_raw_t *readings);
double GyroGetY(gyro_raw_t *readings);
double GyroGetZ(gyro_raw_t *readings);

#endif