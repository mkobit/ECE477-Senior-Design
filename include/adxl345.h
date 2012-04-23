#ifndef ACCEL345_H
#define ACCEL345_H

// constants taken from https://github.com/a1ronzo/6DOF-Digital/
/* Register map for the ACCEL345 */
#define ACCEL_READ  0xA7
#define ACCEL_WRITE  0xA6

//ACCEL Register Map
#define ACCEL_DEVID      0x00  //Device ID Register
#define ACCEL_THRESH_TAP    0x1D  //Tap Threshold
#define ACCEL_OFSX      0x1E  //X-axis offset
#define ACCEL_OFSY      0x1F  //Y-axis offset
#define ACCEL_OFSZ      0x20  //Z-axis offset
#define ACCEL_DUR      0x21  //Tap Duration
#define ACCEL_LATENT      0x22  //Tap latency
#define ACCEL_WINDOW      0x23  //Tap window
#define ACCEL_THRESH_ACT    0x24  //Activity Threshold
#define ACCEL_THRESH_INACT  0x25  //Inactivity Threshold
#define ACCEL_TIME_INACT    0x26  //Inactivity Time
#define ACCEL_INACT_CTL    0x27  //Axis enable control for activity and inactivity detection
#define ACCEL_THRESH_FF    0x28  //free-fall threshold
#define ACCEL_TIME_FF    0x29  //Free-Fall Time
#define ACCEL_TAP_AXES    0x2A  //Axis control for tap/double tap
#define ACCEL_ACT_TAP_STATUS  0x2B  //Source of tap/double tap
#define ACCEL_BW_RATE    0x2C  //Data rate and power mode control
#define ACCEL_POWER_CTL    0x2D  //Power Control Register
#define ACCEL_INT_ENABLE    0x2E  //Interrupt Enable Control
#define ACCEL_INT_MAP    0x2F  //Interrupt Mapping Control
#define ACCEL_INT_SOURCE    0x30  //Source of interrupts
#define ACCEL_DATA_FORMAT  0x31  //Data format control
#define ACCEL_DATAX0      0x32  //X-Axis Data LSB
#define ACCEL_DATAX1      0x33  //X-Axis Data MSB
#define ACCEL_DATAY0      0x34  //Y-Axis Data LSB
#define ACCEL_DATAY1      0x35  //Y-Axis Data MSB
#define ACCEL_DATAZ0      0x36  //Z-Axis Data LSB
#define ACCEL_DATAZ1      0x37  //Z-Axis Data MSB
#define  ACCEL_FIFO_CTL    0x38  //FIFO control
#define  ACCEL_FIFO_STATUS  0x39  //FIFO status

//Power Control Register Bits
#define ACCEL_WU_0    (1 << 0)  //Wake Up Mode - Bit 0
#define ACCEL_WU_1    (1 << 1)  //Wake Up mode - Bit 1
#define ACCEL_SLEEP    (1 << 2)  //Sleep Mode
#define ACCEL_MEASURE  (1 << 3)  //Measurement Mode
#define ACCEL_AUTO_SLP  (1 << 4)  //Auto Sleep Mode bit
#define ACCEL_LINK    (1 << 5)  //Link bit

//Interrupt Enable/Interrupt Map/Interrupt Source Register Bits
#define ACCEL_OVERRUN  (1 << 0)
#define ACCEL_WATERMARK  (1 << 1)
#define ACCEL_FREE_FALL  (1 << 2)
#define ACCEL_INACTIVITY  (1 << 3)
#define ACCEL_ACTIVITY  (1 << 4)
#define ACCEL_DOUBLE_TAP  (1 << 5)
#define ACCEL_SINGLE_TAP  (1 << 6)
#define ACCEL_DATA_READY  (1 << 7)

// Baud rate settings in regular power mode
#define ACCEL_BW_1600 (0x0F)
#define ACCEL_BW_800  (0x0E)
#define ACCEL_BW_400  (0x0D)
#define ACCEL_BW_200  (0x0C)
#define ACCEL_BW_100  (0x0B)
#define ACCEL_BW_50   (0x0A)
#define ACCEL_BW_25   (0x09)

//Data Format Bits
#define ACCEL_RANGE_2G (0x00)
#define ACCEL_RANGE_4G (0x01)
#define ACCEL_RANGE_8G (0x02)
#define ACCEL_RANGE_16G (0x03)
#define ACCEL_RANGE_0  (1 << 0)
#define ACCEL_RANGE_1  (1 << 1)
#define ACCEL_JUSTIFY  (1 << 2)
#define ACCEL_FULL_RES  (1 << 3)

#define ACCEL_INT_INVERT  (1 << 5)
#define ACCEL_SPI    (1 << 6)
#define ACCEL_SELF_TEST  (1 << 7)

typedef enum {
  ACCEL_SUCCESS = 0,
  ACCEL_FAIL
} ACCEL_RESULT;

typedef struct accel_raw_t {
  short int x;
  short int y;
  short int z;
  int scale_ind;
} accel_raw_t;

ACCEL_RESULT AccelInit(I2C_MODULE i2c, unsigned char range, unsigned char bandwidth, accel_raw_t *raw);
ACCEL_RESULT AccelWrite(I2C_MODULE i2c, unsigned char i2c_reg, unsigned char data);
ACCEL_RESULT AccelRead(I2C_MODULE i2c, unsigned char i2c_reg, unsigned char *buffer);
ACCEL_RESULT AccelReadAllAxes(I2C_MODULE i2c, accel_raw_t *raw);
double AccelGetX(accel_raw_t *raw);
double AccelGetY(accel_raw_t *raw);
double AccelGetZ(accel_raw_t *raw);

#endif