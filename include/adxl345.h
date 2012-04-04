#ifndef ADXL345_H
#define ADXL345_H

// constants taken from https://github.com/a1ronzo/6DOF-Digital/
/* Register map for the ADXL345 */
#define ADXL_READ	0xA7
#define ADXL_WRITE	0xA6

//ADXL Register Map
#define	ADXL_DEVID			0x00	//Device ID Register
#define ADXL_THRESH_TAP		0x1D	//Tap Threshold
#define	ADXL_OFSX			0x1E	//X-axis offset
#define	ADXL_OFSY			0x1F	//Y-axis offset
#define	ADXL_OFSZ			0x20	//Z-axis offset
#define	ADXL_DUR			0x21	//Tap Duration
#define	ADXL_LATENT			0x22	//Tap latency
#define	ADXL_WINDOW			0x23	//Tap window
#define	ADXL_THRESH_ACT		0x24	//Activity Threshold
#define	ADXL_THRESH_INACT	0x25	//Inactivity Threshold
#define	ADXL_TIME_INACT		0x26	//Inactivity Time
#define	ADXL_INACT_CTL		0x27	//Axis enable control for activity and inactivity detection
#define	ADXL_THRESH_FF		0x28	//free-fall threshold
#define	ADXL_TIME_FF		0x29	//Free-Fall Time
#define	ADXL_TAP_AXES		0x2A	//Axis control for tap/double tap
#define ADXL_ACT_TAP_STATUS	0x2B	//Source of tap/double tap
#define	ADXL_BW_RATE		0x2C	//Data rate and power mode control
#define ADXL_POWER_CTL		0x2D	//Power Control Register
#define	ADXL_INT_ENABLE		0x2E	//Interrupt Enable Control
#define	ADXL_INT_MAP		0x2F	//Interrupt Mapping Control
#define	ADXL_INT_SOURCE		0x30	//Source of interrupts
#define	ADXL_DATA_FORMAT	0x31	//Data format control
#define ADXL_DATAX0			0x32	//X-Axis Data 0
#define ADXL_DATAX1			0x33	//X-Axis Data 1
#define ADXL_DATAY0			0x34	//Y-Axis Data 0
#define ADXL_DATAY1			0x35	//Y-Axis Data 1
#define ADXL_DATAZ0			0x36	//Z-Axis Data 0
#define ADXL_DATAZ1			0x37	//Z-Axis Data 1
#define	ADXL_FIFO_CTL		0x38	//FIFO control
#define	ADXL_FIFO_STATUS	0x39	//FIFO status

//Power Control Register Bits
#define ADXL_WU_0		(1 << 0)	//Wake Up Mode - Bit 0
#define	ADXL_WU_1		(1 << 1)	//Wake Up mode - Bit 1
#define ADXL_SLEEP		(1 << 2)	//Sleep Mode
#define	ADXL_MEASURE	(1 << 3)	//Measurement Mode
#define ADXL_AUTO_SLP	(1 << 4)	//Auto Sleep Mode bit
#define ADXL_LINK		(1 << 5)	//Link bit

//Interrupt Enable/Interrupt Map/Interrupt Source Register Bits
#define	ADXL_OVERRUN	(1 << 0)
#define	ADXL_WATERMARK	(1 << 1)
#define ADXL_FREE_FALL	(1 << 2)
#define	ADXL_INACTIVITY	(1 << 3)
#define	ADXL_ACTIVITY	(1 << 4)
#define ADXL_DOUBLE_TAP	(1 << 5)
#define	ADXL_SINGLE_TAP	(1 << 6)
#define	ADXL_DATA_READY	(1 << 7)

//Data Format Bits
#define ADXL_RANGE_0	(1 << 0)
#define	ADXL_RANGE_1	(1 << 1)
#define ADXL_JUSTIFY	(1 << 2)
#define	ADXL_FULL_RES	(1 << 3)

#define	ADXL_INT_INVERT	(1 << 5)
#define	ADXL_SPI		(1 << 6)
#define	ADXL_SELF_TEST	(1 << 7)



#endif