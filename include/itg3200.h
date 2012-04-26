#ifndef GYRO3200_H
#define GYRO3200_H

#define GyroGetRoll(p_gyro) (GyroGetX(p_gyro))
#define GyroGetPitch(p_gyro) (GyroGetY(p_gyro))
#define GyroGetYaw(p_gyro) (GyroGetZ(p_gyro))

/* Register map for the GYRO3200 */
#define GYRO_READ  0xD1
#define GYRO_WRITE  0xD0

#define GYRO_WHO_AM_I  0x00
#define GYRO_SMPLRT_DIV  0x15
#define  GYRO_DLPF_FS    0x16
#define GYRO_INT_CFG    0x17
#define GYRO_INT_STATUS  0x1A
#define  GYRO_TEMP_OUT_H  0x1B
#define  GYRO_TEMP_OUT_L  0x1C
#define GYRO_XOUT_H  0x1D
#define  GYRO_XOUT_L  0x1E
#define GYRO_YOUT_H  0x1F
#define GYRO_YOUT_L  0x20
#define GYRO_ZOUT_H  0x21
#define GYRO_ZOUT_L  0x22
#define  GYRO_PWR_MGM  0x3E

//Sample Rate Divider
//Fsample = Fint / (divider + 1) where Fint is either 1kHz or 8kHz

//DLPF register settings
// Low pass filter bandwidth
#define GYRO_DLPF_LPF_256HZ  (0)
#define GYRO_DLPF_LPF_188HZ  (1)
#define GYRO_DLPF_LPF_98HZ  (2)
#define GYRO_DLPF_LPF_42HZ  (3)
#define GYRO_DLPF_LPF_20HZ  (4)
#define GYRO_DLPF_LPF_10HZ  (5)
#define GYRO_DLPF_LPF_5HZ  (6)
#define GYRO_DLPF_CFG_1  (1 << 1)
#define GYRO_DLPF_CFG_2  (1 << 2)
#define GYRO_DLPF_FS_ON (0x18)

//Power Management Register Bits
//Recommended to set CLK_SEL to 1,2 or 3 at startup for more stable clock
#define GYRO_PWR_MGM_CLK_SEL_INTERNAL (0)
#define GYRO_PWR_MGM_CLK_SEL_X  (1)
#define GYRO_PWR_MGM_CLK_SEL_Y  (2)
#define GYRO_PWR_MGM_CLK_SEL_Z  (3)
#define GYRO_PWR_MGM_STBY_Z  (1 << 3)
#define GYRO_PWR_MGM_STBY_Y  (1 << 4)
#define GYRO_PWR_MGM_STBY_X  (1 << 5)
#define GYRO_PWR_MGM_SLEEP  (1 << 6)
#define GYRO_PWR_MGM_H_RESET  (1 << 7)

//Interrupt Configuration Bits
#define GYRO_INT_CFG_ACTL      (1 << 7)
#define GYRO_INT_CFG_OPEN      (1 << 6)
#define GYRO_INT_CFG_LATCH_INT_EN  (1 << 5)
#define GYRO_INT_CFG_INT_ANYRD    (1 << 4)
#define GYRO_INT_CFG_GYRO_RDY_EN    (1 << 2)
#define GYRO_INT_CFG_RAW_RDY_EN    (1 << 0)

#define GYRO_CONV_TO_DEGREES 14.375f
#define GYRO_TEMP_CONV_TO_DEGREES 280.0f
#define GYRO_TEMP_OFFSET -13200
#define GYRO_TEMP_OFFSET_DEGS 35

typedef enum {
  GYRO_SUCCESS = 0,
  GYRO_FAIL
} GYRO_RESULT;

typedef struct gyro_raw_t {
  INT16 x;
  INT16 y;
  INT16 z;
  INT16 temp;
} gyro_raw_t;

typedef struct gyro_t {
  gyro_raw_t raw;
  INT16 xPolarity;
  INT16 yPolarity;
  INT16 zPolarity;
  INT16 xOffset;
  INT16 yOffset;
  INT16 zOffset;
  float xGain;
  float yGain;
  float zGain;
} gyro_t;

GYRO_RESULT GyroInit(gyro_t *const gyro, const I2C_MODULE i2c, const UINT8 dlpf_lpf, const UINT8 sample_rate_div, const UINT8 power_mgmt_sel);
GYRO_RESULT GyroWrite(const I2C_MODULE i2c, const UINT8 i2c_reg, const UINT8 data);
GYRO_RESULT GyroRead(const I2C_MODULE i2c, UINT8 i2c_reg, UINT8 *const buffer);
GYRO_RESULT GyroReadAllAxes(const I2C_MODULE i2c, gyro_t *const gyro, const BOOL readTemp);
GYRO_RESULT GyroCalibrate(I2C_MODULE i2c, gyro_t *const gyro, int samplesToTake, UINT ms_delay) {
void GyroSetRevPolarity(gyro_t *const gyro, const BOOL xPol, const BOOL yPol, const BOOL zPol);
void GyroSetGains(gyro_t *const gyro, const float xGain, const float yGain, const float zGain);
float GyroGetTemp(gyro_t *const gyro);
float GyroGetX(gyro_t *const gyro);   // X === Roll
float GyroGetY(gyro_t *const gyro);   // Y === Pitch
float GyroGetZ(gyro_t *const gyro);   // Z === Yaw

#endif