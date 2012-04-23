#ifndef I2C_SHARED_H
#define I2C_SHARED_H

BOOL I2CShared_Init(I2C_MODULE i2c, unsigned int peripheral_clock_speed, unsigned int i2c_speed);
BOOL I2CShared_WriteByte(I2C_MODULE i2c, unsigned char i2c_addr, unsigned char i2c_register, unsigned char data);
BOOL I2CShared_ReadByte(I2C_MODULE i2c, unsigned char i2c_addr_write, unsigned char i2c_addr_read, unsigned char i2c_register, unsigned char *buffer);
BOOL I2CShared_ReadMultipleBytes(I2C_MODULE i2c, unsigned char i2c_addr_write, unsigned char i2c_addr_read, unsigned char i2c_register_start, int nbytes, unsigned char *buffer);
inline void I2CShared_ResetBus(I2C_MODULE i2c);

#endif