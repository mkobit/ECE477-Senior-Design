#ifndef I2C_SHARED_H
#define I2C_SHARED_H

BOOL I2CShared_Init(I2C_MODULE i2c, UINT peripheral_clock_speed, UINT i2c_speed);
BOOL I2CShared_WriteByte(I2C_MODULE i2c, UINT8 i2c_addr, UINT8 i2c_register, UINT8 data);
BOOL I2CShared_ReadByte(I2C_MODULE i2c, UINT8 i2c_addr_write, UINT8 i2c_addr_read, UINT8 i2c_register, UINT8 *buffer);
BOOL I2CShared_ReadMultipleBytes(I2C_MODULE i2c, UINT8 i2c_addr_write, UINT8 i2c_addr_read, UINT8 i2c_register_start, int nbytes, UINT8 *buffer);
inline void I2CShared_ResetBus(I2C_MODULE i2c);

#endif