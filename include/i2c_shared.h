#ifndef I2C_SHARED_H
#define I2C_SHARED_H

BOOL I2CShared_Init(const I2C_MODULE i2c, const UINT peripheral_clock_speed, const UINT i2c_speed);
BOOL I2CShared_WriteByte(const I2C_MODULE i2c, const UINT8 i2c_addr, const UINT8 i2c_register, UINT8 data);
BOOL I2CShared_ReadByte(const I2C_MODULE i2c, UINT8 i2c_addr_write, const UINT8 i2c_addr_read, const UINT8 i2c_register, UINT8 *const buffer);
BOOL I2CShared_ReadMultipleBytes(const I2C_MODULE i2c, const UINT8 i2c_addr_write, const UINT8 i2c_addr_read, 
    const UINT8 i2c_register_start, const int nbytes, UINT8 *buffer);
void I2CShared_ResetBus(const I2C_MODULE i2c);

#endif