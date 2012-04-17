#ifndef I2C_SHARED_H
#define I2C_SHARED_H

BOOL I2CShared_StartTransfer(I2C_MODULE i2c, BOOL restart);
BOOL I2CShared_TransmitOneByte(I2C_MODULE i2c, BYTE data);
BOOL I2CShared_WriteByte(I2C_MODULE i2c, char i2c_addr, char i2c_register, BYTE data);
BOOL I2CShared_ReadByte(I2C_MODULE i2c, char i2c_addr_write, char i2c_addr_read, char i2c_register, char *buffer);
BOOL I2CShared_ReadMultipleBytes(I2C_MODULE i2c, char i2c_addr_write, char i2c_addr_read, char i2c_register_start, int nbytes, char *buffer);
void I2CShared_StopTransfer(I2C_MODULE i2c);

#endif