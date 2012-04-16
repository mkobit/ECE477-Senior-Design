#ifndef I2C_SHARED_H
#define I2C_SHARED_H

BOOL I2CShared_StartTransfer(I2C_MODULE i2c, BOOL restart);
BOOL I2CShared_TransmitOneByte(I2C_MODULE i2c, BYTE data);
BOOL I2CShared_Write(I2C_MODULE i2c, char i2c_addr, char i2c_register, BYTE data);
BOOL I2CShared_Read(I2C_MODULE i2c, char i2c_addr, char i2c_register, char *buffer);
void I2CShared_StopTransfer(I2C_MODULE i2c);

#endif