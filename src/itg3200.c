#include <p32xxxx.h>
#include <plib.h>
#include "itg3200.h"
#include "i2c_shared.h"

/************************************************************************************************** 
  Function: 

  Author(s): 

  Summary: 

  Description: 

  Preconditions: 

  Parameters: 

  Returns: 

  Example: 

  Conditions at Exit: 

**************************************************************************************************/
GYRO_RESULT GyroInit(I2C_MODULE i2c, char dlpf_lpf, char sample_rate_div, char power_mgmt_sel) {

  // OR the low pass frequency passed with dflp_config with full scale operation and write it to the gyro
  // Set internal clock and full scale operation
  if (GyroWrite(i2c, GYRO_DLPF_FS, dlpf_lpf | GYRO_DLPF_FS_ON) == GYRO_FAIL) {
    DBPRINTF("GyroInit: Error, could not write 0x%c to register GYRO_DLPF_FS (0x%c\n", dflp_config | GYRO_DLPF_FS_ON, GYRO_DLPF_FS);
    return GYRO_FAIL;
  }
  
  // Set sample rate divider
  // If dlpf_lpf == GYRO_DLPF_LPF_256HZ, sample rate = 8 kHz / sample_rate_div
  // Else, sample rate = 1 kHz / sample_rate_div
  if (GyroWrite(i2c, GYRO_SMPLRT_DIV, sample_rate_div) == GYRO_FAIL) {
    DBPRINTF("GyroInit: Error, could not write 0x%c to register GYRO_DLPF_FS (0x%c\n", dflp_config | GYRO_DLPF_FS_ON, GYRO_DLPF_FS);
    return GYRO_FAIL;
  }
  
  // Select a gyro PLL for clock source (more stable)
  if (GyroWrite(i2c, GYRO_PWR_MGM, power_mgmt_sel) == GYRO_FAIL) {
    DBPRINTF("GyroInit: Error, could not write 0x%c to register GYRO_DLPF_FS (0x%c\n", dflp_config | GYRO_DLPF_FS_ON, GYRO_DLPF_FS);
    return GYRO_FAIL;
  }
  
  // Set interrupts - NOT BEING USED CURRENTLY
  
  return GYRO_SUCCESS;
}

/************************************************************************************************** 
  Function: 

  Author(s): 

  Summary: 

  Description: 

  Preconditions: 

  Parameters: 

  Returns: 

  Example: 

  Conditions at Exit: 

**************************************************************************************************/
GYRO_RESULT GyroWrite(I2C_MODULE i2c, char i2c_reg, BYTE data) {
  if (I2CShared_Write(i2c, GYRO_WRITE, i2c_reg, data)) {
    return GYRO_SUCCESS;
  } else {
    return GYRO_FAIL;
  }
}

/************************************************************************************************** 
  Function: 

  Author(s): 

  Summary: 

  Description: 

  Preconditions: 

  Parameters: 

  Returns: 

  Example: 

  Conditions at Exit: 

**************************************************************************************************/
GYRO_RESULT GyroRead(I2C_MODULE i2c, char i2c_reg, char *buffer) {
  if (I2CShared_Read(i2c, GYRO_READ, i2c_reg, buffer)) {
    return GYRO_SUCCESS;
  } else {
    return GYRO_FAIL;
  }
}

/************************************************************************************************** 
  Function: 

  Author(s): 

  Summary: 

  Description: 

  Preconditions: 

  Parameters: 

  Returns: 

  Example: 

  Conditions at Exit: 

**************************************************************************************************/
GYRO_RESULT GyroReadAllAxes(I2C_MODULE i2c, gyro_raw_t *raw, BOOL readTemp) {
  char reading_rainbow[8];
  int dataToRead;
  int offsetForTemp;
  char startReadI2CReg;
  
  dataToRead = 6 + (readTemp ? 2 : 0);
  offsetForTemp = 0 + (readTemp ? 2 : 0)
  startReadI2CReg = readTemp ? GYRO_TEMP_OUT_H : GYRO_XOUT_H;
  
  // read x,y, and z data into buffer
  if (I2CShared_ReadMultipleBytes(I2C_MODULE i2c, GYRO_WRITE, GYRO_READ, startReadI2CReg, dataToRead, reading_rainbow)) {
    //expand data and place them into the accel_raw_t readings
    
    // if readTemp was true, update temperature
    if (readTemp) {
      raw->temp = (reading_rainbow[0] << 8) | reading_rainbow[1];
    }
    
    raw->x = (reading_rainbow[0 + offsetForTemp] << 8) | reading_rainbow[1 + offsetForTemp];
    raw->y = (reading_rainbow[2 + offsetForTemp] << 8) | reading_rainbow[3 + offsetForTemp];
    raw->z = (reading_rainbow[4 + offsetForTemp] << 8) | reading_rainbow[5 + offsetForTemp];
    return GYRO_SUCCESS;
  } else {
    return GYRO_FAIL;
  }
}


/************************************************************************************************** 
  Function: 

  Author(s): 

  Summary: 

  Description: 

  Preconditions: 

  Parameters: 

  Returns: 

  Example: 

  Conditions at Exit: 

**************************************************************************************************/
double GyroGetTemp(gyro_raw_t *raw) {
  double temperature;
  temperature = -GYRO_TEMP_OFFSET - raw->temp;
  temperature /= GYRO_TEMP_CONV_TO_DEGREES;
  temperature += GYRO_TEMP_OFFSET_DEGS;
  return temperature;
}

/************************************************************************************************** 
  Function: 

  Author(s): 

  Summary: 

  Description: 

  Preconditions: 

  Parameters: 

  Returns: 

  Example: 

  Conditions at Exit: 

**************************************************************************************************/
double GyroGetX(gyro_raw_t *raw) {
  double xd;
  xd = (double) raw->x / GYRO_CONV_TO_DEGREES;
  return xd;
}

/************************************************************************************************** 
  Function: 

  Author(s): 

  Summary: 

  Description: 

  Preconditions: 

  Parameters: 

  Returns: 

  Example: 

  Conditions at Exit: 

**************************************************************************************************/
double GyroGetY(gyro_raw_t *raw) {
  double yd;
  yd = (double) raw->y / GYRO_CONV_TO_DEGREES;
  return yd;
}

/************************************************************************************************** 
  Function: 

  Author(s): 

  Summary: 

  Description: 

  Preconditions: 

  Parameters: 

  Returns: 

  Example: 

  Conditions at Exit: 

**************************************************************************************************/
double GyroGetZ(gyro_raw_t *raw) {
  double zd;
  zd = (double) raw->z / GYRO_CONV_TO_DEGREES;
  return zd;
}
