#include "I2Cdriver.h"

//#define DEBUG_SERIAL_ENABLED
#ifdef DEBUG_SERIAL_ENABLED
    Serial debugSerial(USBTX, USBRX);
#endif


I2Cdriver::I2Cdriver(): i2c(I2C_SDA,I2C_SCL)  //default I2C
{
    setFrequency(400000);
}

I2Cdriver::I2Cdriver(PinName i2cSda, PinName i2cScl,uint32_t i2cFreq ): i2c(i2cSda,i2cScl)
{
    setFrequency(i2cFreq); 
}

void I2Cdriver::setFrequency(uint32_t hz)
{
    i2c.frequency(hz);   
}



/** writes bytes to an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddress First register regAddr to write to
 * @param dataLength Number of bytes to write
 * @param data Buffer to write data from
 * @return Number of bytes read (0 indicates failure)
 */
bool I2Cdriver::write(uint8_t slaveAddr,uint8_t regAddress,int dataLength, uint8_t const* data)
{
    uint8_t tempBuf[dataLength+1];
    tempBuf[0]=regAddress;
    memcpy(&(tempBuf[1]),data,dataLength);
    return i2c.write(slaveAddr,(char*)tempBuf,dataLength+1)==0;

}

/** Read bytes from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddress First register regAddr to read from
 * @param dataLength Number of bytes to read
 * @param data Buffer to store read data in
 * @return Number of bytes read (0 indicates failure)
 */
bool I2Cdriver::read(uint8_t slaveAddr, uint8_t regAddress, int dataLength,uint8_t *const data)
{
    i2c.write(slaveAddr,(char*)&regAddress,1,true);
    return (i2c.read(slaveAddr,(char*)data,dataLength)==0);
}



