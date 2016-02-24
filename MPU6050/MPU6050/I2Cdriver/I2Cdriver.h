#ifndef I2Cdriver_h
#define I2Cdriver_h

#include "mbed.h"

class I2Cdriver {
    private:
        I2C i2c;
    public:
        I2Cdriver();
        I2Cdriver(PinName i2cSda, PinName i2cScl, uint32_t i2cFreq= 400000);        
        void setFrequency(uint32_t hz);
        
        bool write(uint8_t slaveAddr,uint8_t regAddress,int dataLength, uint8_t const* data);

        bool read(uint8_t slaveAddr, uint8_t regAddress, int dataLength,uint8_t *const data);

};

#endif