#include "adt7410.h"


ADT7410::ADT7410(PinName sda, PinName scl, int i2cFrequencyHz, int address):mI2c(sda,scl), mI2cAddr(address)
{
    mI2c.frequency(i2cFrequencyHz);
    setConfiguration();
    mTemperature= UNSET_ADT7410_TEMPERATURE;
}




bool ADT7410::write(uint8_t regAddress, uint8_t* data,int dataLength)
{
    uint8_t tempBuf[dataLength+1];
    tempBuf[0]=regAddress;
    memcpy(&(tempBuf[1]),data,dataLength);
    return mI2c.write(mI2cAddr,(char*)tempBuf,dataLength+1)==0;

}

//read data from the sensor
bool ADT7410::read(uint8_t regAddress, uint8_t *data,int dataLength)
{
    mI2c.write(mI2cAddr,(char*)&regAddress,1,true);
    return (mI2c.read(mI2cAddr,(char*)data,dataLength)==0);
}


//configuration of ADT7410 sensor
bool ADT7410::setConfiguration(CONF_FAULT_QUEUE faultQueue, CONF_CT_PIN_POLARITY ctPinPolarity, CONF_INT_PIN_POLARITY intPinPolarity, CONF_INT_CT_MODE intCtMode, CONF_OPERATION_MODE operMode, CONF_RESOLUTION res)
{
    uint8_t confByte=0;
    confByte=(res<<7|operMode<<5|intCtMode<<4|intPinPolarity<<3|ctPinPolarity<<2|faultQueue);
    if(write(0x03,&confByte,1)) {
        mResolution = res;
        return true;
    }
    return false;
}

// read 13 bit temperature
bool ADT7410::readTemp()
{

    uint8_t data[2];
    float tempFin = 0;
    int tempRaw = 0;


    // read temperature register, two bytes
    if(!read(0x00, data, 2)) {
        mTemperature= UNSET_ADT7410_TEMPERATURE;
        return false;

    }

    // temperature received takes only 13 bits
    // discard alarm flags in lower bits
    tempRaw = (data[0] << 8) | (data[1]);
    if(mResolution==_13_BIT) { ////resolution 13 --- bit 0.0625°C
        tempRaw >>= 3;
        if ( tempRaw & 0x1000) {
            tempFin = (float) (tempRaw - 8192) / 16;
        } else {
            tempFin = (float) tempRaw / 16;
        }
    } else { //resolution 16bit --- 0.0078°C.

        if(tempRaw &0x8000) {
            tempFin =(float) (tempRaw-65536)/128;
        } else
            tempFin =(float)tempRaw/128;
    }

    mTemperature=tempFin;
    return true;
}


bool ADT7410::setResolution(CONF_RESOLUTION res)
{

    uint8_t tempVal=0;
    bool retVal=0;

    if(!read(0x03,&tempVal,1)) {
        return false;
    }

    tempVal|=res<<7;
    if( !write(0x03,&tempVal,1)) {
        retVal=false;
    } else {
        retVal=true;
        mResolution=res;
    }

    return retVal;
}


int ADT7410::readIdNumber(void)
{

    uint8_t data = 0;
    uint8_t regAddress = 0x0B;
    if(read(regAddress, &data, 1))
        return data;
    else
        return -1;

}