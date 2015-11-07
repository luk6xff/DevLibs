#include "max44009.h"
#include <math.h>

 
MAX44009::MAX44009(PinName sda, PinName scl, int i2cFrequencyHz, int address):mI2c(sda,scl), mI2cAddr(address){
     mI2c.frequency(i2cFrequencyHz);
     this->mLuxIntensity=UNSET_MAX44009_LUX_INTENSITY_VALUE;
    }



bool MAX44009::write(uint8_t regAddress, uint8_t* data,int dataLength)
{
    uint8_t tempBuf[dataLength+1];
    tempBuf[0]=regAddress;
    memcpy(&(tempBuf[1]),data,dataLength);
    return mI2c.write(mI2cAddr,(char*)tempBuf,dataLength+1)==0;

}

//read data from the sensor
bool MAX44009::read(uint8_t regAddress, uint8_t *data,int dataLength)
{
    mI2c.write(mI2cAddr,(char*)&regAddress,1,true);
    return (mI2c.read(mI2cAddr,(char*)data,dataLength)==0);
}



int MAX44009::getStatus(void){

    uint8_t data=0;
    uint8_t regAddress=0x03 ;
    if(!read(regAddress,&data,1)) return -1;
    return data;
     
}
      
      
bool MAX44009::readLuxIntensity(void){
    /*According to datasheet of MAX44009 -> { Lux Intensity = 2^(exponent)*mantissa*0.045 } */
    uint8_t exponent;
    uint8_t mantissa;
    
    uint8_t highAndLowByte[2];
    uint8_t regAddress=0x03 ;
    //if(i2c.write(i2cAddr,(char*)&regAddress,1,true)) return -1;
    //if(i2c.read(i2cAddr,(char*)highAndLowByte,2,true)) return -1;
    if(!read(regAddress,highAndLowByte,2)){
        mLuxIntensity= UNSET_MAX44009_LUX_INTENSITY_VALUE;
        return false;
    }
    
    mantissa=(((highAndLowByte[0]&0x0F)<<4)|(highAndLowByte[1]&0x0F));
    exponent= (highAndLowByte[0]&0xF0)>>4;
    this->mLuxIntensity= (float)(pow(2,(float)exponent)*mantissa*0.045);
    return true;
}