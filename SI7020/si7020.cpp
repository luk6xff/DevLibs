#include "si7020.h"


SI7020::SI7020(PinName sda, PinName scl,int i2cFrequencyHz,uint8_t address):mI2c(sda,scl),mI2cAddr(address){
    
    mI2c.frequency(i2cFrequencyHz);
    if(!initSi7020()); //while(1); //TODO handle error
    mTemperature=0;
    mHumidity=0;
}   
    
    
bool SI7020::initSi7020(eMeasurementResolution measRes ,eHeater heaterEnabled){
     
     uint8_t  data;
     data= heaterEnabled<<2|measRes ;
     return write(WRITE_RH_T,&data,1);
}

//write data to the sensor
bool SI7020::write(uint8_t regAddress, uint8_t* data,int dataLength)
{
    uint8_t tempBuf[dataLength+1];
    tempBuf[0]=regAddress;
    memcpy(&(tempBuf[1]),data,dataLength);
    return mI2c.write(mI2cAddr,(char*)tempBuf,dataLength+1)==0;

}

//read data from the sensor
bool SI7020::read(uint8_t regAddress, uint8_t *data,int dataLength)
{
    mI2c.write(mI2cAddr,(char*)&regAddress,1,true);
    return (mI2c.read(mI2cAddr,(char*)data,dataLength)==0);
}

bool SI7020::readTemp(void)
{  
   uint8_t rawData[2];
   uint16_t rawTemp=0;
   if(!read(TEMP_HOLD, rawData,2)) return false;
   rawTemp= get16BitData(rawData[0],rawData[1]);
   mTemperature = (float)((175.72*rawTemp)/65536)-46.85;
   return true;
}


bool SI7020::readHumidity(void){
   uint8_t rawData[2];
   uint16_t rawHumidity=0;
   uint8_t dummyByte=0x00;
  // mI2c.write(HUMIDITY_HOLD,NULL,0,true);
 //  write(HUMIDITY_HOLD,&dummyByte,1);
 //  mI2c.read(mI2cAddr,NULL,0,true);
 //  for(int i= 0; i<0xfffff;i++);
   if(!read(HUMIDITY_HOLD, rawData,2)) return false;
   rawHumidity= get16BitData(rawData[0],rawData[1]);
   mHumidity = (float)((125*rawHumidity)/65536)-6;
   return true;    
}

bool SI7020::resetSensor(void){
    uint8_t dummyByte=0x00;
    return write(RESET,&dummyByte,1);
}
