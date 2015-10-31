#include "ds2782.h"


DS2782::DS2782(PinName sda, PinName scl,int i2cFrequencyHz,uint8_t address):mI2c(sda,scl),mI2cAddr(address){
    
    mI2c.frequency(i2cFrequencyHz);
    mTemperature=0;
    //initDS2782();
    //setACRRegister(0xffff);  //to clear flags
}   
    
    
bool DS2782::initDS2782(void){
     
     uint16_t full40Reg = 0x3200;
     uint8_t buf[2];
     fillBuf(full40Reg, buf);
     setEepromBlockRegister(FULL_40_MSB,buf, 2);
    
     buf[0]= 0xD5;
     setEepromBlockRegister(VCHG,buf, 1);
     
     buf[0]=0x14;
     setEepromBlockRegister(IMIN,buf,1);
     
     buf[0]=0xB3;
     setEepromBlockRegister(VAE,buf,1);
     
     buf[0]=0x0a;
     setEepromBlockRegister(IAE,buf,1);
     
     buf[0]=0x06;
     setEepromBlockRegister(ACTIVE_EMPTY_40,buf,1);
     
     //fillBuf(full40Reg, buf);
     //setEepromBlockRegister(RSGAIN_MSB,buf,2);
     
     buf[0]=0;
     setEepromBlockRegister(RSTC,buf,1);
     
     buf[0]=0x32;
     setEepromBlockRegister(RSNSP,buf,1);
     
     buf[0]=0;
     setEepromBlockRegister(AB,buf,1);
     
     
     return true;
}

//write data to the sensor
bool DS2782::write(uint8_t regAddress, uint8_t* data,int dataLength)
{
    uint8_t tempBuf[dataLength+1];
    tempBuf[0]=regAddress;
    memcpy(&(tempBuf[1]),data,dataLength);
    return mI2c.write(mI2cAddr,(char*)tempBuf,dataLength+1)==0;

}

//read data from the sensor
bool DS2782::read(uint8_t regAddress, uint8_t *data,int dataLength)
{
    mI2c.write(mI2cAddr,(char*)&regAddress,1,true);
    return (mI2c.read(mI2cAddr,(char*)data,dataLength)==0);
}

bool DS2782::readTemperature(void)
{  
   uint8_t rawData[2];
   uint16_t rawTemp=0;
   if(!read(TEMP_MSB_REG, rawData,2)) return false;
      rawTemp= (((rawData[0]&~(1<<7))<<3)|((rawData[1]>>5)&0xF));
      mTemperature = (float)(rawTemp*0.125);
   return true;
}

float DS2782::getTemperature(void)
{
    return mTemperature;
}


bool DS2782::readCurrent(void){

   uint8_t rawData[2];
   uint16_t rawRes=0;
   if(!read(CURRENT_MSB_REG, rawData,2)) return false;
      rawRes= get16BitData(rawData[0],rawData[1]);
      //rawRes&=~(1<<15);
    if(rawRes &0x8000){
      mCurrent = (float)(rawRes-65536)*0.07813;
    }
    else 
      //mCurrent = (float)(rawRes-32768)*0.07813;
      mCurrent = (float)(rawRes)*0.07813;
   return true;
}

float DS2782::getCurrent(void)
{
    return mCurrent;
}



bool DS2782::readVoltage(void){

   uint8_t rawData[2];
   uint16_t rawVolt=0;
   if(!read(VOLT_MSB_REG, rawData,2)) return false;
      rawVolt= (((rawData[0]&~(1<<7))<<3)|((rawData[1]>>5)&0xF));
      mVoltage= (float)(rawVolt*4.88);
   return true;
}

float DS2782::getVoltage(void)
{
    return mVoltage;
}


bool DS2782::setACRRegister(uint16_t reg)
{    
    uint8_t buf[2];
    buf[0]= ((reg>>8)&0xFF);
    buf[1]= ((reg)&0xFF);
    if(!(write(ACR_MSB_REG, buf,2))) return false;
    return true;        
}


float DS2782::readAcrReg(void){

   uint8_t rawData[2];
   uint16_t rawRes=0;
   if(!read(ACR_MSB_REG, rawData,2)) return false;
      rawRes= get16BitData(rawData[0],rawData[1]);
      
    if(rawRes &0x8000){
      return ((float)(rawRes-65536)*1.5625);
    }
    else 
      return ((float)(rawRes)*1.5625);
}


bool DS2782::setEepromBlockRegister(ParamEepromReg reg, uint8_t * value, uint8_t length){
    uint8_t buf[length];
    memcpy(buf,value,length);
    if(!(write(reg, buf,length))) return false;
    return true; 
    
}

uint8_t DS2782::readRarcReg(void){
    uint8_t rarcRegVal;  //unit [%]
    if(!read(RARC_REG, &rarcRegVal,1)) return 255;    
    return rarcRegVal;
}


uint8_t DS2782::readStatusReg(void){
    uint8_t statusRegVal;
    if(!read(STATUS, &statusRegVal,1)) return 255;    
    return statusRegVal;
    
}
