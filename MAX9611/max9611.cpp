#include "max9611.h"




MAX9611::MAX9611(PinName sda, PinName scl, int i2cFrequencyHz, int address):mI2c(sda,scl), mI2cAddr(address)
{
    mI2c.frequency(i2cFrequencyHz);
    if(!initMax9611()); //while(1){ //TODO handle error}
    mTemperature=0;
    mCsaCurrentValueOffset=0x0a;  //NOTE! set this parameter on your own, it depends on your used sensor
}



//write data to the sensor
bool MAX9611::write(uint8_t regAddress, uint8_t* data,int dataLength)
{
    uint8_t tempBuf[dataLength+1];
    tempBuf[0]=regAddress;
    memcpy(&(tempBuf[1]),data,dataLength);
    return mI2c.write(mI2cAddr,(char*)tempBuf,dataLength+1)==0;

}

//read data from the sensor
bool MAX9611::read(uint8_t regAddress, uint8_t *data,int dataLength)
{
    mI2c.write(mI2cAddr,(char*)&regAddress,1,true);
    return (mI2c.read(mI2cAddr,(char*)data,dataLength)==0);
}

//configuration of MAX9611
bool MAX9611::initMax9611(eCtrlReg1MUX mux,
                          eCtrlReg1SHDN shdn,
                          eCtrlReg1LR lr,
                          eCtrlReg1MODE mode,
                          eCtrlReg2DTIM watchdogDelay,
                          eCtrlReg2RTIM watchdogRetryDelay)
{
    uint8_t retVal=0;
    uint8_t controlReg1=0;
    uint8_t controlReg2=0;
    controlReg1=(mode<<5|lr<<4|shdn<<3|mux);
    controlReg2=(watchdogDelay<<3|watchdogRetryDelay<<2);
    retVal+= write(CONTROL_REGISTER_1_ADRR,&controlReg1,1);
    retVal+= write(CONTROL_REGISTER_2_ADRR,&controlReg2,1);
    if(retVal!=2) return false;
    mMuxReg= mux;
    return true;
}


bool MAX9611::readTemp(void)
{
    uint8_t rawData[2];
    uint16_t rawTemp=0;
    if(!read(TEMP_DATA_BYTE_MSB_ADRR, rawData,2)) return false;
    rawTemp= get9BitData(rawData[0],rawData[1]);
    //mRawInt =rawTemp;
    if ( rawTemp & 0x100) {
        mTemperature = (float) (rawTemp- 256)*0.48;
    } else {
        mTemperature = (float)(rawTemp) *0.48;
    }

    return true;
}


bool MAX9611::readCSAOutputValue(void)
{
    uint8_t rawData[2];
    uint16_t rawCSAVal=0;
    if(!read(CSA_DATA_BYTE_MSB_ADRR, rawData,2)) return false;
    rawCSAVal= get12BitData(rawData[0],rawData[1]);
    //mRawInt = rawCSAVal; //debug
    if(rawCSAVal<=mCsaCurrentValueOffset)
        mCurrentSenseAmplifierOutput=0;
    else mCurrentSenseAmplifierOutput= (float)(rawCSAVal)*(getCSACurrentCoeffmA()); // to get result in [mA]

    return true;
}

// useful debug methods

uint16_t MAX9611::readRawControl(void)
{
    uint8_t rawData[2];
    uint16_t rawCtrl=0;
    read(CONTROL_REGISTER_1_ADRR, rawData,2) ;
    rawCtrl= (rawData[0]<<8)|rawData[1];
    return rawCtrl;
}


uint16_t MAX9611::readRawRsValue(void)
{
    uint8_t rawData[2];
    uint16_t rawRsVal=0;
    read(RS_DATA_BYTE_MSB_ADRR, rawData,2) ;
    rawRsVal= get12BitData(rawData[0],rawData[1]);
    return rawRsVal;
}


uint16_t MAX9611::readRawCSAOutValue(void)
{
    uint8_t rawData[2];
    uint16_t rawCSAOut=0;
    read(CSA_DATA_BYTE_MSB_ADRR, rawData,2) ;
    rawCSAOut= get12BitData(rawData[0],rawData[1]);
    return rawCSAOut;
}


uint16_t MAX9611::readRawOutValue(void)
{
    uint8_t rawData[2];
    uint16_t rawOut=0;
    read(OUT_DATA_BYTE_MSB_ADRR, rawData,2) ;
    rawOut= get12BitData(rawData[0],rawData[1]);
    return rawOut;
}
