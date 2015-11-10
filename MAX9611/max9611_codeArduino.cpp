

/* 
I2C pins for arduino uno, A4 (SDA), A5 (SCL) 
 */



#ifndef MAX9611_H
#define MAX9611_H

#define MAX9611_I2C_ADDRESS 0xE1>>1   //A0 and A1 PIN are conected to GND , Write address 0xE0>>1, Read Address 0xE1>>1 (shifted >>1 cuz of arduino likes 7 bit addresses : )


class MAX9611
{


    /**********private members and methods********************************/
private:

    typedef enum {
        CHANNEL_A_0=0,  /*Read current-sense amplifier output from ADC, gain = 1x*/
        CHANNEL_A_1,    /*Read current-sense amplifier output from ADC, gain = 4x*/
        CHANNEL_A_2,    /*Read current-sense amplifier output from ADC, gain = 8x*/
        CHANNEL_B,      /*Read average voltage of RS+ (input common-mode voltage) from ADC*/
        CHANNEL_C,      /*Read voltage of OUT from ADC*/
        CHANNEL_D,      /*Read voltage of SET from ADC*/
        CHANNEL_E,      /*Read internal die temperature from ADC*/
        ALL_CHANNELS    /*Read all channels in fast-read mode, sequentially every 2ms. Uses last gain setting.*/
    } eCtrlReg1MUX;

    typedef enum {
        NORMAL_OPERATION_SHDN=0,
        SHUTDOWN_MODE
    } eCtrlReg1SHDN;

    typedef enum {
        NORMAL_OPERATION_LR=0,
        RESET
    } eCtrlReg1LR;

    typedef enum {
        NORMAL_OPERATION_MODE=0,
        COMPARATOR_MODE=7,
        OPAMP_MODE=3
    } eCtrlReg1MODE;

//watchdog delay time
    typedef enum {
        _1MS=0,
        _100US=1
    } eCtrlReg2DTIM;

//watchdog retry delay time
    typedef enum {
        _50MS=0,
        _10MS=1
    } eCtrlReg2RTIM;

    //watchdog retry delay time
    typedef enum {
        CSA_DATA_BYTE_MSB_ADRR= 0x00,
        CSA_DATA_BYTE_LSB_ADRR= 0x01,
        RS_DATA_BYTE_MSB_ADRR= 0x02,
        RS_DATA_BYTE_LSB_ADRR= 0x03,
        OUT_DATA_BYTE_MSB_ADRR= 0x04,
        OUT_DATA_BYTE_LSB_ADRR= 0x05,
        SET_DATA_BYTE_MSB_ADRR= 0x06,
        SET_DATA_BYTE_LSB_ADRR= 0x07,
        TEMP_DATA_BYTE_MSB_ADRR= 0x08,
        TEMP_DATA_BYTE_LSB_ADRR= 0x09,
        CONTROL_REGISTER_1_ADRR= 0x0A,
        CONTROL_REGISTER_2_ADRR= 0x0B
    } eRegAddresses;



    /** Write data to the given register
     *
     * @returns
     *   1 on success,
     *   0 on error
     */
    bool write(uint8_t regAddress, uint8_t* data,int dataLength);

    /** Write data to the given register
     * @param register Address
     * @param data to read
     * @param length of data to read
     * @returns
     *   1 on success,
     *   0 on error
     */
    bool read(uint8_t regAddress, uint8_t* data,int length);


    /** Make 12 bit data from 2 bytes received from thr device data read from Data regiters of Max9611/9612 are laid in the following way :
     *  Byte 1:  bit7-MSB12........bit0-MSB05 ;  Byte 2: bit7-LSB04.... bit4-LSB00
     * @param MSB byte
     * @param 4 bits of LSB bytes
     * @returns 1 2bit data
     *
     */
    inline uint16_t get12BitData(uint8_t msbByte,uint8_t lsbByte) {
        uint16_t data12Bit= (msbByte<<4)|((lsbByte>>4)&0x0F);
        return data12Bit;
    }


    inline uint16_t get9BitData(uint8_t msbByte,uint8_t lsbByte) {
        uint16_t data9Bit= (msbByte<<1)|((lsbByte>>6)&0x01);
        return data9Bit;
    }


    /** Compute a value of current coefficient to be used to mulitiple by rawData obained from CSA output in order to getting real current value in [mA]
     * @param empty
     * @returns coefficient that you can used to get real value of measured current depending on muxReg value
     *
     */
    inline float getCSACurrentCoeffmA(void) {
        float coeff=1;
        switch(mMuxReg) {
            case CHANNEL_A_0:    /*gain = 1x*/
                coeff=1.075;
                break;           
            case CHANNEL_A_1:    /*gain = 4x*/
                coeff=0.269;
                break;
            case CHANNEL_A_2:    /*gain = 8x*/
                coeff=0.134;
                break;
            default:                
                break;
        }
        return coeff;
    }

    /**********protected members********************************/
protected:

    int mI2cAddr;
    float mTemperature;
    float mCurrentSenseAmplifierOutput;
    uint16_t mCsaCurrentValueOffset;  //this parameter depends on your sensor
    uint8_t mMuxReg;//


    /**********public methods********************************/
public:

    /** Create an MAX9611 instance
     * @param address: I2C slave address
     */
    MAX9611(int address = MAX9611_I2C_ADDRESS);

    /** Initialization: set member values and configuration registers, ought to be invoked in the body of constructor
     * @returns
     *    true on success,
     *    false on error
     */
    bool initMax9611(eCtrlReg1MUX mux= CHANNEL_A_1,
                     eCtrlReg1SHDN shdn= NORMAL_OPERATION_SHDN,
                     eCtrlReg1LR lr=NORMAL_OPERATION_LR,
                     eCtrlReg1MODE mode= NORMAL_OPERATION_MODE,
                     eCtrlReg2DTIM watchdogDelay= _1MS,
                     eCtrlReg2RTIM watchdogRetryDelay=_50MS);


    /** Read temperature from the MAX9611.
     * @param none
     * @returns
     *   1 on success,
     *   0 on error
     */
    bool readTemp(void);


    /** Get temperature from the last measurement
     *
     * @returns
     *   temperature (C)
     */
    inline float getTemp(void) {
        return mTemperature;
    }


    /** Read CSA output value from the MAX9611.
      * @param none
      * @returns
      *   1 on success,
      *   0 on error
      */
    bool readCSAOutputValue(void);


    /** Get value of CSA output from the last measurement
      *
      * @returns
      *   Current Value [mA]
      */
    inline float getCSAOutput(void) {
        return mCurrentSenseAmplifierOutput;
    }

    //DEBUG
    uint16_t mRawInt;
    uint16_t readRawControl(void);
    uint16_t readRawCSAOutValue(void);
    uint16_t readRawRsValue(void);
    uint16_t readRawOutValue(void);



};

#endif






/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////.
///////////////////////////////////////.cpp file////////////////////////////////////////////////////////////////////.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////.
#include "max9611.h"
#include<Wire.h>

MAX9611::MAX9611(int address):mI2cAddr(address)
{   
    Wire.begin(mI2cAddr);
    if(!initMax9611()); //while(1){ //TODO handle error}
    mTemperature=0;
    mCsaCurrentValueOffset=0x0a;  //NOTE! set this parameter on your own, it depends on your used sensor
}



//read data
bool MAX9611::read(uint8_t regAddress, uint8_t *data,int dataLength)
{
     uint16_t timeout=1000; //1s you can use it or not : )
     uint8_t counter = 0;
     Wire.beginTransmission(mI2cAddr);
     Wire.write(regAddress);
     Wire.endTransmission();
     Wire.beginTransmission(mI2cAddr);
     Wire.requestFrom(mI2cAddr, (uint8_t)dataLength);
        
     for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); counter++) {
            data[count] = Wire.read();
     }
     // check for timeout or error
     if (counter < dataLength) return false; // timeout or some error occured
     else return true;
}

//write data to the sensor
bool MAX9611::write(uint8_t regAddress, uint8_t* data,int dataLength) {

    uint8_t status = 0;
    Wire.beginTransmission(mI2cAddr);
    Wire.write((uint8_t) regAddress); // addr
    for (uint8_t i = 0; i < dataLength; i++) {
            Wire.write((uint8_t) data[i]);
    }
    status = Wire.endTransmission();   
    return status == 0; //(true = success)
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



}



//and here quick demo app for arduino 




#include "max9611.h"


MAX9611 max9611();
void setup()
{
  Wire.begin(MAX9611_I2C_ADDRESS);        
  Serial.begin(9600);  
}

void loop()
{
   if(!max9611.readCSAOutputValue()){
       Serial.printf("MAX9611_CSA_Reading ERROR!- check all connections\r\n");
   }
   else{
       Serial.printf("MAX9611_CSA  %5.2f [mA]\r\n", max9611.getCSAOutput());
   }

   if(!max9611.readTemp()){
       Serial.printf("MAX9611_TEMP_Reading ERROR!- check all connections\r\n");
   }
   else{
       Serial.printf("MAX9611_TEMP:  %5.2f [C]\r\n", max9611.getTemp());
   }
}

