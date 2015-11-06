/*
  @file MAX9611.h

  @brief MAX9611 High-Side, Current-Sense Amplifiers with
        12-Bit ADC and Op Amp/Comparator- Breakout I2C Library

  @Author lukasz uszko(luszko@op.pl)

  Tested on FRDM-KL46Z and FRDM-KL25Z

  Copyright (c) 2014 luszko
  Released under the MIT License (see http://mbed.org/license/mit)

  Documentation regarding the MAX9611 might be found here:
  http://datasheets.maximintegrated.com/en/ds/MAX9611-MAX9612.pdf
*/

//DEMO - HOW TO USE:
// NOTE before first use, do not forget to set mCsaCurrentValueOffset in constructor, it is a value that you can obtain from method: uint16_t readRawCSAOutValue(void),
// when a current value flowing through shunt resistor equals 0.
/*
---------------------------------------- DEMO: 1 version ----------------------------------------
#include "mbed.h"
#include "max9611.h"

#define MAX9611_PIN_SDA PTC9    // I2C0    //for example
#define MAX9611_PIN_SCL PTC8
int main()
{
    MAX9611 max9611(MAX9611_PIN_SDA, MAX9611_PIN_SCL);
    Serial debug(USBTX, USBRX);
    debug.baud(115200);
    while(1) {
        if(!max9611.readCSAOutputValue()){
            debug.printf("MAX9611_CSA_Reading ERROR!- check all connections\r\n");
        }
        else{
            debug.printf("MAX9611_CSA  %5.2f [mA]\r\n", max9611.getCSAOutput());
        }

        if(!max9611.readTemp()){
            debug.printf("MAX9611_TEMP_Reading ERROR!- check all connections\r\n");
        }
        else{
            debug.printf("MAX9611_TEMP:  %5.2f [C]\r\n", max9611.getTemp());
        }


    }
    return 0;
}
*/





#ifndef MAX9611_H
#define MAX9611_H

#include "mbed.h"

#define MAX9611_I2C_ADDRESS 0xE1   //A0 and A1 PIN are conected to GND , Write address 0xE0, Read Address 0xE1






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

    I2C mI2c;
    int mI2cAddr;
    float mTemperature;
    float mCurrentSenseAmplifierOutput;
    uint16_t mCsaCurrentValueOffset;  //this parameter depends on your sensor
    uint8_t mMuxReg;//


    /**********public methods********************************/
public:

    /** Create an MAX9611 instance
     * @param sda pin
     * @param scl pin
     * @param address: I2C slave address
     */
    MAX9611(PinName sda, PinName scl,int i2cFrequency=100000,int address = MAX9611_I2C_ADDRESS);


    /** Create a MAX9611 instance
     * @param i2c object
     * @param address: I2C slave address
     */
    MAX9611(I2C& i2c, int address = MAX9611_I2C_ADDRESS);


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