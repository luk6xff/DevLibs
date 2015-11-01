/*
  @file adt7410.h

  @brief Temperature Sensor ADT7410 Breakout I2C Library

  @Author lukasz uszko(luszko@op.pl)

  Tested on FRDM-KL46Z

  Copyright (c) 2014 luszko
  Released under the MIT License (see http://mbed.org/license/mit)

  Documentation regarding the ADT7410 can be found here:
  http://www.analog.com/static/imported-files/data_sheets/ADT7410.pdf
*/



#ifndef ADT7410_H
#define ADT7410_H

#include "mbed.h"


#define ADT7410_I2C_ADDRESS 0x97   //A0 and A1 PIN are conected to VDD
#define ADT7410_2_I2C_ADDRESS 0x90   //A0 and A1 PIN are conected to GND


#define UNSET_ADT7410_TEMPERATURE -999




class ADT7410
{

public:
    //typedefs:

    typedef enum {
        _13_BIT=0,
        _16_BIT=1
    } CONF_RESOLUTION;

    typedef enum {
        CONT_CONV=0,
        ONE_SHOT,
        SPS_MODE,
        SHUTDOWN
    } CONF_OPERATION_MODE;

    typedef enum {
        INTERRUPT_MODE=0,
        COMPARATOR_MODE=1
    } CONF_INT_CT_MODE;

    typedef enum {
        INT_ACTIVE_LOW=0,
        INT_ACTIVE_HIGH=1
    } CONF_INT_PIN_POLARITY;

    typedef enum {
        CT_ACTIVE_LOW=0,
        CT_ACTIVE_HIGH=1
    } CONF_CT_PIN_POLARITY;

    typedef enum {
        _1_FAULT=0,
        _2_FAULTS,
        _3_FAULTS,
        _4_FAULTS
    } CONF_FAULT_QUEUE;

    /** Create an ADT7410 instance
     * @param sda pin
     * @param scl pin
     * @param address: I2C slave address
     */
    ADT7410(PinName sda, PinName scl,int i2cFrequency=100000,int address = ADT7410_I2C_ADDRESS);

    /** Create a ADT7410 instance
     * @param i2c object
     * @param address: I2C slave address
     */
    ADT7410(I2C& i2c, int address = ADT7410_I2C_ADDRESS);

    /** Initialization: set member values and
     * @returns
     *    1 on success,
     *    0 on error
     */
    bool setConfiguration(CONF_FAULT_QUEUE faultQueue=_1_FAULT,
                          CONF_CT_PIN_POLARITY ctPinPolarity=CT_ACTIVE_LOW,
                          CONF_INT_PIN_POLARITY intPinPolarity=INT_ACTIVE_LOW,
                          CONF_INT_CT_MODE intCtMode=INTERRUPT_MODE,
                          CONF_OPERATION_MODE operMode=CONT_CONV,
                          CONF_RESOLUTION res=_16_BIT);

    /** Read temperature from the ADT7410.
     * @param temperature (C)
     * @returns
     *   1 on success,
     *   0 on error
     */
    bool readTemp();


    /** Read ID  number of the chip.
     * @param temperature (C)
     * @returns ID number (8 bit Value)
     */
    int readIdNumber(void);


    /** Set resolution of read data
     * @returns
     *   1 on success,
     *   0 on error
     */
    bool setResolution(CONF_RESOLUTION res);

    /** Get temperature from a previous measurement
     *
     * @returns
     *   temperature (C)
     */
    inline float getTemperature(void) {
        return mTemperature;
    }

protected:

    float mTemperature;
    I2C mI2c;
    int mI2cAddr;
    uint8_t mResolution;

private:

    /** Write data to the given register (using I2C bus)
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



};

#endif