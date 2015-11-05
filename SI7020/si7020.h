/*
  @file SI7020.h
  
  @brief SI7020 HUMIDITY AND TEMPERATURE SENSOR
         Breakout I2C Library      

  @Author lukasz uszko(luszko@op.pl)

  Tested on FRDM-KL46Z and FRDM-KL25Z
  
  Copyright (c) 2014 lukasz uszko
  Released under the MIT License (see http://mbed.org/license/mit)

  Documentation regarding the MAX9611 might be found here: 
  http://www.silabs.com/Support%20Documents/TechnicalDocs/Si7020.pdf
*/



#ifndef SI7020_H
#define SI7020_H

#include "mbed.h"

#define SI7020_I2C_ADDRESS 0x81   



 


class SI7020{
      
      
 /**********private members and methods********************************/       
 private: 
     
 typedef enum {
    HUMIDITY_HOLD=0xE5,     /*Measure Relative Humidity, Hold Master Mode*/
    HUMIDITY_NO_HOLD=0xF5,  /*Measure Relative Humidity, No Hold Master Mode*/
    TEMP_HOLD=0xE3,         /*Measure Temperature, Hold Master Mode*/
    TEMP_NO_HOLD=0xF3,      /*Measure Temperature, No Hold Master Mode*/
    TEMP_FROM_RH=0xE0,      /*Read Temperature Value from Previous RH Measurement*/
    RESET=0xFE,             /*Reset*/
    WRITE_RH_T=0xE6,        /*Write RH/T User Register 1*/
    READ_RH_T=0xE7,         /*Read RH/T User Register 1*/
    READ_ID_1BYTE=0xFA,     /*Read Electronic ID 1st Byte*/
    READ_ID_2BYTE=0xFC,     /*Read Electronic ID 2nd Byte*/
    READ_FIRMWARE_REV=0x84, /*Read Firmware Revision*/
 }eCommandAddr;
 
 //User Register
 typedef enum {
   RH12_TEMP14=0x00,        /*RH=12bit, TEMP=14bit and so on*/
   RH8_TEMP12 =0x01,
   RH10_TEMP13=0x80,
   RH11_TEMP11=0x81     
 }eMeasurementResolution;
 
typedef enum {
   VDD_OK=0,
   VDD_LOW=1
 }eVddStatus;
 
 typedef enum {
    HEATER_ENABLE=1,
    HEATER_DISABLE=0
 }eHeater;
  
 
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
    
    
   /** merge two bytes in one word
    * @param 1st byte
    * @param 2nd byte
    * @returns 16 bit word
    */
    inline uint16_t get16BitData(uint8_t msbByte,uint8_t lsbByte){
        uint16_t data16Bit= (msbByte<<8)|(lsbByte);
        return data16Bit; 
    }
       
        
    I2C mI2c;   
    uint8_t mI2cAddr;
   
    
    /**********protected methods********************************/
    protected:
       

    float mTemperature;  
    float mHumidity; 
   
   /**********public methods********************************/
    public:

    /** Create an SI7020 instance
     * @param sda pin 
     * @param scl pin 
     * @param address: I2C slave address 
     */
     SI7020(PinName sda, PinName scl,int i2cFrequencyHz=100000,uint8_t address = SI7020_I2C_ADDRESS); 


    /** Initialization: set member values and configuration registers, ought to be invoked in the body of constructor 
     * @returns
     *    true on success,
     *    false on error
     */
    bool initSi7020(eMeasurementResolution= RH8_TEMP12 ,eHeater=HEATER_DISABLE);


    /** Read temperature from the sensor.
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
   inline float getTemp(void) {return mTemperature;};
   
   
   /** Read Humidity value from the sensor.
     * @param none 
     * @returns
     *   1 on success,
     *   0 on error
     */    
    bool readHumidity(void);
   
   
   /** Get value of Humidity from the last measurement 
     *  
     * @returns
     *   Humidity Value [%]
     */    
   inline float getHumidity(void) {return mHumidity;}
   
   
    /** inoke reset command.
     * @param none 
     * @returns
     *   1 on success,
     *   0 on error
     */    
    bool resetSensor(void);
   
   

   //DEBUG
   uint16_t mRawInt;
   
   uint16_t readControl(void);
   uint16_t readRsValue(void);
   uint16_t readOutValue(void);
   

    
};

#endif