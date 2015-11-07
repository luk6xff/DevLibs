/*
  @file max44009.h
  
  @brief MAX44009 ambient light sensor with an I²C digital output Breakout I2C Library      

  @Author lukasz uszko(luszko@op.pl)

  Tested on FRDM-KL46Z and FRDM-KL25Z
  
  Copyright (c) 2014 lukasz uszko
  Released under the MIT License (see http://mbed.org/license/mit)

  Documentation regarding the MAX9611 might be found here: 
  http://www.maximintegrated.com/en/products/analog/sensors-and-sensor-interface/MAX44009.html
*/








#ifndef MAX44009_H
#define MAX44009_H

#include "mbed.h"



#define MAX44009_I2C_ADDRESS 0x97  //1001 1011 -- A0 PIN is conected to VDD
#define UNSET_MAX44009_LUX_INTENSITY_VALUE -999 



class MAX44009{   
    
    
    
 public:   
 
   /** Create an MAX44009 instance
     * @param sda pin 
     * @param scl pin 
     * @param address: I2C slave address 
     */
    MAX44009(PinName sda, PinName scl,int i2cFrequency=100000,int address = MAX44009_I2C_ADDRESS); 

    /** Create a MAX44009 instance
     * @param i2c object
     * @param address: I2C slave address 
     */
    MAX44009(I2C& i2c, int address = MAX44009_I2C_ADDRESS); 
   
    int getStatus(void);
    
    bool readLuxIntensity(void);
    
    
    inline float getLuxIntensity(void){
        return this->mLuxIntensity;
        }
    
 private:
    
    I2C mI2c;   
    int mI2cAddr;
    float mLuxIntensity;
 
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
    bool read(uint8_t regAddress, uint8_t *data,int dataLength);  


    
    

};


#endif 