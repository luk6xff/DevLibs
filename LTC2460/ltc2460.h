/*
  @file LTC2460.h
  
  @brief LTC2460 - Ultra-Tiny, 16-Bit ΔΣ ADCs with 10ppm/°C Max Precision Reference
         Breakout SPI Library      

  @Author lukasz uszko(luszko@op.pl)

  Tested on FRDM-KL46Z and FRDM-KL25Z
  
  Copyright (c) 2015 lukasz uszko
  Released under the MIT License (see http://mbed.org/license/mit)

  Documentation regarding the  LTC2460 might be found here: 
  http://www.linear.com/product/LTC2460
*/



#ifndef LTC2460_H
#define LTC2460_H

#include "mbed.h"


class LTC2460{
      
      
 /**********private members and methods********************************/       
 private: 
 
        DigitalOut          mCSpin;
        SPI                 mSpi;
        double mResDividerVal;


   /**********public methods********************************/
    public:

    /** Constructor- Creates an LTC2460 instance
     * @param cs pin 
     * @param mosi pin 
     * @param miso pin
     * @param sck  pin
     */
    LTC2460(PinName csPin , PinName mosiPin, PinName misoPin, PinName sckPin,double resDividerVal=0 ); 


    /** Initialization: set member values and configuration registers, ought to be invoked in the body of constructor 
     * @returns
     *    true on success,
     *    false on error
     */
    bool initLTC2460(void);


    /*reading , writing registers
    
    */
    uint16_t spiRead(void);

    void spiWrite(uint16_t value);

    /** Read volatge real value from ADC , with .
     * @param resistor divider value: GND---- R1 ----- ADC-----R2--- Vmeasured    --> resDividerVal = R1/R2
     * @returns
     *   value of voltage measured voltage 
     */    
    float readVoltage();
   

    
};

#endif