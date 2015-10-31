/*
  @file ds2782.h
  
  @brief DS2782 Stand-Alone Fuel Gauge IC
         Breakout I2C Library      

  @Author lukasz uszko(luszko@op.pl)

  Tested on FRDM-KL46Z and FRDM-KL25Z
  
  Copyright (c) 2014 lukasz uszko
  Released under the MIT License (see http://mbed.org/license/mit)

  Documentation regarding the MAX9611 might be found here: 
  http://www.maximintegrated.com/en/products/power/battery-management/DS2782.html
  
  and some very useful tutorials:
  http://www.maximintegrated.com/en/products/power/battery-management/DS2782.html/tb_tab2
  http://www.maximintegrated.com/en/app-notes/index.mvp/id/3584
  http://www.maximintegrated.com/en/app-notes/index.mvp/id/3463
  
*/


#ifndef DS2782_H
#define DS2782_H

#include "mbed.h"

#define DS2782_I2C_ADDRESS 0x34<<1  



 


class DS2782{
      
      
 /**********private members and methods********************************/       
 private: 
     
  typedef enum {
    STATUS=0x01,     /*STATUS - Status Register*/
    RAAC_MSB_REG,        /*RAAC - Remaining Active Absolute Capacity MSB*/
    RAAC_LSB_REG,       
    RSAC_MSB_REG,        /*RSAC - Remaining Standby Absolute Capacity MSB*/
    RSAC_LSB_REG,     
    RARC_REG,            /*RARC - Remaining Active Relative Capacity*/
    RSRC_REG,            /*RSRC - Remaining Standby Relative Capacity*/
    IAVG_MSB_REG,        /*IAVG - Average Current Register MSB*/
    IAVG_LSB_REG,     
    TEMP_MSB_REG,        /*TEMP - Temperature Register MSB*/
    TEMP_LSB_REG, 
    VOLT_MSB_REG,        /*VOLT - Voltage Register MSB*/
    VOLT_LSB_REG,      
    CURRENT_MSB_REG,     /*CURRENT - Current Register MSB*/
    CURRENT_LSB_REG,    
    ACR_MSB_REG,         /*ACR - Accumulated Current Register MSB*/
    ACR_LSB_REG,       
    ACRL_MSB_REG,        /*Low Accumulated Current Register MSB*/
    ACRL_LSB_REG,     
    AS_REG,              /*AS - Age Scalar*/
    SFR_REG,             /*SFR - Special Feature Register*/
    FULL_MSB_REG,        /*FULL - Full Capacity MSB*/
    FULL_LSB_REG,       
    AE_MSB_REG,          /*AE - Active Empty MSB*/
    AE_LSB_REG,    
    SE_MSB_REG,          /*SE - Standby Empty MSB*/
    SE_LSB_REG,
    EEPROM_REG= 0x1F,         /*EEPROM - EEPROM Register */
    USR_EEPROM_REG= 0x20,     /*User EEPROM, Lockable, Block 0 [20 to 2F]*/
    ADD_USR_EEPROM_REG=0x30,  /*Additional User EEPROM, Lockable, Block 0 [30 to 37]*/
    PARAM_EEPROM_REG=0x60,    /*Parameter EEPROM, Lockable, Block 1 [60 to 7F]*/
    UNIQUE_ID_REG =0xF0,      /*Unique ID [F0 to F7]*/
    FUNC_COMMAND_REG= 0xFE    /*Function Command Register */
         
 }RegAddr;
 
 typedef enum {
     
   CONTROL = 0x60,   //Control Register
   AB =0x61,         //Accumulation Bias
   AC_MSB = 0x62,    //Aging Capacity MSB
   AC_LSB = 0x63,    //Aging Capacity LSB
   VCHG = 0x64,      //Charge Voltage
   IMIN =0x65,       //Minimum Charge Current
   VAE = 0x66,       //Active Empty Voltage
   IAE = 0x67,       //Active Empty Current
   ACTIVE_EMPTY_40, 
   RSNSP,     //Sense Resistor Prime
   FULL_40_MSB,
   FULL_40_LSB,
   FULL_3040_SLOPE,
   FULL_2030_SLOPE,
   FULL_1020_SLOPE,
   FULL_0010_SLOPE,
   AE_3040_SLOPE,
   AE_2030_SLOPE,
   AE_1020_SLOPE,
   AE_0010_SLOPE,
   SE_3040_SLOPE,
   SE_2030_SLOPE,
   SE_1020_SLOPE,
   SE_0010_SLOPE,
   RSGAIN_MSB,          //Sense Resistor Gain MSB
   RSGAIN_LSB,          //Sense Resistor Gain LSB
   RSTC,                //Sense Resistor Temp. Coeff.
   FRSGAIN_MSB,         //Factory Gain MSB
   FRSGAIN_LSB,         //Factory Gain LSB
   I2C_SLAVE_ADDR= 0x7E //2-Wire Slave Address     
 }ParamEepromReg;
 
 
 

 
 
 
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
    
    /** divide 16 bit word to 2 8bit bytes
    * @param 1st byte
    * @param buf
    */
    inline void fillBuf(uint16_t varVal, uint8_t* buf){
        buf[0]= ((varVal>>8)&0xFF);
        buf[1]= ((varVal)&0xFF);
    }
        
    I2C mI2c;   
    uint8_t mI2cAddr;
   
    
    /**********protected members and methods********************************/
    protected:
    float mTemperature; 
    float mCurrent;  
    float mVoltage;
   
   /**********public methods********************************/
    public:
    
    typedef enum {
        PORF = 0x02,     //Power-On Reset Flag – Useful for reset detection, see text below.
        UVF =0x04,       //Under-Voltage Flag
        LEARNF = 0x10,   //Learn Flag – When set to 1, a charge cycle can be used to learn battery capacity.
        SEF = 0x20,      //Standby Empty Flag
        AEF = 0x40,      //Active Empty Flag
        CHGTF =0x80,     //Charge Termination Flag
    }StatusReg ;

    /** Create an SI7020 instance
     * @param sda pin 
     * @param scl pin 
     * @param address: I2C slave address 
     */
     DS2782(PinName sda, PinName scl,int i2cFrequencyHz=100000,uint8_t address = DS2782_I2C_ADDRESS); 


    /** Initialization: set member values and configuration registers, ought to be invoked in the body of constructor 
     * @returns
     *    true on success,
     *    false on error
     */
    bool initDS2782(void);
    
    /** Read temperature from the sensor.
     * @param none
     * @returns
     *   1 on success,
     *   0 on error
     */ 
    bool readTemperature(void);   
    bool readCurrent(void);
    bool readVoltage(void);
    bool setACRRegister(uint16_t reg);  // set to 0 clears LEARNF and other flags
    bool setEepromBlockRegister(ParamEepromReg reg, uint8_t * value, uint8_t length);
    uint8_t readStatusReg(void);
    float readAcrReg(void);
    uint8_t readRarcReg(void);
    
    
    // setters-getters
    float getTemperature(void);
    float getCurrent(void);
    float getVoltage(void);

};

#endif