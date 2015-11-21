/*
  @file AS3935.h
  
  @brief AS3935 Franklin Lightning Sensor
         Breakout I2C Library      

  @Author lukasz uszko(luszko@op.pl)

  Tested on FRDM-KL46Z and FRDM-KL25Z
  
  Copyright (c) 2014 lukasz uszko
  Released under the MIT License (see http://mbed.org/license/mit)

  Documentation regarding the AS3935 might be found here: 
  http://www.ams.com/eng/Products/Lightning-Sensor
*/


#ifndef _AS3935_H
#define _AS3935_H

#include "mbed.h"
#include "rtos.h"

#define AS3935_I2C_ADDRESS 0x03<<1



// register access macros - register address, bitmask
#define AS3935_AFE_GB       0x00, 0x3E
#define AS3935_PWD          0x00, 0x01
#define AS3935_NF_LEV       0x01, 0x70
#define AS3935_WDTH         0x01, 0x0F
#define AS3935_CL_STAT      0x02, 0x40
#define AS3935_MIN_NUM_LIGH 0x02, 0x30
#define AS3935_SREJ         0x02, 0x0F
#define AS3935_LCO_FDIV     0x03, 0xC0
#define AS3935_MASK_DIST    0x03, 0x20
#define AS3935_INT          0x03, 0x0F
#define AS3935_DISTANCE     0x07, 0x3F
#define AS3935_DISP_LCO     0x08, 0x80
#define AS3935_DISP_SRCO    0x08, 0x40
#define AS3935_DISP_TRCO    0x08, 0x20
#define AS3935_TUN_CAP      0x08, 0x0F

#define AS3935_TUN_CAP_VALUE 3 // set by the maker of the board -> Take a look at hardware antenna desing for more details

// other constants
#define AS3935_AFE_INDOOR   0x12
#define AS3935_AFE_OUTDOOR  0x0E


class AS3935 {
    
    
    
  /**********public methods********************************/
  public:

    /** Create an AS3935 instance
     * @param sda pin 
     * @param scl pin 
     * @param PinName IRQ Pin
     * @param i2c bus frequency [Hz]
     * @param address: I2C slave address  
     */
    AS3935(PinName sda, PinName scl, PinName irqPin,int i2cFrequencyHz=100000,uint8_t address=AS3935_I2C_ADDRESS); 
     
     
    bool setConfiguration(void);   
    
    void handleIrqInterrupt(void);
       
    bool reset();
    
    bool calibrate();
    
    bool powerDown();
    
    bool powerUp();
    
    int readInterruptSource();
    
    bool disableDisturbers();
    
    bool enableDisturbers();
    
    int getMinimumLightnings();
    
    bool setMinimumLightnings(int minlightning);
    
    int getLightningDistanceKm();
    
    bool setIndoors();
    
    bool setOutdoors();
    
    int getNoiseFloor();
    
    bool setNoiseFloor(int noisefloor);
    
    int getSpikeRejection();
    
    bool setSpikeRejection(int srej);
    
    int getWatchdogThreshold();
    
    int getTuneCap();
    
    bool setTuneCap(int cap);
    
    bool setWatchdogThreshold(int wdth);
    
    void clearStats(); 
    
    osEvent checkQueueState(void);
       
       
       
        
        
  /**********private members and methods********************************/       
  private: 
  
   typedef enum {
        NOISE_FLOOR_REG_ADDR=  0x01,
        NOISE_FLOOR_BIT_MASK=  0x70,
        NOISE_FLOOR_390out_28in_uVrms= 0x00,
        NOISE_FLOOR_630out_45in_uVrms,
        NOISE_FLOOR_860out_62in_uVrms,
        NOISE_FLOOR_1100out_78in_uVrms,
        NOISE_FLOOR_1140out_95in_uVrms,
        NOISE_FLOOR_1570out_112in_uVrms,
        NOISE_FLOOR_1800out_130in_uVrms,
        NOISE_FLOOR_2000out_146in_uVrms=0x07
   }NoiseFloorReg;
   
   typedef enum {
        DISTANCE_REG_ADDR=  0x07, 
        DISTANCE_BIT_MASK=  0x3F,
        DISTANCE_OUT_OF_RANGE=  0x3F,
        DISTANCE_STORM_IS_OVERHEAD=  0x01,
    }DistanceEstimationReg;
     
    typedef enum {
        INTERRUPTS_REG_ADDR=  0x03, 
        INTERRUPTS_BIT_MASK=  0x0F,
        INTERRUPTS_INT_NH=  0x01,    /* Noise level too high  */
        INTERRUPTS_INT_D=  0x04,     /* Disturber detected  */
        INTERRUPTS_INT_L= 0x08    /* Lightning interrupt  */
    }InterruptsReg;
    
    typedef enum {
        MIN_NUM_OF_LIGH_REG_ADDR=  0x02, 
        MIN_NUM_OF_LIGH_BIT_MASK=  0x30,
        MIN_NUM_OF_LIGH_1=  0x00,
        MIN_NUM_OF_LIGH_5=  0x01,
        MIN_NUM_OF_LIGH_9=  0x02,
        MIN_NUM_OF_LIGH_16=  0x03
    }NumOfLightningsReg; 
     
     typedef enum {
        LC_OSC_DIV_RATIO_REG_ADDR=  0x03, 
        LC_OSC_DIV_RATIO_BIT_MASK=  0xC0,
        LC_OSC_DIV_RATIO_16=  0x00,
        LC_OSC_DIV_RATIO_32=  0x01,
        LC_OSC_DIV_RATIO_64=  0x02,
        LC_OSC_DIV_RATIO_128=  0x03
    }AntennaTuningFreqDivisionRatio; 
     
     
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
    
    
    /** Write data into the given byte
     * @param register Address
     * @param dataBitMask in register
     * @param dat to be written
     * @returns
     *   1 on success,
     *   0 on error
     */
    bool writeReg(uint8_t regAddress, uint8_t dataBitMask, uint8_t data);
    
    /** Helper method for writeReg and readReg methods
     * @param uint8_t dataBitMask
     * @returns  ShiftValue
     */
    uint8_t getShiftValue(uint8_t dataBitMask);
    
    
    /** Read data from the given byte
     * @param register Address
     * @param dataBitMask in register
     * @param dat to be read
     * @returns
     *   1 on success,
     *   0 on error
     */
    bool readReg(uint8_t regAddress, uint8_t dataBitMask, uint8_t *data);
    
    
    I2C mI2c;   
    uint8_t mI2cAddr;
    InterruptIn mIrqPinInterrupt;  
    Queue<uint8_t, 10> *queue;           
        
};









#endif