#include "ltc2460.h"


LTC2460::LTC2460(PinName csPin , PinName mosiPin, PinName misoPin, PinName sckPin,double resDividerVal ): mCSpin(csPin), mSpi(mosiPin, misoPin, sckPin){
    
     mResDividerVal= resDividerVal ;
    if(!initLTC2460()){}; //while(1); //TODO handle error

}   
    
    
bool LTC2460::initLTC2460(void){     
   //  uint8_t  data;
    // Setup the spi for 16 bit data
    // second edge capture, with a 1MHz clock rate
     mSpi.format(16,0);
     mSpi.frequency(1000000);
     return true;
}


uint16_t LTC2460::spiRead(void)
{
    __disable_irq();    // Disable Interrupts
    mCSpin=0;
    uint16_t val = mSpi.write(0); // The written value is ignored, reg value is read
    mCSpin = 1;
    __enable_irq();     // Enable Interrupts
    return val;
}

void LTC2460::spiWrite(uint16_t value)
{
    __disable_irq();    // Disable Interrupts
    mCSpin = 0;
    mSpi.write(value); // New value follows
    mCSpin= 1;
    __enable_irq();     // Enable Interrupts
}

float LTC2460::readVoltage()
{  
  
   uint16_t rawData=spiRead() ;
   float temp = (float)((1.25/0xFFFF)*rawData);
   float val = temp*mResDividerVal+temp;
   return val;
}


