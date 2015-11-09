/*
  @brief MAX4070 Bidirectional, High-Side, Current-Sense Amplifier with Reference     

  @Author lukasz uszko(luszko@op.pl)

  Tested on FRDM-KL46Z and FRDM-KL25Z
  
  Copyright (c) 2014 lukasz uszko
  Released under the MIT License (see http://mbed.org/license/mit)

  Documentation regarding the MAX4070 might be found here: 
  http://datasheets.maximintegrated.com/en/ds/MAX4069-MAX4072.pdf

*/

#include "mbed.h"


class MAX4070 {
    
 public: 
     MAX4070(AnalogIn sensorInput, int resolution);
     float getResult(void);
     
     virtual int readValueFromInput(void);   
     virtual ~MAX4070(){};  
   
   
    
 protected:   
     AnalogIn  mSensorInput; 
     int mResolution; //it means how many (number of) milivolts we get on the output of the sensor for 1[mA]
     
     float result;
    
   
};

//divider: Vbat -> 1191[ohm] -> AIN -> 982[ohm] -> GND
class MAX4070Voltage: public MAX4070{
 public: 
    MAX4070Voltage(AnalogIn sensorInput, int resolution);
    int readValueFromInput(void);   
};