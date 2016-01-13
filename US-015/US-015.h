/*
  @file US-015.h
  
  @brief library for US-015 (or HC-SR04) Ultrasonic Module Distance Measuring Sensor

  @Author lukasz uszko(luszko@op.pl)

  Tested on most mbed platforms
  
  Copyright (c) 2016 lukasz uszko
  Released under the MIT License (see http://mbed.org/license/mit)

  Documentation regarding the US-015 sensor can be found here: 
  http://www.micropik.com/PDF/HCSR04.pdf
*/
//DEMO - HOW TO USE:
/*
---------------------------------------- DEMO 1:----------------------------------------
#include <mbed.h>
#include "US-015.h"
 
 
#define TRIGGER_PIN PC_1
#define ECHO_PIN PC_0
Serial debugPort(SERIAL_TX, SERIAL_RX); //Default 9600 bauds, 8-bit data, no parity
 
void measurementFinished(int resultVal){
   debugPort.printf("Distance: %d",resultVal);
}
int main()
{
    
    US015 us015(TRIGGER_PIN,ECHO_PIN,US015::convertMaxDistanceToMaxTimeout(MAX_DISTANCE_FOR_US015_SENSOR_CM) );
    us015.setFinishCallback(measurementFinished);
    bool timeoutFlag=true;
    while (1) {
        wait_ms(1000);  
        us015.doMeasurement();
        while(us015.getTimePassedValue()<us015.getTimeoutValue())
        {
            if(us015.isMeasurementRuning())
            {
                timeoutFlag=true;
            }
            else
            {
                timeoutFlag =false;
                break;
            }
        }  
        if(timeoutFlag){
            debugPort.printf("Timeout Happened");
            us015.resetMeasuremnt();
        }
        
    }
}
*/



#include <mbed.h>

#define DEFAULT_TIMEOUT 0xFFFFFFFF
#define MAX_DISTANCE_FOR_US015_SENSOR_CM  400  //400cm
#define SPEED_OF_SOUND 343.2f
class US015{
    
public:

    struct TimeController {
        Timer timer;
        int startTime;
        int difTime;
    };
    
    US015(PinName triggerPin, PinName echoPin,uint32_t timeout=DEFAULT_TIMEOUT);
    static int convertMaxDistanceToMaxTimeout(int maxDistHandledBySensorInCm);
    bool doMeasurement(void);    
    int convertTimeToDistanceValue_mm(int timeUs);
    int convertTimeToDistanceValue_cm(int timeUs);
    void setFinishCallback(void (*finishCallback)(int resultValue));
    bool isMeasurementRuning(void );
    int getTimePassedValue(void);
    int getTimeoutValue(void);
    void resetMeasuremnt(void);

private:    
    void startMeasureCallback(void);
    void stopMeasureCallback(void);
    void (*measuremntFinishedCallback)(int result);
    
private:
    bool measurementRunning;
    DigitalOut trigger;
    InterruptIn echoIn;
    uint32_t timeoutValue;
    struct TimeController* timerCtrl;
    
};