#include "US-015.h"



US015::US015(PinName triggerPin, PinName echoPin, uint32_t timeout):trigger(triggerPin),echoIn(echoPin),timeoutValue(timeout)
{
    echoIn.rise(this, &US015::startMeasureCallback);
    echoIn.fall(this, &US015::stopMeasureCallback);
    timerCtrl = new TimeController();
    measurementRunning=false;
    measuremntFinishedCallback= NULL;
}

int US015::convertMaxDistanceToMaxTimeout(int maxDistHandledBySensorInCm){
    return (maxDistHandledBySensorInCm) / (float)((SPEED_OF_SOUND)/10000)/2;
}

void US015::setFinishCallback(void (*finishCallback)(int resultValue))
{
    measuremntFinishedCallback= finishCallback;
}

bool US015::doMeasurement(void)
{
    if(isMeasurementRuning())
        return false;
    timerCtrl->startTime=0;
    trigger.write(1);  //triggerPin
    wait_us(30);
    trigger.write(0);
    measurementRunning=true;
    return true;
}

bool US015::isMeasurementRuning()
{
    return measurementRunning;
}

int US015::convertTimeToDistanceValue_mm(int timeUs)
{
    return (int)((timeUs) * (float)((SPEED_OF_SOUND)/1000))/2;  //(0.343mm/us)
}

int US015::convertTimeToDistanceValue_cm(int timeUs)
{
    return ((timeUs) *(float)((SPEED_OF_SOUND)/10000))/2; //(0.034cm/us)
}

int US015::getTimePassedValue(void)
{
    return timerCtrl->timer.read_us()-timerCtrl->startTime;
}

int US015::getTimeoutValue(void)
{
    return timeoutValue;
}

void US015::resetMeasuremnt(void)
{
    timerCtrl->timer.reset();
    measurementRunning=false;
}

//private methods
void US015::startMeasureCallback(void)
{
    timerCtrl->timer.start();
    timerCtrl->startTime= timerCtrl->timer.read_us();
}

void US015::stopMeasureCallback(void)
{
    timerCtrl->difTime= timerCtrl->timer.read_us()-timerCtrl->startTime;
    resetMeasuremnt();
    if(measuremntFinishedCallback!=NULL) 
    {
        (*measuremntFinishedCallback)(convertTimeToDistanceValue_mm(timerCtrl->difTime)); //for default
    }
}

