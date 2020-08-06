
/**
 *  @brief:  Implementation of a DS3231 platform dependent [ARDUINO] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-12-02
 */


#ifndef __DS3231_ARDUINO_H__
#define __DS3231_ARDUINO_H__

#include "../../ds3231.h"
#include <Arduino.h>
#include <Wire.h>

/** DS3231 arduino specific dev object **/
typedef struct
{
    TwoWire* i2c;
} ds3231_arduino;

void ds3231_arduino_init(ds3231* const dev, ds3231_arduino* const arduino_dev);

void ds3231_arduino_deinit(ds3231* const dev);

#endif // __DS3231_ARDUINO_H__