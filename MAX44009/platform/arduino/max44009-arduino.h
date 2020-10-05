/**
 *  @brief:   MAX44009 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MAX44009_ARDUINO_H__
#define __MAX44009_ARDUINO_H__

#include "../../max44009.h"
#include <Arduino.h>
#include <Wire.h>

/**
 * @brief MAX44009 arduino specific dev object
 */
typedef struct
{
    TwoWire* i2c;
} max44009_arduino;

/**
 * @brief Initialize MAX44009 device with ARDUINO dependent parameters.
 */
bool max44009_arduino_init(max44009* const dev, max44009_arduino* const arduino_dev);


#endif // __MAX44009_ARDUINO_H__