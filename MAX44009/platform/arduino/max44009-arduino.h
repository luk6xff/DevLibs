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
    TwoWire* i2c;      // I2C bus instance, e.g. &Wire
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
} max44009_arduino;

/**
 * @brief Initialize MAX44009 device with ARDUINO dependent parameters.
 *
 * @param dev         max44009 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool max44009_arduino_init(max44009* const dev,
                           max44009_arduino* const arduino_dev);


#endif // __MAX44009_ARDUINO_H__
