/**
 *  @brief:   DS2782 stand-alone battery fuel gauge library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __DS2782_ARDUINO_H__
#define __DS2782_ARDUINO_H__

#include "../../ds2782.h"
#include <Arduino.h>
#include <Wire.h>

/**
 * @brief DS2782 arduino specific dev object
 */
typedef struct
{
    TwoWire* i2c;      // I2C bus instance, e.g. &Wire
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
} ds2782_arduino;

/**
 * @brief Initialize DS2782 device with ARDUINO dependent parameters.
 *
 * @param dev         ds2782 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool ds2782_arduino_init(ds2782* const dev, ds2782_arduino* const arduino_dev);


#endif // __DS2782_ARDUINO_H__
