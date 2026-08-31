/**
 *  @brief:   DS1621 digital thermometer and thermostat library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __DS1621_ARDUINO_H__
#define __DS1621_ARDUINO_H__

#include "../../ds1621.h"
#include <Arduino.h>
#include <Wire.h>

/**
 * @brief DS1621 arduino specific dev object
 */
typedef struct
{
    TwoWire* i2c;      // I2C bus instance, e.g. &Wire
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
} ds1621_arduino;

/**
 * @brief Initialize DS1621 device with ARDUINO dependent parameters.
 *
 * @param dev         ds1621 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool ds1621_arduino_init(ds1621* const dev, ds1621_arduino* const arduino_dev);


#endif // __DS1621_ARDUINO_H__
