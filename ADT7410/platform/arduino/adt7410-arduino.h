/**
 *  @brief:   ADT7410 digital temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __ADT7410_ARDUINO_H__
#define __ADT7410_ARDUINO_H__

#include "../../adt7410.h"
#include <Arduino.h>
#include <Wire.h>

/**
 * @brief ADT7410 arduino specific dev object
 */
typedef struct
{
    TwoWire* i2c;      // I2C bus instance, e.g. &Wire
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
} adt7410_arduino;

/**
 * @brief Initialize ADT7410 device with ARDUINO dependent parameters.
 *
 * @param dev         adt7410 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool adt7410_arduino_init(adt7410* const dev,
                          adt7410_arduino* const arduino_dev);


#endif // __ADT7410_ARDUINO_H__
