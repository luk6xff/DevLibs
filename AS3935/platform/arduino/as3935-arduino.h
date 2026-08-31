/**
 *  @brief:   AS3935 Franklin lightning sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __AS3935_ARDUINO_H__
#define __AS3935_ARDUINO_H__

#include "../../as3935.h"
#include <Arduino.h>
#include <Wire.h>

/**
 * @brief AS3935 arduino specific dev object
 */
typedef struct
{
    TwoWire* i2c;      // I2C bus instance, e.g. &Wire
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
    int irq_pin;       // Digital pin the sensor IRQ line is connected to
} as3935_arduino;

/**
 * @brief Initialize AS3935 device with ARDUINO dependent parameters.
 *
 * @param dev         as3935 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool as3935_arduino_init(as3935* const dev, as3935_arduino* const arduino_dev);


#endif // __AS3935_ARDUINO_H__
