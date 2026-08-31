/**
 *  @brief:   SI7020 relative humidity and temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __SI7020_ARDUINO_H__
#define __SI7020_ARDUINO_H__

#include "../../si7020.h"
#include <Arduino.h>
#include <Wire.h>

/**
 * @brief SI7020 arduino specific dev object
 */
typedef struct
{
    TwoWire* i2c;      // I2C bus instance, e.g. &Wire
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
} si7020_arduino;

/**
 * @brief Initialize SI7020 device with ARDUINO dependent parameters.
 *
 * @param dev         si7020 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool si7020_arduino_init(si7020* const dev, si7020_arduino* const arduino_dev);


#endif // __SI7020_ARDUINO_H__
