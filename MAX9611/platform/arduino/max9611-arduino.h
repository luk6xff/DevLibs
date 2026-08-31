/**
 *  @brief:   MAX9611 high-side current-sense amplifier with 12-bit ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MAX9611_ARDUINO_H__
#define __MAX9611_ARDUINO_H__

#include "../../max9611.h"
#include <Arduino.h>
#include <Wire.h>

/**
 * @brief MAX9611 arduino specific dev object
 */
typedef struct
{
    TwoWire* i2c;      // I2C bus instance, e.g. &Wire
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
} max9611_arduino;

/**
 * @brief Initialize MAX9611 device with ARDUINO dependent parameters.
 *
 * @param dev         max9611 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool max9611_arduino_init(max9611* const dev,
                          max9611_arduino* const arduino_dev);


#endif // __MAX9611_ARDUINO_H__
