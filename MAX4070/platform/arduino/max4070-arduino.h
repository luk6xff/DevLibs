/**
 *  @brief:   MAX4070 bidirectional high-side current-sense amplifier library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MAX4070_ARDUINO_H__
#define __MAX4070_ARDUINO_H__

#include "../../max4070.h"
#include <Arduino.h>

/**
 * @brief MAX4070 arduino specific dev object
 */
typedef struct
{
    int adc_pin; // Analog pin wired to the amplifier OUT pin, e.g. A0
} max4070_arduino;

/**
 * @brief Initialize MAX4070 device with ARDUINO dependent parameters.
 *
 * @param dev         max4070 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool max4070_arduino_init(max4070* const dev,
                          max4070_arduino* const arduino_dev);


#endif // __MAX4070_ARDUINO_H__
