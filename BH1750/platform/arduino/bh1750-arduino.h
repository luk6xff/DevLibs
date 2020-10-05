/**
 *  @brief:   BH1750 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-10-02
 *  @license: MIT
 */

#ifndef __BH1750_ARDUINO_H__
#define __BH1750_ARDUINO_H__

#include "../../bh1750.h"
#include <Arduino.h>
#include <Wire.h>

/**
 * @brief BH1750 arduino specific dev object
 */
typedef struct
{
    TwoWire *i2c;
} bh1750_arduino;

/**
 * @brief Initialize BH1750 device with ARDUINO dependent parameters.
 */
bool bh1750_arduino_init(bh1750 *const dev, bh1750_arduino *const arduino_dev);


#endif // __BH1750_ARDUINO_H__