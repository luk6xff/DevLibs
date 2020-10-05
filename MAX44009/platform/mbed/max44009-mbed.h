/**
 *  @brief:   MAX44009 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MAX44009_MBED_H__
#define __MAX44009_MBED_H__

#include "mbed.h"
#include "../../max44009.h"


typedef struct
{
    I2C* i2c;       // I2C Interface
    DigitalOut* wp; // AT24Cxx Write protection pin
} max44009_mbed;


/**
 * @brief Initialize MAX44009 device with MBED dependent parameters.
 */
bool max44009_mbed_init(max44009* const dev, max44009_mbed* const mbed_dev);


#endif /*__MAX44009_MBED_H__ */

