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

/**
 * @brief MAX44009 mbed specific dev object
 */
typedef struct
{
    I2C* i2c;          // I2C bus instance
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
} max44009_mbed;


/**
 * @brief Initialize MAX44009 device with MBED dependent parameters.
 *
 * @param dev      max44009 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool max44009_mbed_init(max44009* const dev, max44009_mbed* const mbed_dev);


#endif /*__MAX44009_MBED_H__ */


