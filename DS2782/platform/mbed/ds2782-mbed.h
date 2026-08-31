/**
 *  @brief:   DS2782 stand-alone battery fuel gauge library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __DS2782_MBED_H__
#define __DS2782_MBED_H__

#include "mbed.h"
#include "../../ds2782.h"

/**
 * @brief DS2782 mbed specific dev object
 */
typedef struct
{
    I2C* i2c;          // I2C bus instance
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
} ds2782_mbed;

/**
 * @brief Initialize DS2782 device with MBED dependent parameters.
 *
 * @param dev      ds2782 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool ds2782_mbed_init(ds2782* const dev, ds2782_mbed* const mbed_dev);


#endif /* __DS2782_MBED_H__ */
