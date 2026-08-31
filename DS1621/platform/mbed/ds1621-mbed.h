/**
 *  @brief:   DS1621 digital thermometer and thermostat library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __DS1621_MBED_H__
#define __DS1621_MBED_H__

#include "mbed.h"
#include "../../ds1621.h"

/**
 * @brief DS1621 mbed specific dev object
 */
typedef struct
{
    I2C* i2c;          // I2C bus instance
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
} ds1621_mbed;


/**
 * @brief Initialize DS1621 device with MBED dependent parameters.
 *
 * @param dev      ds1621 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool ds1621_mbed_init(ds1621* const dev, ds1621_mbed* const mbed_dev);


#endif /*__DS1621_MBED_H__ */
