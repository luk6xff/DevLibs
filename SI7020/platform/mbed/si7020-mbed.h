/**
 *  @brief:   SI7020 relative humidity and temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __SI7020_MBED_H__
#define __SI7020_MBED_H__

#include "mbed.h"
#include "../../si7020.h"

/**
 * @brief SI7020 mbed specific dev object
 */
typedef struct
{
    I2C* i2c;          // I2C bus instance
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
} si7020_mbed;


/**
 * @brief Initialize SI7020 device with MBED dependent parameters.
 *
 * @param dev      si7020 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool si7020_mbed_init(si7020* const dev, si7020_mbed* const mbed_dev);


#endif /*__SI7020_MBED_H__ */
