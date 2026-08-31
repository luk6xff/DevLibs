/**
 *  @brief:   MAX9611 high-side current-sense amplifier with 12-bit ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MAX9611_MBED_H__
#define __MAX9611_MBED_H__

#include "mbed.h"
#include "../../max9611.h"

/**
 * @brief MAX9611 mbed specific dev object
 */
typedef struct
{
    I2C* i2c;          // I2C bus instance
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
} max9611_mbed;


/**
 * @brief Initialize MAX9611 device with MBED dependent parameters.
 *
 * @param dev      max9611 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool max9611_mbed_init(max9611* const dev, max9611_mbed* const mbed_dev);


#endif /*__MAX9611_MBED_H__ */
