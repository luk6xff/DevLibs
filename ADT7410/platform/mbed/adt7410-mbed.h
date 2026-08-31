/**
 *  @brief:   ADT7410 digital temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __ADT7410_MBED_H__
#define __ADT7410_MBED_H__

#include "mbed.h"
#include "../../adt7410.h"

/**
 * @brief ADT7410 mbed specific dev object
 */
typedef struct
{
    I2C* i2c;          // I2C bus instance
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
} adt7410_mbed;

/**
 * @brief Initialize ADT7410 device with MBED dependent parameters.
 *
 * @param dev      adt7410 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool adt7410_mbed_init(adt7410* const dev, adt7410_mbed* const mbed_dev);


#endif /* __ADT7410_MBED_H__ */
