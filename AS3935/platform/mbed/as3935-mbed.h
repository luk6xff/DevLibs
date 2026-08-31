/**
 *  @brief:   AS3935 Franklin lightning sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __AS3935_MBED_H__
#define __AS3935_MBED_H__

#include "mbed.h"
#include "../../as3935.h"

/**
 * @brief AS3935 mbed specific dev object
 */
typedef struct
{
    I2C* i2c;          // I2C bus instance
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
    DigitalIn* irq;    // Pin the sensor IRQ line is connected to
} as3935_mbed;

/**
 * @brief Initialize AS3935 device with MBED dependent parameters.
 *
 * @param dev      as3935 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool as3935_mbed_init(as3935* const dev, as3935_mbed* const mbed_dev);


#endif /* __AS3935_MBED_H__ */
