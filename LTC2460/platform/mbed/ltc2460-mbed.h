/**
 *  @brief:   LTC2460 ultra-tiny 16-bit delta-sigma ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __LTC2460_MBED_H__
#define __LTC2460_MBED_H__

#include "mbed.h"
#include "../../ltc2460.h"

/**
 * @brief LTC2460 mbed specific dev object
 */
typedef struct
{
    SPI* spi;       // SPI bus instance, its frequency is set by the caller
    DigitalOut* cs; // Chip select pin, driven by the library
} ltc2460_mbed;


/**
 * @brief Initialize LTC2460 device with MBED dependent parameters.
 *
 * @param dev      ltc2460 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool ltc2460_mbed_init(ltc2460* const dev, ltc2460_mbed* const mbed_dev);


#endif /*__LTC2460_MBED_H__ */
