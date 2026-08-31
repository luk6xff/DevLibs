/**
 *  @brief:   AD9106 quad, low power, 12-bit, 180 MSPS DAC and waveform generator library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __AD9106_MBED_H__
#define __AD9106_MBED_H__

#include "mbed.h"
#include "../../ad9106.h"

/**
 * @brief AD9106 mbed specific dev object
 */
typedef struct
{
    SPI* spi;             // SPI bus instance
    DigitalOut* cs;       // SPI chip select pin
    DigitalOut* reset;    // Hardware reset pin, NULL if not connected
    DigitalOut* trigger;  // Pattern trigger pin, NULL if not connected
} ad9106_mbed;

/**
 * @brief Initialize AD9106 device with MBED dependent parameters.
 *
 * @param dev      ad9106 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool ad9106_mbed_init(ad9106* const dev, ad9106_mbed* const mbed_dev);


#endif /* __AD9106_MBED_H__ */
