/**
 *  @brief:   LTC2460 ultra-tiny 16-bit delta-sigma ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __LTC2460_ARDUINO_H__
#define __LTC2460_ARDUINO_H__

#include "../../ltc2460.h"
#include <Arduino.h>
#include <SPI.h>

/**
 * @brief LTC2460 arduino specific dev object
 */
typedef struct
{
    SPIClass* spi;     // SPI bus instance, e.g. &SPI
    int cs_pin;        // Chip select pin, driven by the library
    uint32_t spi_freq; // SPI clock in Hz, LTC2460 handles up to 2 MHz
} ltc2460_arduino;

/**
 * @brief Initialize LTC2460 device with ARDUINO dependent parameters.
 *
 * @param dev         ltc2460 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool ltc2460_arduino_init(ltc2460* const dev,
                          ltc2460_arduino* const arduino_dev);


#endif // __LTC2460_ARDUINO_H__
