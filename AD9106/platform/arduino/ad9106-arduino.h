/**
 *  @brief:   AD9106 quad, low power, 12-bit, 180 MSPS DAC and waveform generator library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __AD9106_ARDUINO_H__
#define __AD9106_ARDUINO_H__

#include "../../ad9106.h"
#include <Arduino.h>
#include <SPI.h>

/**
 * @brief AD9106 arduino specific dev object
 */
typedef struct
{
    SPIClass* spi;     // SPI bus instance, e.g. &SPI
    uint32_t spi_freq; // SPI clock in Hz, 0 selects the driver default
    int cs_pin;        // SPI chip select pin
    int reset_pin;     // Hardware reset pin, negative if not connected
    int trigger_pin;   // Pattern trigger pin, negative if not connected
} ad9106_arduino;

/**
 * @brief Initialize AD9106 device with ARDUINO dependent parameters.
 *
 * @param dev         ad9106 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool ad9106_arduino_init(ad9106* const dev, ad9106_arduino* const arduino_dev);


#endif // __AD9106_ARDUINO_H__
