/**
 *  @brief:   SPLC501C 128x64 monochrome graphic LCD library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __SPLC501C_ARDUINO_H__
#define __SPLC501C_ARDUINO_H__

#include "../../splc501c.h"
#include <Arduino.h>
#include <SPI.h>

/**
 * @brief SPLC501C arduino specific dev object
 */
typedef struct
{
    SPIClass* spi;     // SPI bus instance, e.g. &SPI
    int cs_pin;        // Active low chip select
    int a0_pin;        // Data (high) / command (low) select
    int reset_pin;     // Active low hardware reset
    uint32_t spi_freq; // SPI clock in Hz, 0 selects 4 MHz
} splc501c_arduino;

/**
 * @brief Initialize SPLC501C device with ARDUINO dependent parameters.
 *
 * @param dev         splc501c device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool splc501c_arduino_init(splc501c* const dev,
                           splc501c_arduino* const arduino_dev);


#endif // __SPLC501C_ARDUINO_H__
