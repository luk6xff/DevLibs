/**
 *  @brief:   SPLC501C 128x64 monochrome graphic LCD library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __SPLC501C_MBED_H__
#define __SPLC501C_MBED_H__

#include "mbed.h"
#include "../../splc501c.h"

/**
 * @brief SPLC501C mbed specific dev object
 */
typedef struct
{
    SPI* spi;          // SPI bus instance
    DigitalOut* cs;    // Active low chip select
    DigitalOut* a0;    // Data (high) / command (low) select
    DigitalOut* reset; // Active low hardware reset
    uint32_t spi_freq; // SPI clock in Hz, 0 keeps the current bus setting
} splc501c_mbed;

/**
 * @brief Initialize SPLC501C device with MBED dependent parameters.
 *
 * @param dev      splc501c device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool splc501c_mbed_init(splc501c* const dev, splc501c_mbed* const mbed_dev);


#endif /* __SPLC501C_MBED_H__ */
