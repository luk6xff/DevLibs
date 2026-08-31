/**
 *  @brief:   SPFD5408 240x320 TFT LCD controller library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __SPFD5408_MBED_H__
#define __SPFD5408_MBED_H__

#include "mbed.h"
#include "../../spfd5408.h"

/**
 * @brief SPFD5408 mbed specific dev object
 */
typedef struct
{
    BusOut* data;      // 16-bit parallel data bus, D0 is the least significant bit
    DigitalOut* rs;    // Register / data select
    DigitalOut* wr;    // Active low write strobe
    DigitalOut* rd;    // Active low read strobe
    DigitalOut* cs;    // Active low chip select
    DigitalOut* reset; // Active low hardware reset
} spfd5408_mbed;

/**
 * @brief Initialize SPFD5408 device with MBED dependent parameters.
 *
 * @param dev      spfd5408 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool spfd5408_mbed_init(spfd5408* const dev, spfd5408_mbed* const mbed_dev);


#endif /* __SPFD5408_MBED_H__ */
