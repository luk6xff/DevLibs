/**
 *  @brief:   SPFD5408 240x320 TFT LCD controller library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __SPFD5408_ARDUINO_H__
#define __SPFD5408_ARDUINO_H__

#include "../../spfd5408.h"
#include <Arduino.h>

/** Width of the parallel bus driven by this library */
#define SPFD5408_ARDUINO_DATA_PINS 16

/**
 * @brief SPFD5408 arduino specific dev object
 */
typedef struct
{
    int data_pins[SPFD5408_ARDUINO_DATA_PINS]; // D0 first, D15 last
    int rs_pin;                                // Register / data select
    int wr_pin;                                // Active low write strobe
    int rd_pin;                                // Active low read strobe
    int cs_pin;                                // Active low chip select
    int reset_pin;                             // Active low hardware reset
} spfd5408_arduino;

/**
 * @brief Initialize SPFD5408 device with ARDUINO dependent parameters.
 *
 * @param dev         spfd5408 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool spfd5408_arduino_init(spfd5408* const dev,
                           spfd5408_arduino* const arduino_dev);


#endif // __SPFD5408_ARDUINO_H__
