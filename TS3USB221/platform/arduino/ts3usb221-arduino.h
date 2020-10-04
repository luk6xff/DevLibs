/**
 *  @brief:   TS3USB221 High-speed USB 2.0 (480-Mbps) 1:2 multiplexer – demultiplexer library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-27
 *  @license: MIT
 */

#ifndef __TS3USB221_ARDUINO_H__
#define __TS3USB221_ARDUINO_H__

#include "../../ts3usb221.h"
#include <Arduino.h>

/**
 * @brief TS3USB221 arduino specific dev object
 */
typedef struct
{
    int oe_pin;
    int s_pin;
} ts3usb221_arduino;

/**
 * @brief Initialize TS3USB221 device with ARDUINO dependent parameters.
 */
bool ts3usb221_arduino_init(ts3usb221 *const dev, ts3usb221_arduino *const arduino_dev);


#endif // __TS3USB221_ARDUINO_H__