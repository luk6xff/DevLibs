/**
 *  @brief:   US-015 (HC-SR04 compatible) ultrasonic distance sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __US015_MBED_H__
#define __US015_MBED_H__

#include "mbed.h"
#include "../../us015.h"

/**
 * @brief US-015 mbed specific dev object
 */
typedef struct
{
    DigitalOut* trigger; // Output driving the sensor TRIG input
    DigitalIn* echo;     // Input reading the sensor ECHO output
} us015_mbed;


/**
 * @brief Initialize US-015 device with MBED dependent parameters.
 *
 * @param dev      us015 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool us015_mbed_init(us015* const dev, us015_mbed* const mbed_dev);


#endif /*__US015_MBED_H__ */
