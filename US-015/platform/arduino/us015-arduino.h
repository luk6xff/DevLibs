/**
 *  @brief:   US-015 (HC-SR04 compatible) ultrasonic distance sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __US015_ARDUINO_H__
#define __US015_ARDUINO_H__

#include "../../us015.h"
#include <Arduino.h>

/**
 * @brief US-015 arduino specific dev object
 */
typedef struct
{
    int trigger_pin; // Digital output driving the sensor TRIG input
    int echo_pin;    // Digital input reading the sensor ECHO output
} us015_arduino;

/**
 * @brief Initialize US-015 device with ARDUINO dependent parameters.
 *
 * @param dev         us015 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool us015_arduino_init(us015* const dev, us015_arduino* const arduino_dev);


#endif // __US015_ARDUINO_H__
