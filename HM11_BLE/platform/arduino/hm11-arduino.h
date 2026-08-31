/**
 *  @brief:   HM-11 Bluetooth Low Energy 4.0 module library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __HM11_ARDUINO_H__
#define __HM11_ARDUINO_H__

#include "../../hm11.h"
#include <Arduino.h>

/**
 * @brief HM11 arduino specific dev object
 *
 * `Stream` is used so that both `HardwareSerial` and `SoftwareSerial` fit.
 * `Stream` has no begin() method, therefore the sketch has to open the port
 * itself before calling hm11_arduino_init(); `baudrate` is only kept for
 * reference and is copied into `dev->baudrate`.
 */
typedef struct
{
    Stream*  serial;   // Serial port connected to the module
    uint32_t baudrate; // Baudrate the port has been opened with
} hm11_arduino;

/**
 * @brief Initialize HM11 device with ARDUINO dependent parameters.
 *
 * @param dev         hm11 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool hm11_arduino_init(hm11* const dev, hm11_arduino* const arduino_dev);


#endif // __HM11_ARDUINO_H__
