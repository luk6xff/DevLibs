/**
 *  @brief:   BTM222 Bluetooth SPP module library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __BTM222_ARDUINO_H__
#define __BTM222_ARDUINO_H__

#include "../../btm222.h"
#include <Arduino.h>

/**
 * @brief BTM222 arduino specific dev object
 *
 * `Stream` is used so that both `HardwareSerial` and `SoftwareSerial` fit.
 * `Stream` has no begin() method, therefore the sketch has to open the port
 * itself before calling btm222_arduino_init(); `baudrate` is only kept for
 * reference and is copied into `dev->baudrate`.
 */
typedef struct
{
    Stream*  serial;   // Serial port connected to the module
    uint32_t baudrate; // Baudrate the port has been opened with
} btm222_arduino;

/**
 * @brief Initialize BTM222 device with ARDUINO dependent parameters.
 *
 * @param dev         btm222 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool btm222_arduino_init(btm222* const dev, btm222_arduino* const arduino_dev);


#endif // __BTM222_ARDUINO_H__
