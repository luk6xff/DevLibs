/**
 *  @brief:   GTS-4E-60 (Fibocom) GPS module library, NMEA 0183 parser
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __GTS4E60_ARDUINO_H__
#define __GTS4E60_ARDUINO_H__

#include "../../gts4e60.h"
#include <Arduino.h>

/** Value of `shutdown_pin` meaning that no shutdown line is wired */
#define GTS4E60_ARDUINO_NO_PIN (-1)

/**
 * @brief GTS4E60 arduino specific dev object
 *
 * `Stream` is used so that both `HardwareSerial` and `SoftwareSerial` fit.
 * `Stream` has no begin() method, therefore the sketch has to open the port
 * itself before calling gts4e60_arduino_init(); `baudrate` is only kept for
 * reference and is copied into `dev->baudrate`.
 */
typedef struct
{
    Stream*  serial;       // Serial port connected to the module
    uint32_t baudrate;     // Baudrate the port has been opened with
    int      shutdown_pin; // Shutdown line, GTS4E60_ARDUINO_NO_PIN when unused
    bool     shutdown_active_high; // Polarity of the shutdown line
} gts4e60_arduino;

/**
 * @brief Initialize GTS4E60 device with ARDUINO dependent parameters.
 *
 * @param dev         gts4e60 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool gts4e60_arduino_init(gts4e60* const dev,
                          gts4e60_arduino* const arduino_dev);


#endif // __GTS4E60_ARDUINO_H__
