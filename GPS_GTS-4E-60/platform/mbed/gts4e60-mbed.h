/**
 *  @brief:   GTS-4E-60 (Fibocom) GPS module library, NMEA 0183 parser
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __GTS4E60_MBED_H__
#define __GTS4E60_MBED_H__

#include "mbed.h"
#include "../../gts4e60.h"

/**
 * @brief GTS4E60 mbed specific dev object
 *
 * `RawSerial` is used on purpose: it is interrupt safe, it never blocks in
 * getc() when readable() reports data and it is available in both Mbed 2 and
 * Mbed OS 5. Ported to Mbed OS 6 the same descriptor can hold a
 * `BufferedSerial` instead, only the three UART helpers below need to change.
 */
typedef struct
{
    RawSerial*  serial;       // Serial port connected to the module
    uint32_t    baudrate;     // Baudrate applied by gts4e60_mbed_init(), 0 keeps the current one
    DigitalOut* shutdown_pin; // Shutdown line, NULL when unused
    bool        shutdown_active_high; // Polarity of the shutdown line
} gts4e60_mbed;

/**
 * @brief Initialize GTS4E60 device with MBED dependent parameters.
 *
 * @param dev      gts4e60 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool gts4e60_mbed_init(gts4e60* const dev, gts4e60_mbed* const mbed_dev);


#endif /* __GTS4E60_MBED_H__ */
