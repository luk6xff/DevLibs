/**
 *  @brief:   BTM222 Bluetooth SPP module library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __BTM222_MBED_H__
#define __BTM222_MBED_H__

#include "mbed.h"
#include "../../btm222.h"

/**
 * @brief BTM222 mbed specific dev object
 *
 * `RawSerial` is used on purpose: it is interrupt safe, it never blocks in
 * getc() when readable() reports data and it is available in both Mbed 2 and
 * Mbed OS 5. Ported to Mbed OS 6 the same descriptor can hold a
 * `BufferedSerial` instead, only the three UART helpers need to change.
 */
typedef struct
{
    RawSerial* serial;   // Serial port connected to the module
    uint32_t   baudrate; // Baudrate applied by btm222_mbed_init(), 0 keeps the current one
} btm222_mbed;

/**
 * @brief Initialize BTM222 device with MBED dependent parameters.
 *
 * @param dev      btm222 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool btm222_mbed_init(btm222* const dev, btm222_mbed* const mbed_dev);

/**
 * @brief Attach the module UART receive interrupt to the ring buffer.
 *
 * Optional: without it the application has to call btm222_poll() often enough
 * to keep up with the incoming data.
 *
 * @param dev btm222 device object, must stay alive as long as the IRQ is attached
 * @return True on success, false otherwise
 */
bool btm222_mbed_attach_rx_irq(btm222* const dev);


#endif /* __BTM222_MBED_H__ */
