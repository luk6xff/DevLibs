/**
 *  @brief:   HM-11 Bluetooth Low Energy 4.0 module library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __HM11_MBED_H__
#define __HM11_MBED_H__

#include "mbed.h"
#include "../../hm11.h"

/**
 * @brief HM11 mbed specific dev object
 *
 * `RawSerial` is used on purpose: it is interrupt safe, it never blocks in
 * getc() when readable() reports data and it is available in both Mbed 2 and
 * Mbed OS 5. Ported to Mbed OS 6 the same descriptor can hold a
 * `BufferedSerial` instead, only the three UART helpers need to change.
 */
typedef struct
{
    RawSerial* serial;   // Serial port connected to the module
    uint32_t   baudrate; // Baudrate applied by hm11_mbed_init(), 0 keeps the current one
} hm11_mbed;

/**
 * @brief Initialize HM11 device with MBED dependent parameters.
 *
 * @param dev      hm11 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool hm11_mbed_init(hm11* const dev, hm11_mbed* const mbed_dev);

/**
 * @brief Attach the module UART receive interrupt to the ring buffer.
 *
 * Optional: without it the application has to call hm11_poll() often enough to
 * keep up with the incoming data.
 *
 * @param dev hm11 device object, must stay alive as long as the IRQ is attached
 * @return True on success, false otherwise
 */
bool hm11_mbed_attach_rx_irq(hm11* const dev);


#endif /* __HM11_MBED_H__ */
