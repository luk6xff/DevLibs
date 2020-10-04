/**
 *  @brief:   TS3USB221 High-speed USB 2.0 (480-Mbps) 1:2 multiplexer – demultiplexer library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-27
 *  @license: MIT
 */

#ifndef __TS3USB221_H__
#define __TS3USB221_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>


/**
 * @brief ts3usb221 dev object
 */
typedef struct
{
    void* platform_dev;
} ts3usb221;


/**
 * @brief ts3usb221 pins
 */
typedef enum
{
    TS3USB221_PIN_OE,
    TS3USB221_PIN_S,
} ts3usb221_pin;


/**
 * @brief ts3usb221 pin state
 */
typedef enum
{
    TS3USB221_PIN_STATE_HIGH,
    TS3USB221_PIN_STATE_LOW,
} ts3usb221_pin_state;


/**
 * @brief Initialize device
 *
 * @param dev ts3usb221 device object
 * @return True on success, false otherwise
 */
bool ts3usb221_init(ts3usb221 *const dev);

/**
 * @brief Puts ts3usb221 device into low power mode.
 *
 * @param dev ts3usb221 device object
 * @return True on success, false otherwise
 */
void ts3usb221_low_power_mode(const ts3usb221 *const dev);

/**
 * @brief Forwards data from 1D input to D output of ts3usb221 device.
 *
 * @param dev ts3usb221 device object
 * @return True on success, false otherwise
 */
void ts3usb221_enable_1D_channel(const ts3usb221 *const dev);

/**
 * @brief Forwards data from 2D input to D output of ts3usb221 device.
 *
 * @param dev ts3usb221 device object
 * @return True on success, false otherwise
 */
void ts3usb221_enable_2D_channel(const ts3usb221 *const dev);


/**-----------------------------------------------------------------------------
 * @brief HW dependent functions - must be defined for each platform
 *        (look at the `platform` directory)
 -----------------------------------------------------------------------------*/

/**
 * @brief Sets a proper state of ts3usb221 input
 *
 * @param  dev      ts3usb221 device object
 * @param  pin      ts3usb221 pin to be set
 * @param  pin_state ts3usb221 pin state to be set
 * @return None
 */
extern void ts3usb221_set_pin(const ts3usb221 *const dev, ts3usb221_pin pin,
                                ts3usb221_pin_state pin_state);


#ifdef __cplusplus
}
#endif

#endif /* __TS3USB221_H__ */

