/**
 *  @brief:   MAX4070 bidirectional high-side current-sense amplifier library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MAX4070_MBED_H__
#define __MAX4070_MBED_H__

#include "mbed.h"
#include "../../max4070.h"

/**
 * @brief MAX4070 mbed specific dev object
 */
typedef struct
{
    AnalogIn* adc; // Analog input wired to the amplifier OUT pin
} max4070_mbed;


/**
 * @brief Initialize MAX4070 device with MBED dependent parameters.
 *
 * @note AnalogIn::read_u16() always reports a 16-bit result, so
 *       `dev->adc_max_value` has to be set to 0xFFFF.
 *
 * @param dev      max4070 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool max4070_mbed_init(max4070* const dev, max4070_mbed* const mbed_dev);


#endif /*__MAX4070_MBED_H__ */
