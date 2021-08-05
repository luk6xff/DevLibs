/**
 *  @brief:   EEPROM AT24CXX library - mbed platform
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2019-11-25
 *  @license: MIT
 */

#ifndef __AT24CXX_MBED_H__
#define __AT24CXX_MBED_H__

#include "mbed.h"
#include "../../at24cxx.h"


typedef struct
{
    I2C* i2c;       // I2C Interface
    DigitalOut* wp; // AT24Cxx Write protection pin
} at24cxx_mbed;


/**
 * @brief Initialize eeprom with MBED dependent parameters.
 */
void at24cxx_mbed_init(at24cxx* const dev, at24cxx_mbed* const mbed_dev, uint8_t i2c_addr_pins);

/**
 * @brief Deinitalize eeprom.
 */
void at24cxx_mbed_deinit(void);


#endif /*__AT24CXX_MBED_H__ */

