
/**
 *  @brief:  Implementation of a TEMT6000 platform dependent [ARDUINO] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-08-06
 */


#ifndef __TEMT6000_ARDUINO_H__
#define __TEMT6000_ARDUINO_H__

#include "../../temt6000.h"
#include <Arduino.h>

/**
 * @brief TEMT6000 arduino platform specific dev object
 */
typedef struct
{
    uint8_t analog_gpio_pin;
} temt6000_arduino;

void temt6000_arduino_init(temt6000* const dev, temt6000_arduino* const arduino_dev);

#endif // __TEMT6000_ARDUINO_H__