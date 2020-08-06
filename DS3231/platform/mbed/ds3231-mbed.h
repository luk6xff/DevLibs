
/**
 *  @brief:  Implementation of a DS3231 platform dependent [MBED] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-12-02
 */


#ifndef __DS3231_MBED_H__
#define __DS3231_MBED_H__

#include "mbed.h"
#include "../../ds3231.h"


/** DS3231 mbed specific dev object **/
typedef struct
{
    I2C* i2c;
} ds3231_mbed;

void ds3231_mbed_init(ds3231* const dev, ds3231_mbed* const mbed_dev);

void ds3231_mbed_deinit();

#endif // __DS3231_MBED_H__