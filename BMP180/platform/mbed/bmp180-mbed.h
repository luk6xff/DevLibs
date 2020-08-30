
/**
 *  @brief:  Implementation of a BMP180 platform dependent [MBED] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-08-12
 */


#ifndef __BMP180_MBED_H__
#define __BMP180_MBED_H__

#include "mbed.h"
#include "../../bmp180.h"


/** BMP180 mbed specific dev object **/
typedef struct
{
    I2C* i2c;
} bmp180_mbed;

void bmp180_mbed_init(bmp180 *const dev, bmp180_mbed* const mbed_dev);

void bmp180_mbed_deinit();

#endif // __BMP180_MBED_H__