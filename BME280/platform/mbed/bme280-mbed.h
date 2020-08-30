
/**
 *  @brief:  Implementation of a BME280 platform dependent [MBED] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-08-18
 */


#ifndef __BME280_MBED_H__
#define __BME280_MBED_H__

#include "mbed.h"
#include "../../bme280.h"


/** BME280 mbed specific dev object **/
typedef struct
{
    I2C* i2c;
    SPI* spi;
    DigitalOut* spi_cs;
} bme280_mbed;

bool bme280_mbed_init(bme280 *const dev, bme280_mbed* const mbed_dev);

bool bme280_mbed_deinit();

#endif // __BME280_MBED_H__
