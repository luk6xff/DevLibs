/**
 *  @brief:   Implementation of a BME280 platform dependent [ARDUINO] functions
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2022-10-04
 *  @license: MIT
 */

#ifndef __BME280_ARDUINO_H__
#define __BME280_ARDUINO_H__

#include "../../bme280.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>


/**
 * @brief BME280 arduino specific spi object
 */
typedef struct
{
    SPISettings settings;
    SPIClass *hndl;
    int mosi;
    int miso;
    int sck;
    int nss;
} bme280_arduino_spi;

/**
 * @brief BME280 arduino specific i2c object
 */
typedef struct
{
    TwoWire *hndl;
    int sda_pin;
    int scl_pin;
} bme280_arduino_i2c;

/**
 * @brief BME280 arduino specific dev object
 */
typedef struct
{
    bme280_arduino_i2c i2c;
    bme280_arduino_spi spi;
} bme280_arduino;

/**
 * @brief Initialize BME280 device with ARDUINO dependent parameters.
 */
bool bme280_arduino_init(bme280 *const dev, bme280_arduino *const arduino_dev);

bool bme280_arduino_deinit(bme280 *const dev);


#endif // __BME280_ARDUINO_H__