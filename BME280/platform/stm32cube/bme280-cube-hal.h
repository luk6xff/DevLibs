
/**
 *  @brief:  Implementation of a BME280 platform dependent [STM32 CUBE HAL] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-08-18
 */


#ifndef __BME280_CUBE_HAL_H__
#define __BME280_CUBE_HAL_H__

#include "stm32l0xx_hal.h"
#include "../../bme280.h"


/** BME280 cube_hal specific dev object **/
typedef struct
{
    I2C_HandleTypeDef* i2c;
    SPI_HandleTypeDef* spi;
    GPIO_TypeDef* spi_cs_port;
    uint16_t      spi_cs_pin;
} bme280_cube_hal;

bool bme280_cube_hal_init(bme280 *const dev, bme280_cube_hal* const cube_hal_dev);

bool bme280_cube_hal_deinit();

#endif // __BME280_MBED_H__
