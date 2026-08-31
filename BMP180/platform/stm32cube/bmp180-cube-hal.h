
/**
 *  @brief:  Implementation of a BMP180 platform dependent [STM32 CUBE HAL] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-08-12
 */


#ifndef __BMP180_CUBE_HAL_H__
#define __BMP180_CUBE_HAL_H__

#include "stm32l0xx_hal.h"
#include "../../bmp180.h"


/** BMP180 cube_hal specific dev object **/
typedef struct
{
    I2C_HandleTypeDef* i2c;
} bmp180_cube_hal;

bool bmp180_cube_hal_init(bmp180 *const dev, bmp180_cube_hal* const cube_hal_dev);

bool bmp180_cube_hal_deinit();

#endif // __BMP180_MBED_H__
