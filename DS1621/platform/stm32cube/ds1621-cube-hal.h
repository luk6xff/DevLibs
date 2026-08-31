/**
 *  @brief:   DS1621 digital thermometer and thermostat library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __DS1621_CUBE_HAL_H__
#define __DS1621_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../ds1621.h"

/**
 * @brief DS1621 cube hal specific dev object
 */
typedef struct
{
    I2C_HandleTypeDef* i2c; // I2C peripheral handle
} ds1621_cube_hal;

/**
 * @brief Initialize DS1621 device with STM32 CUBE HAL dependent parameters.
 *
 * @param dev          ds1621 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool ds1621_cube_hal_init(ds1621* const dev,
                          ds1621_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /*__DS1621_CUBE_HAL_H__ */
