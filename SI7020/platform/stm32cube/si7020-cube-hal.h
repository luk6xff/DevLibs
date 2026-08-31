/**
 *  @brief:   SI7020 relative humidity and temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __SI7020_CUBE_HAL_H__
#define __SI7020_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../si7020.h"

/**
 * @brief SI7020 cube hal specific dev object
 */
typedef struct
{
    I2C_HandleTypeDef* i2c; // I2C peripheral handle
} si7020_cube_hal;

/**
 * @brief Initialize SI7020 device with STM32 CUBE HAL dependent parameters.
 *
 * @param dev          si7020 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool si7020_cube_hal_init(si7020* const dev,
                          si7020_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /*__SI7020_CUBE_HAL_H__ */
