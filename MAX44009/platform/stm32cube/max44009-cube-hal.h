/**
 *  @brief:   MAX44009 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MAX44009_CUBE_HAL_H__
#define __MAX44009_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../max44009.h"

/**
 * @brief MAX44009 cube hal specific dev object
 */
typedef struct
{
    I2C_HandleTypeDef* i2c; // I2C peripheral handle
} max44009_cube_hal;

/**
 * @brief Initialize MAX44009 device with STM32 CUBE HAL dependent parameters.
 *
 * @param dev          max44009 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool max44009_cube_hal_init(max44009* const dev,
                            max44009_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /*__MAX44009_CUBE_HAL_H__ */

