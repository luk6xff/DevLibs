/**
 *  @brief:   DS2782 stand-alone battery fuel gauge library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __DS2782_CUBE_HAL_H__
#define __DS2782_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../ds2782.h"

/**
 * @brief DS2782 cube hal specific dev object
 */
typedef struct
{
    I2C_HandleTypeDef* i2c; // I2C peripheral handle
} ds2782_cube_hal;

/**
 * @brief Initialize DS2782 device with STM32 CUBE HAL dependent parameters.
 *
 * @param dev          ds2782 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool ds2782_cube_hal_init(ds2782* const dev,
                          ds2782_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /* __DS2782_CUBE_HAL_H__ */
