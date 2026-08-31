/**
 *  @brief:   ADT7410 digital temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __ADT7410_CUBE_HAL_H__
#define __ADT7410_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../adt7410.h"

/**
 * @brief ADT7410 cube hal specific dev object
 */
typedef struct
{
    I2C_HandleTypeDef* i2c; // I2C peripheral handle
} adt7410_cube_hal;

/**
 * @brief Initialize ADT7410 device with STM32 CUBE HAL dependent parameters.
 *
 * @param dev          adt7410 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool adt7410_cube_hal_init(adt7410* const dev,
                           adt7410_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /* __ADT7410_CUBE_HAL_H__ */
