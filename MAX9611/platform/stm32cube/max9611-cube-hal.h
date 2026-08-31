/**
 *  @brief:   MAX9611 high-side current-sense amplifier with 12-bit ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MAX9611_CUBE_HAL_H__
#define __MAX9611_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../max9611.h"

/**
 * @brief MAX9611 cube hal specific dev object
 */
typedef struct
{
    I2C_HandleTypeDef* i2c; // I2C peripheral handle
} max9611_cube_hal;

/**
 * @brief Initialize MAX9611 device with STM32 CUBE HAL dependent parameters.
 *
 * @param dev          max9611 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool max9611_cube_hal_init(max9611* const dev,
                           max9611_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /*__MAX9611_CUBE_HAL_H__ */
