/**
 *  @brief:   MAX4070 bidirectional high-side current-sense amplifier library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MAX4070_CUBE_HAL_H__
#define __MAX4070_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../max4070.h"

/**
 * @brief MAX4070 cube hal specific dev object
 */
typedef struct
{
    ADC_HandleTypeDef* adc; // ADC peripheral handle, already configured for the
                            // channel wired to the amplifier OUT pin
    uint32_t timeout_ms;    // Conversion poll timeout in milliseconds
} max4070_cube_hal;

/**
 * @brief Initialize MAX4070 device with STM32 CUBE HAL dependent parameters.
 *
 * @param dev          max4070 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool max4070_cube_hal_init(max4070* const dev,
                           max4070_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /*__MAX4070_CUBE_HAL_H__ */
