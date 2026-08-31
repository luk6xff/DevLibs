/**
 *  @brief:   US-015 (HC-SR04 compatible) ultrasonic distance sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __US015_CUBE_HAL_H__
#define __US015_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../us015.h"

/**
 * @brief US-015 cube hal specific dev object
 */
typedef struct
{
    GPIO_TypeDef* trigger_port; // Trigger GPIO port, configured as output
    uint16_t trigger_pin;       // Trigger GPIO pin
    GPIO_TypeDef* echo_port;    // Echo GPIO port, configured as input
    uint16_t echo_pin;          // Echo GPIO pin
} us015_cube_hal;

/**
 * @brief Initialize US-015 device with STM32 CUBE HAL dependent parameters.
 *
 * @param dev          us015 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool us015_cube_hal_init(us015* const dev,
                         us015_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /*__US015_CUBE_HAL_H__ */
