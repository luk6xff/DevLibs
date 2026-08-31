/**
 *  @brief:   AS3935 Franklin lightning sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __AS3935_CUBE_HAL_H__
#define __AS3935_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../as3935.h"

/**
 * @brief AS3935 cube hal specific dev object
 */
typedef struct
{
    I2C_HandleTypeDef* i2c; // I2C peripheral handle
    GPIO_TypeDef* irq_port; // GPIO port of the sensor IRQ line
    uint16_t irq_pin;       // GPIO pin of the sensor IRQ line
} as3935_cube_hal;

/**
 * @brief Initialize AS3935 device with STM32 CUBE HAL dependent parameters.
 *
 * @param dev          as3935 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool as3935_cube_hal_init(as3935* const dev,
                          as3935_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /* __AS3935_CUBE_HAL_H__ */
