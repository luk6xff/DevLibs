/**
 *  @brief:   LTC2460 ultra-tiny 16-bit delta-sigma ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __LTC2460_CUBE_HAL_H__
#define __LTC2460_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../ltc2460.h"

/**
 * @brief LTC2460 cube hal specific dev object
 */
typedef struct
{
    SPI_HandleTypeDef* spi; // SPI peripheral handle
    GPIO_TypeDef* cs_port;  // Chip select GPIO port
    uint16_t cs_pin;        // Chip select GPIO pin
} ltc2460_cube_hal;

/**
 * @brief Initialize LTC2460 device with STM32 CUBE HAL dependent parameters.
 *
 * @param dev          ltc2460 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool ltc2460_cube_hal_init(ltc2460* const dev,
                           ltc2460_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /*__LTC2460_CUBE_HAL_H__ */
