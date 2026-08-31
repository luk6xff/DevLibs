/**
 *  @brief:   AD9106 quad, low power, 12-bit, 180 MSPS DAC and waveform generator library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __AD9106_CUBE_HAL_H__
#define __AD9106_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../ad9106.h"

/**
 * @brief AD9106 cube hal specific dev object
 */
typedef struct
{
    SPI_HandleTypeDef* spi;   // SPI peripheral handle
    GPIO_TypeDef* cs_port;    // SPI chip select port
    uint16_t cs_pin;          // SPI chip select pin
    GPIO_TypeDef* reset_port; // Hardware reset port, NULL if not connected
    uint16_t reset_pin;       // Hardware reset pin
    GPIO_TypeDef* trigger_port; // Pattern trigger port, NULL if not connected
    uint16_t trigger_pin;     // Pattern trigger pin
} ad9106_cube_hal;

/**
 * @brief Initialize AD9106 device with STM32 CUBE HAL dependent parameters.
 *
 * @param dev          ad9106 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool ad9106_cube_hal_init(ad9106* const dev,
                          ad9106_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /* __AD9106_CUBE_HAL_H__ */
