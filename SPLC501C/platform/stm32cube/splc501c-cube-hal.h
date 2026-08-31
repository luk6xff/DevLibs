/**
 *  @brief:   SPLC501C 128x64 monochrome graphic LCD library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __SPLC501C_CUBE_HAL_H__
#define __SPLC501C_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../splc501c.h"

/**
 * @brief SPLC501C cube hal specific dev object
 */
typedef struct
{
    SPI_HandleTypeDef* spi;   // SPI peripheral handle
    GPIO_TypeDef* cs_port;    // Active low chip select
    uint16_t cs_pin;
    GPIO_TypeDef* a0_port;    // Data (high) / command (low) select
    uint16_t a0_pin;
    GPIO_TypeDef* reset_port; // Active low hardware reset
    uint16_t reset_pin;
} splc501c_cube_hal;

/**
 * @brief Initialize SPLC501C device with STM32 CUBE HAL dependent parameters.
 *
 * The GPIO ports and the SPI peripheral have to be configured by the
 * application before this function is called.
 *
 * @param dev          splc501c device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool splc501c_cube_hal_init(splc501c* const dev,
                            splc501c_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /* __SPLC501C_CUBE_HAL_H__ */
