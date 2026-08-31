/**
 *  @brief:   SPFD5408 240x320 TFT LCD controller library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __SPFD5408_CUBE_HAL_H__
#define __SPFD5408_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../spfd5408.h"

/**
 * @brief SPFD5408 cube hal specific dev object
 *
 * The whole 16-bit data bus has to live on a single GPIO port, D0 is mapped
 * onto pin 0 of that port and D15 onto pin 15.
 */
typedef struct
{
    GPIO_TypeDef* data_port;  // Port carrying D0..D15
    GPIO_TypeDef* rs_port;    // Register / data select
    uint16_t rs_pin;
    GPIO_TypeDef* wr_port;    // Active low write strobe
    uint16_t wr_pin;
    GPIO_TypeDef* rd_port;    // Active low read strobe
    uint16_t rd_pin;
    GPIO_TypeDef* cs_port;    // Active low chip select
    uint16_t cs_pin;
    GPIO_TypeDef* reset_port; // Active low hardware reset
    uint16_t reset_pin;
} spfd5408_cube_hal;

/**
 * @brief Initialize SPFD5408 device with STM32 CUBE HAL dependent parameters.
 *
 * All the GPIO ports have to be configured as push-pull outputs by the
 * application before this function is called.
 *
 * @param dev          spfd5408 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool spfd5408_cube_hal_init(spfd5408* const dev,
                            spfd5408_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /* __SPFD5408_CUBE_HAL_H__ */
