// rf22-cube-hal.h
//
// STM32 CUBE HAL platform layer for the portable rf22 core.
//
// Derived from the RF22 library:
//   Author: Mike McCauley (mikem@open.com.au)
//   Copyright (C) 2011 Mike McCauley
//   ported to mbed by Karl Zweimueller
//   modified by Lukasz Uszko luszko@op.pl
//
// This software is Copyright (C) 2011 Mike McCauley. Use is subject to license
// conditions. The main licensing options available are GPL V2 or Commercial.
// See LICENSE.txt for the full statement. This file is NOT MIT licensed.
//
// Restructured for the DevLibs portable-core/platform layout by luk6xff, 2020.

#ifndef __RF22_CUBE_HAL_H__
#define __RF22_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"

#include "../../rf22.h"

/**
 * @brief RF22 cube hal specific dev object
 */
typedef struct
{
    SPI_HandleTypeDef* spi; // SPI peripheral handle, mode 0, MSB first, 8 bit
    GPIO_TypeDef* cs_port;  // nSEL port
    uint16_t cs_pin;        // nSEL pin
    GPIO_TypeDef* irq_port; // nIRQ port
    uint16_t irq_pin;       // nIRQ pin
    GPIO_TypeDef* sdn_port; // SDN port, NULL if SDN is tied low in hardware
    uint16_t sdn_pin;       // SDN pin
} rf22_cube_hal;

/**
 * @brief Initialize RF22 device with STM32 CUBE HAL dependent parameters.
 *
 * @param dev          rf22 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool rf22_cube_hal_init(rf22* const dev, rf22_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif // __RF22_CUBE_HAL_H__
