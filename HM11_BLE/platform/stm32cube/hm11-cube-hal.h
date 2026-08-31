/**
 *  @brief:   HM-11 Bluetooth Low Energy 4.0 module library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __HM11_CUBE_HAL_H__
#define __HM11_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../hm11.h"

/**
 * @brief HM11 cube hal specific dev object
 */
typedef struct
{
    UART_HandleTypeDef* uart;     // UART peripheral handle
    uint32_t            baudrate; // Baudrate the UART has been configured with
} hm11_cube_hal;

/**
 * @brief Initialize HM11 device with STM32 CUBE HAL dependent parameters.
 *
 * The UART peripheral has to be initialized by the application (normally by the
 * CubeMX generated code) before this function is called.
 *
 * @param dev          hm11 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool hm11_cube_hal_init(hm11* const dev, hm11_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /* __HM11_CUBE_HAL_H__ */
