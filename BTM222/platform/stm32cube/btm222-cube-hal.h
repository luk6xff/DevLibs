/**
 *  @brief:   BTM222 Bluetooth SPP module library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __BTM222_CUBE_HAL_H__
#define __BTM222_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../btm222.h"

/**
 * @brief BTM222 cube hal specific dev object
 */
typedef struct
{
    UART_HandleTypeDef* uart;     // UART peripheral handle
    uint32_t            baudrate; // Baudrate the UART has been configured with
} btm222_cube_hal;

/**
 * @brief Initialize BTM222 device with STM32 CUBE HAL dependent parameters.
 *
 * The UART peripheral has to be initialized by the application (normally by the
 * CubeMX generated code) before this function is called.
 *
 * @param dev          btm222 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool btm222_cube_hal_init(btm222* const dev,
                          btm222_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /* __BTM222_CUBE_HAL_H__ */
