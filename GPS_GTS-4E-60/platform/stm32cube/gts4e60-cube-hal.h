/**
 *  @brief:   GTS-4E-60 (Fibocom) GPS module library, NMEA 0183 parser
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __GTS4E60_CUBE_HAL_H__
#define __GTS4E60_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../gts4e60.h"

/**
 * @brief GTS4E60 cube hal specific dev object
 */
typedef struct
{
    UART_HandleTypeDef* uart;          // UART peripheral handle
    uint32_t            baudrate;      // Baudrate the UART has been configured with
    GPIO_TypeDef*       shutdown_port; // Shutdown line GPIO port, NULL when unused
    uint16_t            shutdown_pin;  // Shutdown line GPIO pin mask
    bool                shutdown_active_high; // Polarity of the shutdown line
} gts4e60_cube_hal;

/**
 * @brief Initialize GTS4E60 device with STM32 CUBE HAL dependent parameters.
 *
 * The UART and the GPIO peripherals have to be initialized by the application
 * (normally by the CubeMX generated code) before this function is called.
 *
 * @param dev          gts4e60 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool gts4e60_cube_hal_init(gts4e60* const dev,
                           gts4e60_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /* __GTS4E60_CUBE_HAL_H__ */
