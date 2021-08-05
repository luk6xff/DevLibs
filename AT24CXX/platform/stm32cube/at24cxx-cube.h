/**
 *  @brief:   EEPROM AT24CXX library - stm32cube platform
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2019-12-01
 *  @license: MIT
 */

#ifndef __AT24CXX_CUBE_H__
#define __AT24CXX_CUBE_H__

#include "stm32f0xx_hal.h"
#include "../../at24cxx.h"


typedef struct
{
    I2C_HandleTypeDef* i2c;       // I2C Interface
    GPIO_TypeDef* gpio_wp_port;   // AT24Cxx Write protection port
	uint16_t gpio_wp_pin;		  // AT24Cxx Write protection pin
} at24cxx_cube;

/**
 * @brief Initialize eeprom with STM32 CUBE dependent parameters.
 */
void at24cxx_cube_init(at24cxx* const dev, at24cxx_cube* const cube_dev, uint8_t i2c_addr_pins);

/**
 * @brief Deinitalize eeprom.
 */
void at24cxx_cube_deinit(at24cxx* const dev);


#endif /*__AT24CXX_CUBE_H__ */
