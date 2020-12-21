
/**
 *  @brief:  Implementation of a LORA platform dependent [STM32 CUBE HAL] radio functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-11-22
 */


#ifndef __LORA_CUBE_HAL_H__
#define __LORA_CUBE_HAL_H__

#include "stm32l0xx_hal.h"
#include "../../../lora.h"


/** LORA STM32Cube  specific dev object **/
typedef struct
{
    SPI_HandleTypeDef* spi;
    GPIO_TypeDef* nss_port;
    uint16_t      nss_pin;
    GPIO_TypeDef* reset_port;
    uint16_t      reset_pin;
} lora_cube_hal;


/**
 * @brief LORA STM32Cube Init function driving.
 *
 * @param  dev       LORA Device object pointer
 * @param  cube_dev  STM32Cube  device object pointer
 */
bool lora_cube_hal_init(lora *const dev, lora_cube_hal *const cube_dev);


/**
 * @brief LORA STM32Cube deinit function driving.
 *
 * @param  dev       LORA Device object pointer
 */
void lora_cube_hal_deinit(lora *const dev);

#endif // __LORA_CUBE_HAL_H__
