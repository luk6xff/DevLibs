
/**
 *  @brief:  Implementation of a RFM69 platform dependent [STM32 CUBE LL] radio functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-01-11
 */


#ifndef __RFM69_CUBE_LL_H__
#define __RFM69_CUBE_LL_H__

#include "stm32l0xx.h"
#include "stm32l0xx_ll_crs.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_dma.h"
#include "stm32l0xx_ll_spi.h"
#include "stm32l0xx_ll_gpio.h"
#include "../../../rfm69.h"


/**
 * @brief RFM69 STM32Cube  specific dev object
 */
typedef struct
{
    SPI_TypeDef* spi;
    GPIO_TypeDef* nss_port;
    uint16_t      nss_pin;
    GPIO_TypeDef* reset_port;
    uint16_t      reset_pin;
} rfm69_cube;


/**
 * @brief RFM69 STM32Cube Init function driving.
 *
 * @param  dev       RFM69 Device object pointer
 * @param  cube_dev  STM32Cube  device object pointer
 */
void rfm69_cube_init(rfm69* const dev, rfm69_cube* const cube_dev);


/**
 * @brief RFM69 STM32Cube deinit function driving.
 *
 * @param  dev       RFM69 Device object pointer
 */
void rfm69_cube_deinit(rfm69* const dev);

#endif // __RFM69_CUBE_LL_H__
