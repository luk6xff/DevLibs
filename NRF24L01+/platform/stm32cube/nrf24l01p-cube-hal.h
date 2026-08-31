
/**
 *  @brief:  Implementation of a NRF24L01 platform dependent [STM32CUBE] radio functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-12-02
 */


#ifndef __NRF24L01P_CUBE_H__
#define __NRF24L01P_CUBE_H__

#include "stm32f0xx_hal.h"
#include "../../nrf24l01p.h"

typedef struct
{
	SPI_HandleTypeDef *spi;
	GPIO_TypeDef *ce_port;
	uint16_t ce_pin;
	GPIO_TypeDef *csn_port;
	uint16_t csn_pin;
} nrf24l01p_cube;


void nrf24l01p_cube_init(nrf24l01p * const dev, nrf24l01p_cube* const cube_dev);

void nrf24l01p_cube_deinit(nrf24l01p * const dev);

#endif // __NRF24L01P_CUBE_H__
