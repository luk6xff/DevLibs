
/**
 *  @brief:  Implementation of a NRF24L01+ platform dependent [MBED] radio functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-12-02
 */


#ifndef __NRF24L01P_MBED_H__
#define __NRF24L01P_MBED_H__

#include "mbed.h"
#include "../../nrf24l01p.h"

typedef struct
{
    Spi *spi;       // SPI Interface
    DigitalOut *ce;
    DigitalOut *csn;
} nrf24l01p_mbed;


void nrf24l01p_mbed_init(nrf24l01p * const dev, nrf24l01p_mbed * const mbed_dev);

void nrf24l01p_mbed_deinit(nrf24l01p * const dev);

#endif // __NRF24L01P_MBED_H__
