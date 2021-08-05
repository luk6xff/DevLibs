
/**
 *  @brief:  Implementation of a NRF24L01 platform dependent [MBED] radio functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-11-15
 */

#include "nrf24l01p-mbed.h"


//------------------------------------------------------------------------------
void nrf24l01p_mbed_init(nrf24l01p * const dev, nrf24l01p_mbed * const mbed_dev)
{
    dev->platform_dev = mbed_dev;

    // Init SPI
    *(mbed_dev->csn) = 1;
    mbed_dev->spi->format(8,0);
    mbed_dev->spi->frequency(4000000);
    nrf24l01p_delay_ms(10);

    nrf24l01p_init(dev);
}

//------------------------------------------------------------------------------
void nrf24l01p_mbed_deinit(nrf24l01p * const dev)
{
    nrf24l01p_mbed * const pd = (nrf24l01p_mbed*)dev->platform_dev;
    delete(pd->spi);
    delete(pd->csn);
    delete(pd->ce);
}

//------------------------------------------------------------------------------
void nrf24l01p_set_ce_pin(nrf24l01p * const dev, bool enable)
{
    nrf24l01p_mbed * const pd = (nrf24l01p_mbed*)dev->platform_dev;
    if (enable)
    {
        *(pd->ce) = 1;
        return;
    }
    *(pd->ce) = 0;
}


//------------------------------------------------------------------------------
void nrf24l01p_set_csn_pin(nrf24l01p * const dev, bool enable)
{
    nrf24l01p_mbed * const pd = (nrf24l01p_mbed*)dev->platform_dev;
    if (enable)
    {
        *(pd->csn) = 1;
        return;
    }
    *(pd->csn) = 0;
}

//------------------------------------------------------------------------------
uint8_t nrf24l01p_spi_write(nrf24l01p * const dev, uint8_t data)
{
    nrf24l01p_mbed * const pd = (nrf24l01p_mbed*)dev->platform_dev;
    return pd->spi->write(data);
}

//------------------------------------------------------------------------------
void nrf24l01p_delay_ms(int ms)
{
    wait_us(ms*1000);
}

//------------------------------------------------------------------------------
void nrf24l01p_print_all_regs(nrf24l01p * const dev)
{
    uint8_t reg_val;
    debug("\r\n<<<NRF24L01 REGISTERS>>>\r\nADDR - HEX\r\n");
    for (uint8_t reg_addr = 1; reg_addr <= 0x1D; reg_addr++)
    {
        reg_val = nrf24l01p_read_reg(dev, reg_addr);
        debug("0x%x", reg_addr);
        debug(" - ");
        debug("0x%x\r\n", reg_val);
    }
}

//-----------------------------------------------------------------------------
