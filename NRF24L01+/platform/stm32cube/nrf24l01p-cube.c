
/**
 *  @brief:  Implementation of a NRF24L01 platform dependent [STM32CUBE] radio functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-12-01
 */

#include "nrf24l01p-cube.h"


//------------------------------------------------------------------------------
void nrf24l01p_cube_init(nrf24l01p * const dev, nrf24l01p_cube* const cube_dev)
{
    dev->platform_dev = cube_dev;
    nrf24l01p_init(dev);
}

//------------------------------------------------------------------------------
void nrf24l01p_cube_deinit(nrf24l01p * const dev)
{
	// Empty
}

//------------------------------------------------------------------------------
void nrf24l01p_set_ce_pin(nrf24l01p * const dev, bool enable)
{
    const nrf24l01p_cube * const pd = (nrf24l01p_cube*)dev->platform_dev;
    if (enable)
    {
        HAL_GPIO_WritePin(pd->ce_port, pd->ce_pin, GPIO_PIN_SET);
        return;
    }
    HAL_GPIO_WritePin(pd->ce_port, pd->ce_pin, GPIO_PIN_RESET);
}


//------------------------------------------------------------------------------
bool nrf24l01p_get_ce_pin(nrf24l01p * const dev)
{
    const nrf24l01p_cube * const pd = (nrf24l01p_cube*)dev->platform_dev;
	return HAL_GPIO_ReadPin(pd->ce_port, pd->ce_pin);
}

//------------------------------------------------------------------------------
void nrf24l01p_set_csn_pin(nrf24l01p * const dev, bool enable)
{
    const nrf24l01p_cube * const pd = (nrf24l01p_cube*)dev->platform_dev;
    if (enable)
    {
    	HAL_GPIO_WritePin(pd->csn_port, pd->csn_pin, GPIO_PIN_SET);
        return;
    }
	HAL_GPIO_WritePin(pd->csn_port, pd->csn_pin, GPIO_PIN_RESET);
}

//------------------------------------------------------------------------------
uint8_t nrf24l01p_spi_write(nrf24l01p * const dev, uint8_t data)
{
    const nrf24l01p_cube * const pd = (nrf24l01p_cube*)dev->platform_dev;
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(pd->spi, &data, &rx_data, 1, 100);
    return rx_data;
}

//------------------------------------------------------------------------------
void nrf24l01p_delay_ms(uint32_t delay_ms)
{
    uint32_t tickstart_ms = HAL_GetTick();
    while ((HAL_GetTick()-tickstart_ms) < delay_ms);
}

//------------------------------------------------------------------------------
void nrf24l01p_print_all_regs(nrf24l01p * const dev)
{
    uint8_t reg_val;
    printf("\r\n<<<NRF24L01 REGISTERS>>>\r\nADDR - HEX\r\n");
    for (uint8_t reg_addr = 0; reg_addr <= 0x1D; reg_addr++)
    {
        reg_val = nrf24l01p_read_reg(dev, reg_addr);
        printf("0x%02x", reg_addr);
        printf(" - ");
        printf("0x%02x\r\n", reg_val);
    }
}

//------------------------------------------------------------------------------
void nrf24l01p_print_chip_info(nrf24l01p * const dev)
{
    printf("\r\n<<<NRF24L01 CHIP INFO>>>\r\n");
    printf( "nRF24L01+ Frequency    : %d MHz\r\n",  nrf24l01p_get_rf_freq(dev) );
    printf( "nRF24L01+ Output power : %d dBm\r\n",  nrf24l01p_get_rf_tx_power(dev) );
    printf( "nRF24L01+ Data Rate    : %d kbps\r\n", nrf24l01p_get_data_rate(dev) );
    printf( "nRF24L01+ Transfer Size: %d bytes\r\n", nrf24l01p_get_transfer_size(dev, 0) );
    printf( "nRF24L01+ TX Address   : 0x%x\r\n", nrf24l01p_get_tx_addr(dev) );
    printf( "nRF24L01+ RX Address   : 0x%x\r\n", nrf24l01p_get_rx_addr(dev, 0) );
}


//-----------------------------------------------------------------------------
