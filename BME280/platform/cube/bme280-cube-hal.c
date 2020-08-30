
/**
 *  @brief:  Implementation of a BME280 platform dependent [STM32 CUBE HAL] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-08-18
 */

#include "bme280-cube-hal.h"

//------------------------------------------------------------------------------
bool bme280_cube_hal_init(bme280 *const dev, bme280_cube_hal* const cube_hal_dev)
{
    dev->platform_dev = cube_hal_dev;
    return bme280_init(dev);
}

//------------------------------------------------------------------------------
bool bme280_cube_hal_deinit(void)
{
    // Empty
    return true;
}

//-----------------------------------------------------------------------------
bool bme280_write(bme280 *const dev, const uint8_t* buf, const size_t buf_size)
{
    bme280_cube_hal* const pd = (bme280_cube_hal*)dev->platform_dev;
    HAL_StatusTypeDef status;
    if (dev->intf == BME280_INTF_I2C)
    {
        status = HAL_I2C_Master_Transmit(pd->i2c, dev->i2c_addr, buf, buf_size, 1000);
    }
    else // dev->intf == BME280_INTF_SPI
    {
        HAL_GPIO_WritePin(pd->spi_cs_port, pd->spi_cs_pin, GPIO_PIN_RESET);
        status = HAL_SPI_Transmit(pd->spi, (uint8_t*)buf, buf_size, 1000);
        HAL_GPIO_WritePin(pd->spi_cs_port, pd->spi_cs_pin, GPIO_PIN_SET);
    }
    if (status != HAL_OK)
    {
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool bme280_read(bme280 *const dev, uint8_t* buf, const size_t buf_size)
{
    bme280_cube_hal* const pd = (bme280_cube_hal*)dev->platform_dev;
    HAL_StatusTypeDef status;
    if (dev->intf == BME280_INTF_I2C)
    {
        status = HAL_I2C_Master_Receive(pd->i2c, dev->i2c_addr, buf, buf_size, 1000);
    }
    else // dev->intf == BME280_INTF_SPI
    {
        HAL_GPIO_WritePin(pd->spi_cs_port, pd->spi_cs_pin, GPIO_PIN_RESET);
        status = HAL_SPI_Receive(pd->spi, buf, buf_size, 1000);
        HAL_GPIO_WritePin(pd->spi_cs_port, pd->spi_cs_pin, GPIO_PIN_SET);
    }
    if (status != HAL_OK)
    {
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
void bme280_delay_ms(uint32_t delay_ms)
{
    uint32_t tickstart_ms = HAL_GetTick();
    while ((HAL_GetTick()-tickstart_ms) < delay_ms);
}

//-----------------------------------------------------------------------------
