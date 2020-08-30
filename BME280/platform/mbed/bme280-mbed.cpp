
/**
 *  @brief:  Implementation of a BME280 platform dependent [MBED] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-08-18
 */

#include "bme280-mbed.h"

//------------------------------------------------------------------------------
bool bme280_mbed_init(bme280 *const dev, bme280_mbed* const mbed_dev)
{
    dev->platform_dev = mbed_dev;

    if (dev->intf == BME280_INTF_I2C)
    {
        // Init I2C
        dev->platform_dev->i2c->frequency(400000);
    }
    else // dev->intf == BME280_INTF_SPI
    {
        // Init SPI
        *(dev->platform_dev->spi_cs) = 1;
        dev->platform_dev->spi->format(8,0);
        uint32_t spi_freq = 8000000;
        dev->platform_dev->spi->frequency(spi_freq);
    }

    return bme280_init(dev);
}

//------------------------------------------------------------------------------
bool bme280_mbed_deinit(void)
{
    // Empty
	return true;
}

//-----------------------------------------------------------------------------
bool bme280_write(bme280 *const dev, const uint8_t* buf, const size_t buf_size)
{
    bme280_mbed* const pd = (bme280_mbed*)dev->platform_dev;
    if (dev->intf == BME280_INTF_I2C)
    {
        return pd->i2c->write((int)dev->i2c_addr, (char*)buf, buf_size) == 0;
    }
    else // dev->intf == BME280_INTF_SPI
    {
        *(pd->spi_cs) = 0;
        for (uint8_t i = 0; i < buf_size; i++)
        {
            pd->spi->write(buf[i]);
        }
        *(pd->spi_cs) = 1;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool bme280_read(bme280 *const dev, uint8_t* buf, const size_t buf_size)
{
    bme280_mbed* const pd = (bme280_mbed*)dev->platform_dev;
    if (dev->intf == BME280_INTF_I2C)
    {
        return pd->i2c->read(dev->i2c_addr, (char*)buf, buf_size) == 0;
    }
    else // dev->intf == BME280_INTF_SPI
    {
        *(pd->spi_cs) = 0;
        for (uint8_t i = 0; i < buf_size; i++)
        {
            buf[i] = pd->spi->write(0);
        }
        *(pd->spi_cs) = 1;
    }
    return true;
}

//-----------------------------------------------------------------------------
void bme280_delay_ms(uint32_t delay_ms)
{
    wait_ms(delay_ms);
}

//-----------------------------------------------------------------------------
