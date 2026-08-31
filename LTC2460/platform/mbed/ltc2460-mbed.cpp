/**
 *  @brief:   LTC2460 ultra-tiny 16-bit delta-sigma ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "ltc2460-mbed.h"

//-----------------------------------------------------------------------------
bool ltc2460_mbed_init(ltc2460* const dev, ltc2460_mbed* const mbed_dev)
{
    if (!dev || !mbed_dev || !mbed_dev->spi || !mbed_dev->cs)
    {
        return false;
    }

    // 8 bit frames, SPI mode 0
    mbed_dev->spi->format(8, 0);
    *(mbed_dev->cs) = 1;

    dev->platform_dev = mbed_dev;
    return ltc2460_init(dev);
}

//-----------------------------------------------------------------------------
bool ltc2460_spi_transfer(const ltc2460* const dev, const uint8_t* tx,
                          uint8_t* rx, size_t len)
{
    const ltc2460_mbed* const pd = (const ltc2460_mbed*)dev->platform_dev;

    if (!tx || !rx)
    {
        return false;
    }

    for (size_t i = 0; i < len; ++i)
    {
        rx[i] = (uint8_t)pd->spi->write((int)tx[i]);
    }
    return true;
}

//-----------------------------------------------------------------------------
void ltc2460_set_cs_pin(const ltc2460* const dev, bool state)
{
    const ltc2460_mbed* const pd = (const ltc2460_mbed*)dev->platform_dev;

    *(pd->cs) = state ? 1 : 0;
}

//-----------------------------------------------------------------------------
void ltc2460_delay_ms(uint32_t delay_ms)
{
    wait_us(delay_ms * 1000);
}

//-----------------------------------------------------------------------------
