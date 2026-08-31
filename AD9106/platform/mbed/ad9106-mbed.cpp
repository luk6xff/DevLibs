/**
 *  @brief:   AD9106 quad, low power, 12-bit, 180 MSPS DAC and waveform generator library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "ad9106-mbed.h"

//-----------------------------------------------------------------------------
static DigitalOut* ad9106_mbed_pin(const ad9106_mbed* const pd, ad9106_pin pin)
{
    switch (pin)
    {
        case AD9106_PIN_CS:
            return pd->cs;
        case AD9106_PIN_RESET:
            return pd->reset;
        case AD9106_PIN_TRIGGER:
            return pd->trigger;
        default:
            return NULL;
    }
}

//-----------------------------------------------------------------------------
bool ad9106_mbed_init(ad9106* const dev, ad9106_mbed* const mbed_dev)
{
    if (!dev || !mbed_dev || !mbed_dev->spi || !mbed_dev->cs)
    {
        return false;
    }

    // 8 bit frames, SPI mode 0 (CPOL=0, CPHA=0) as required by the AD9106
    mbed_dev->spi->format(8, 0);
    *(mbed_dev->cs) = 1;
    if (mbed_dev->reset)
    {
        *(mbed_dev->reset) = 1;
    }
    if (mbed_dev->trigger)
    {
        *(mbed_dev->trigger) = 1;
    }

    dev->platform_dev = mbed_dev;
    return ad9106_init(dev);
}

//-----------------------------------------------------------------------------
bool ad9106_spi_transfer(const ad9106* const dev, const uint8_t* tx,
                         uint8_t* rx, size_t len)
{
    const ad9106_mbed* const pd = (const ad9106_mbed*)dev->platform_dev;

    if (!pd || !tx || !rx)
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
void ad9106_set_pin(const ad9106* const dev, ad9106_pin pin, bool state)
{
    const ad9106_mbed* const pd = (const ad9106_mbed*)dev->platform_dev;

    if (!pd)
    {
        return;
    }

    DigitalOut* const out = ad9106_mbed_pin(pd, pin);
    if (!out)
    {
        // Line not wired on this board
        return;
    }
    *out = state ? 1 : 0;
}

//-----------------------------------------------------------------------------
void ad9106_delay_ms(uint32_t delay_ms)
{
    wait_us(delay_ms * 1000);
}

//-----------------------------------------------------------------------------
