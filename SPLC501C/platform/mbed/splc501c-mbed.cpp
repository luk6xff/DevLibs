/**
 *  @brief:   SPLC501C 128x64 monochrome graphic LCD library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "splc501c-mbed.h"

//-----------------------------------------------------------------------------
bool splc501c_mbed_init(splc501c* const dev, splc501c_mbed* const mbed_dev)
{
    if (!dev || !mbed_dev || !mbed_dev->spi || !mbed_dev->cs ||
        !mbed_dev->a0 || !mbed_dev->reset)
    {
        return false;
    }

    mbed_dev->spi->format(8, 0);
    if (mbed_dev->spi_freq != 0)
    {
        mbed_dev->spi->frequency((int)mbed_dev->spi_freq);
    }

    mbed_dev->cs->write(1);
    mbed_dev->a0->write(0);
    mbed_dev->reset->write(1);

    dev->platform_dev = mbed_dev;
    return splc501c_init(dev);
}

//-----------------------------------------------------------------------------
bool splc501c_write_bytes(const splc501c* const dev, const uint8_t* buf,
                          size_t len)
{
    const splc501c_mbed* const pd = (const splc501c_mbed*)dev->platform_dev;

    if (!buf || len == 0)
    {
        return false;
    }

    for (size_t i = 0; i < len; i++)
    {
        (void)pd->spi->write((int)buf[i]);
    }
    return true;
}

//-----------------------------------------------------------------------------
void splc501c_set_pin(const splc501c* const dev, splc501c_pin pin, bool state)
{
    const splc501c_mbed* const pd = (const splc501c_mbed*)dev->platform_dev;

    DigitalOut* gpio;
    switch (pin)
    {
        case SPLC501C_PIN_A0:
            gpio = pd->a0;
            break;
        case SPLC501C_PIN_RESET:
            gpio = pd->reset;
            break;
        case SPLC501C_PIN_CS:
        default:
            gpio = pd->cs;
            break;
    }
    gpio->write(state ? 1 : 0);
}

//-----------------------------------------------------------------------------
void splc501c_delay_ms(uint32_t delay_ms)
{
    wait_us(delay_ms * 1000);
}

//-----------------------------------------------------------------------------
