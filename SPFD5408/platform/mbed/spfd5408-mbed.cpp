/**
 *  @brief:   SPFD5408 240x320 TFT LCD controller library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "spfd5408-mbed.h"

//-----------------------------------------------------------------------------
bool spfd5408_mbed_init(spfd5408* const dev, spfd5408_mbed* const mbed_dev)
{
    if (!dev || !mbed_dev || !mbed_dev->data || !mbed_dev->rs ||
        !mbed_dev->wr || !mbed_dev->rd || !mbed_dev->cs || !mbed_dev->reset)
    {
        return false;
    }

    mbed_dev->data->write(0);
    mbed_dev->rs->write(1);
    mbed_dev->wr->write(1);
    mbed_dev->rd->write(1);
    mbed_dev->cs->write(1);
    mbed_dev->reset->write(1);

    dev->platform_dev = mbed_dev;
    return spfd5408_init(dev);
}

//-----------------------------------------------------------------------------
void spfd5408_write_bus(const spfd5408* const dev, uint16_t value)
{
    const spfd5408_mbed* const pd = (const spfd5408_mbed*)dev->platform_dev;

    pd->data->write((int)value);
}

//-----------------------------------------------------------------------------
void spfd5408_set_pin(const spfd5408* const dev, spfd5408_pin pin, bool state)
{
    const spfd5408_mbed* const pd = (const spfd5408_mbed*)dev->platform_dev;

    DigitalOut* gpio;
    switch (pin)
    {
        case SPFD5408_PIN_RS:
            gpio = pd->rs;
            break;
        case SPFD5408_PIN_WR:
            gpio = pd->wr;
            break;
        case SPFD5408_PIN_RD:
            gpio = pd->rd;
            break;
        case SPFD5408_PIN_RESET:
            gpio = pd->reset;
            break;
        case SPFD5408_PIN_CS:
        default:
            gpio = pd->cs;
            break;
    }
    gpio->write(state ? 1 : 0);
}

//-----------------------------------------------------------------------------
void spfd5408_delay_ms(uint32_t delay_ms)
{
    wait_us(delay_ms * 1000);
}

//-----------------------------------------------------------------------------
