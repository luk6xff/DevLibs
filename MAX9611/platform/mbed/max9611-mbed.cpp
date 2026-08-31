/**
 *  @brief:   MAX9611 high-side current-sense amplifier with 12-bit ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "max9611-mbed.h"

// mbed I2C API expects the address shifted to the 8-bit form
#define MAX9611_MBED_ADDR(dev) ((int)((dev)->i2c_addr) << 1)

//-----------------------------------------------------------------------------
bool max9611_mbed_init(max9611* const dev, max9611_mbed* const mbed_dev)
{
    if (!dev || !mbed_dev || !mbed_dev->i2c)
    {
        return false;
    }

    if (mbed_dev->i2c_freq != 0)
    {
        mbed_dev->i2c->frequency(mbed_dev->i2c_freq);
    }
    dev->platform_dev = mbed_dev;
    return max9611_init(dev);
}

//-----------------------------------------------------------------------------
bool max9611_write(const max9611* const dev, uint8_t reg_addr,
                   const uint8_t* buf, size_t buf_size)
{
    const max9611_mbed* const pd = (const max9611_mbed*)dev->platform_dev;

    // Keep the bus busy so register address and payload form a single transfer
    if (pd->i2c->write(MAX9611_MBED_ADDR(dev), (const char*)&reg_addr, 1, true) != 0)
    {
        return false;
    }
    return pd->i2c->write(MAX9611_MBED_ADDR(dev), (const char*)buf, buf_size) == 0;
}

//-----------------------------------------------------------------------------
bool max9611_read(const max9611* const dev, uint8_t reg_addr,
                  uint8_t* buf, size_t buf_size)
{
    const max9611_mbed* const pd = (const max9611_mbed*)dev->platform_dev;

    if (pd->i2c->write(MAX9611_MBED_ADDR(dev), (const char*)&reg_addr, 1, true) != 0)
    {
        return false;
    }
    return pd->i2c->read(MAX9611_MBED_ADDR(dev), (char*)buf, buf_size) == 0;
}

//-----------------------------------------------------------------------------
void max9611_delay_ms(uint32_t delay_ms)
{
    wait_us(delay_ms * 1000);
}

//-----------------------------------------------------------------------------
