/**
 *  @brief:   MAX44009 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "max44009-mbed.h"

// mbed I2C API expects the address shifted to the 8-bit form
#define MAX44009_MBED_ADDR(dev) ((int)((dev)->i2c_addr) << 1)

//-----------------------------------------------------------------------------
bool max44009_mbed_init(max44009* const dev, max44009_mbed* const mbed_dev)
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
    return max44009_init(dev);
}

//-----------------------------------------------------------------------------
bool max44009_write(const max44009* const dev, uint8_t reg_addr,
                    const uint8_t* buf, size_t buf_size)
{
    const max44009_mbed* const pd = (const max44009_mbed*)dev->platform_dev;

    // Keep the bus busy so register address and payload form a single transfer
    if (pd->i2c->write(MAX44009_MBED_ADDR(dev), (const char*)&reg_addr, 1, true) != 0)
    {
        return false;
    }
    return pd->i2c->write(MAX44009_MBED_ADDR(dev), (const char*)buf, buf_size) == 0;
}

//-----------------------------------------------------------------------------
bool max44009_read(const max44009* const dev, uint8_t reg_addr,
                   uint8_t* buf, size_t buf_size)
{
    const max44009_mbed* const pd = (const max44009_mbed*)dev->platform_dev;

    if (pd->i2c->write(MAX44009_MBED_ADDR(dev), (const char*)&reg_addr, 1, true) != 0)
    {
        return false;
    }
    return pd->i2c->read(MAX44009_MBED_ADDR(dev), (char*)buf, buf_size) == 0;
}

//-----------------------------------------------------------------------------
void max44009_delay_ms(uint32_t delay_ms)
{
    wait_us(delay_ms * 1000);
}

//-----------------------------------------------------------------------------

