/**
 *  @brief:   DS1621 digital thermometer and thermostat library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "ds1621-mbed.h"

// mbed I2C API expects the address shifted to the 8-bit form
#define DS1621_MBED_ADDR(dev) ((int)((dev)->i2c_addr) << 1)

//-----------------------------------------------------------------------------
bool ds1621_mbed_init(ds1621* const dev, ds1621_mbed* const mbed_dev)
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
    return ds1621_init(dev);
}

//-----------------------------------------------------------------------------
bool ds1621_write(const ds1621* const dev, uint8_t cmd,
                  const uint8_t* buf, size_t buf_size)
{
    const ds1621_mbed* const pd = (const ds1621_mbed*)dev->platform_dev;

    // Hold the bus between the command byte and the payload
    const bool repeated = (buf_size != 0);
    if (pd->i2c->write(DS1621_MBED_ADDR(dev), (const char*)&cmd, 1, repeated) != 0)
    {
        return false;
    }
    if (!repeated)
    {
        return true;
    }
    return pd->i2c->write(DS1621_MBED_ADDR(dev), (const char*)buf, buf_size) == 0;
}

//-----------------------------------------------------------------------------
bool ds1621_read(const ds1621* const dev, uint8_t cmd,
                 uint8_t* buf, size_t buf_size)
{
    const ds1621_mbed* const pd = (const ds1621_mbed*)dev->platform_dev;

    if (pd->i2c->write(DS1621_MBED_ADDR(dev), (const char*)&cmd, 1, true) != 0)
    {
        return false;
    }
    return pd->i2c->read(DS1621_MBED_ADDR(dev), (char*)buf, buf_size) == 0;
}

//-----------------------------------------------------------------------------
void ds1621_delay_ms(uint32_t delay_ms)
{
    wait_us(delay_ms * 1000);
}

//-----------------------------------------------------------------------------
