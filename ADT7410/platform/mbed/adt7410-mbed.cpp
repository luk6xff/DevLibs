/**
 *  @brief:   ADT7410 digital temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "adt7410-mbed.h"

// mbed I2C API expects the address shifted to the 8-bit form
#define ADT7410_MBED_ADDR(dev) ((int)((dev)->i2c_addr) << 1)

//-----------------------------------------------------------------------------
bool adt7410_mbed_init(adt7410* const dev, adt7410_mbed* const mbed_dev)
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
    return adt7410_init(dev);
}

//-----------------------------------------------------------------------------
bool adt7410_write(const adt7410* const dev, uint8_t reg_addr,
                   const uint8_t* buf, size_t buf_size)
{
    const adt7410_mbed* const pd = (const adt7410_mbed*)dev->platform_dev;

    // Hold the bus between the register address and the payload
    const bool repeated = (buf_size != 0);
    if (pd->i2c->write(ADT7410_MBED_ADDR(dev), (const char*)&reg_addr, 1, repeated) != 0)
    {
        return false;
    }
    if (!repeated)
    {
        return true;
    }
    return pd->i2c->write(ADT7410_MBED_ADDR(dev), (const char*)buf, buf_size) == 0;
}

//-----------------------------------------------------------------------------
bool adt7410_read(const adt7410* const dev, uint8_t reg_addr,
                  uint8_t* buf, size_t buf_size)
{
    const adt7410_mbed* const pd = (const adt7410_mbed*)dev->platform_dev;

    if (pd->i2c->write(ADT7410_MBED_ADDR(dev), (const char*)&reg_addr, 1, true) != 0)
    {
        return false;
    }
    return pd->i2c->read(ADT7410_MBED_ADDR(dev), (char*)buf, buf_size) == 0;
}

//-----------------------------------------------------------------------------
void adt7410_delay_ms(uint32_t delay_ms)
{
    wait_us(delay_ms * 1000);
}

//-----------------------------------------------------------------------------
