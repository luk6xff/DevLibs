/**
 *  @brief:   AS3935 Franklin lightning sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "as3935-mbed.h"

// mbed I2C API expects the address shifted to the 8-bit form
#define AS3935_MBED_ADDR(dev) ((int)((dev)->i2c_addr) << 1)

//-----------------------------------------------------------------------------
bool as3935_mbed_init(as3935* const dev, as3935_mbed* const mbed_dev)
{
    if (!dev || !mbed_dev || !mbed_dev->i2c || !mbed_dev->irq)
    {
        return false;
    }

    if (mbed_dev->i2c_freq != 0)
    {
        mbed_dev->i2c->frequency(mbed_dev->i2c_freq);
    }
    dev->platform_dev = mbed_dev;
    return as3935_init(dev);
}

//-----------------------------------------------------------------------------
bool as3935_write(const as3935* const dev, uint8_t reg_addr,
                  const uint8_t* buf, size_t buf_size)
{
    const as3935_mbed* const pd = (const as3935_mbed*)dev->platform_dev;

    // Hold the bus between the register address and the payload
    const bool repeated = (buf_size != 0);
    if (pd->i2c->write(AS3935_MBED_ADDR(dev), (const char*)&reg_addr, 1, repeated) != 0)
    {
        return false;
    }
    if (!repeated)
    {
        return true;
    }
    return pd->i2c->write(AS3935_MBED_ADDR(dev), (const char*)buf, buf_size) == 0;
}

//-----------------------------------------------------------------------------
bool as3935_read(const as3935* const dev, uint8_t reg_addr,
                 uint8_t* buf, size_t buf_size)
{
    const as3935_mbed* const pd = (const as3935_mbed*)dev->platform_dev;

    if (pd->i2c->write(AS3935_MBED_ADDR(dev), (const char*)&reg_addr, 1, true) != 0)
    {
        return false;
    }
    return pd->i2c->read(AS3935_MBED_ADDR(dev), (char*)buf, buf_size) == 0;
}

//-----------------------------------------------------------------------------
bool as3935_get_irq_pin(const as3935* const dev)
{
    const as3935_mbed* const pd = (const as3935_mbed*)dev->platform_dev;

    return pd->irq->read() != 0;
}

//-----------------------------------------------------------------------------
void as3935_delay_ms(uint32_t delay_ms)
{
    wait_us(delay_ms * 1000);
}

//-----------------------------------------------------------------------------
