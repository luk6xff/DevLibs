/**
 *  @brief:   SI7020 relative humidity and temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "si7020-mbed.h"

// mbed I2C API expects the address shifted to the 8-bit form
#define SI7020_MBED_ADDR(dev) ((int)((dev)->i2c_addr) << 1)

//-----------------------------------------------------------------------------
bool si7020_mbed_init(si7020* const dev, si7020_mbed* const mbed_dev)
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
    return si7020_init(dev);
}

//-----------------------------------------------------------------------------
bool si7020_write_cmd(const si7020* const dev, const uint8_t* cmd,
                      size_t cmd_size)
{
    const si7020_mbed* const pd = (const si7020_mbed*)dev->platform_dev;

    return pd->i2c->write(SI7020_MBED_ADDR(dev), (const char*)cmd, cmd_size) == 0;
}

//-----------------------------------------------------------------------------
bool si7020_read_raw(const si7020* const dev, uint8_t* buf, size_t buf_size)
{
    const si7020_mbed* const pd = (const si7020_mbed*)dev->platform_dev;

    // The device answers the previously latched command, nothing is sent here
    return pd->i2c->read(SI7020_MBED_ADDR(dev), (char*)buf, buf_size) == 0;
}

//-----------------------------------------------------------------------------
void si7020_delay_ms(uint32_t delay_ms)
{
    wait_us(delay_ms * 1000);
}

//-----------------------------------------------------------------------------
