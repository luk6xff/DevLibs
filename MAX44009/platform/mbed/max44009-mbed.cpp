#include "max44009-mbed.h"




//-----------------------------------------------------------------------------
bool max44009_mbed_init(max44009* const dev, max44009_mbed* const mbed_dev)
{
    dev->platform_dev = mbed_dev;
    return max44009_init(dev);
}

//-----------------------------------------------------------------------------
bool max44009_write(const max44009* const dev, uint8_t reg_addr,
                    const uint8_t* buf, size_t buf_size)
{
    const max44009_mbed* const pd = (max44009_mbed*)dev->platform_dev;

    int ack = pd->i2c->write((int)dev->addr, (char*)&reg_addr, 1, true);
    ack     = pd->i2c->write((int)dev->addr, (char*)buf, buf_size);
    if (ack != 0)
    {
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool max44009_read(const max44009* const dev, uint8_t reg_addr,
                   uint8_t* buf, size_t buf_size)
{
    const max44009_mbed* const pd = (max44009_mbed*)dev->platform_dev;

    // Write addr
    int ack = pd->i2c->write((int)dev->addr, (char*)&reg_addr, 1, true);
    if (ack != 0)
    {
        return false;
    }
    // Sequential Read
    ack = pd->i2c->read(dev->addr, (char*)buf, buf_size);
    if (ack != 0)
    {
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
