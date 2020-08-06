
/**
 *  @brief:  Implementation of a DS321 platform dependent [MBED] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-11-15
 */

#include "ds3231-mbed.h"

//------------------------------------------------------------------------------
void ds3231_mbed_init(ds3231* const dev, ds3231_mbed* const mbed_dev)
{
    dev->platform_dev = mbed_dev;
    dev->platform_dev->i2c->frequency(400000);
    ds3231_init(dev);
}

//------------------------------------------------------------------------------
void ds3231_mbed_deinit(void)
{
    // Empty
}

//-----------------------------------------------------------------------------
bool ds3231_write(ds3231* const dev, const uint8_t* buf, const size_t buf_size)
{
    ds3231_mbed* const pd = (ds3231_mbed*)dev->platform_dev;
    return pd->i2c->write((int)dev->w_addr, (char*)buf, buf_size);
}

//-----------------------------------------------------------------------------
bool ds3231_read(ds3231* const dev, uint8_t* buf, const size_t buf_size)
{
    ds3231_mbed* const pd = (ds3231_mbed*)dev->platform_dev;
    return pd->i2c->read(dev->r_addr, (char*)buf, buf_size);
}
//-----------------------------------------------------------------------------


