
/**
 *  @brief:  Implementation of a BMP180 platform dependent [MBED] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-08-12
 */

#include "bmp180-mbed.h"

//------------------------------------------------------------------------------
void bmp180_mbed_init(bmp180 *const dev, bmp180_mbed* const mbed_dev)
{
    dev->platform_dev = mbed_dev;
    dev->platform_dev->i2c->frequency(400000);
    bmp180_init(dev);
}

//------------------------------------------------------------------------------
void bmp180_mbed_deinit(void)
{
    // Empty
}

//-----------------------------------------------------------------------------
bool bmp180_write(bmp180 *const dev, const uint8_t* buf, const size_t buf_size)
{
    bmp180_mbed* const pd = (bmp180_mbed*)dev->platform_dev;
    return pd->i2c->write((int)dev->i2c_addr, (char*)buf, buf_size) == 0;
}

//-----------------------------------------------------------------------------
bool bmp180_read(bmp180 *const dev, uint8_t* buf, const size_t buf_size)
{
    bmp180_mbed* const pd = (bmp180_mbed*)dev->platform_dev;
    return pd->i2c->read(dev->i2c_addr, (char*)buf, buf_size) == 0;
}

//-----------------------------------------------------------------------------
void bmp180_delay_ms(uint32_t delay_ms)
{
    wait_ms(delay_ms);
}

//-----------------------------------------------------------------------------
