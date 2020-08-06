
/**
 *  @brief:  Implementation of a DS321 platform dependent [ARDUINO] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-11-15
 */

#include "ds3231-arduino.h"

//------------------------------------------------------------------------------
void ds3231_arduino_init(ds3231* const dev, ds3231_arduino* const arduino_dev)
{
    arduino_dev->i2c->begin(dev->i2c_addr);
    arduino_dev->i2c->setClock(400000);
    dev->platform_dev = arduino_dev;
    ds3231_init(dev);
}

//------------------------------------------------------------------------------
void ds3231_arduino_deinit(ds3231* const dev)
{
    // Empty
}

//-----------------------------------------------------------------------------
bool ds3231_write(ds3231* const dev, const uint8_t* buf, const size_t buf_size)
{
    ds3231_arduino* const pd = (ds3231_arduino*)dev->platform_dev;
    pd->i2c->beginTransmission(dev->w_addr);
    for (uint8_t i = 0; i < buf_size; i++)
    {
        pd->i2c->write((uint8_t) buf[i]);
    }
    return pd->i2c->endTransmission() != 0; // 0 on success
}

//-----------------------------------------------------------------------------
bool ds3231_read(ds3231* const dev, uint8_t* buf, const size_t buf_size)
{
    const size_t timeout_ms = 1000; // 1 second timeout
    size_t recv_data_cntr = 0;


    ds3231_arduino* const pd = (ds3231_arduino*)dev->platform_dev;
    // Read data
    pd->i2c->beginTransmission(dev->r_addr);
    pd->i2c->requestFrom(dev->r_addr, buf_size);
    const size_t t1 = millis();
    for (; pd->i2c->available() && (timeout_ms == 0 || (millis() - t1) < timeout_ms); recv_data_cntr++)
    {
        buf[recv_data_cntr ] = pd->i2c->read();
    }
    return pd->i2c->endTransmission() != 0 && recv_data_cntr != buf_size;
}
//-----------------------------------------------------------------------------


