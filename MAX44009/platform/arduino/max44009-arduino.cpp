/**
 *  @brief:   MAX44009 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "max44009-arduino.h"

//-------------------------------------------------------------------------------
bool max44009_arduino_init(max44009* const dev, max44009_arduino* const arduino_dev)
{
    arduino_dev->i2c->begin();
    arduino_dev->i2c->setClock(400000);
    dev->platform_dev = arduino_dev;
    return max44009_init(dev);
}
//------------------------------------------------------------------------------
bool max44009_write(const max44009* const dev, uint8_t reg_addr,
                    const uint8_t* buf, size_t buf_size)
{
    max44009_arduino* const pd = (max44009_arduino*)dev->platform_dev;
    pd->i2c->beginTransmission(dev->i2c_addr);
    pd->i2c->write(reg_addr);
    for (uint8_t i = 0; i < buf_size; i++)
    {
        pd->i2c->write((uint8_t) buf[i]);
    }
    return pd->i2c->endTransmission() == 0; // true on success
}

//------------------------------------------------------------------------------
bool max44009_read(const max44009* const dev, uint8_t reg_addr,
                   uint8_t* buf, size_t buf_size)
{
    const size_t timeout_ms = 1000; // 1 second timeout
    size_t recv_data_cntr = 0;
    max44009_arduino* const pd = (max44009_arduino*)dev->platform_dev;

    pd->i2c->beginTransmission(dev->i2c_addr);
    // Start transmission by write to a given register
    pd->i2c->write(reg_addr);
    // Read data
    pd->i2c->requestFrom(dev->i2c_addr, buf_size);
    const size_t t1 = millis();
    for (; pd->i2c->available() && (timeout_ms == 0 || (millis() - t1) < timeout_ms); recv_data_cntr++)
    {
        buf[recv_data_cntr] = pd->i2c->read();
    }
    return pd->i2c->endTransmission() == 0 && recv_data_cntr == buf_size;
}

//------------------------------------------------------------------------------
