/**
 *  @brief:   DS1621 digital thermometer and thermostat library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "ds1621-arduino.h"

#define DS1621_I2C_TIMEOUT_MS 1000

//-----------------------------------------------------------------------------
bool ds1621_arduino_init(ds1621* const dev, ds1621_arduino* const arduino_dev)
{
    if (!dev || !arduino_dev || !arduino_dev->i2c)
    {
        return false;
    }

    arduino_dev->i2c->begin();
    if (arduino_dev->i2c_freq != 0)
    {
        arduino_dev->i2c->setClock(arduino_dev->i2c_freq);
    }
    dev->platform_dev = arduino_dev;
    return ds1621_init(dev);
}

//-----------------------------------------------------------------------------
bool ds1621_write(const ds1621* const dev, uint8_t cmd,
                  const uint8_t* buf, size_t buf_size)
{
    ds1621_arduino* const pd = (ds1621_arduino*)dev->platform_dev;

    pd->i2c->beginTransmission(dev->i2c_addr);
    if (pd->i2c->write(cmd) != 1)
    {
        pd->i2c->endTransmission();
        return false;
    }
    if (buf_size != 0 && pd->i2c->write(buf, buf_size) != buf_size)
    {
        pd->i2c->endTransmission();
        return false;
    }
    return pd->i2c->endTransmission() == 0;
}

//-----------------------------------------------------------------------------
bool ds1621_read(const ds1621* const dev, uint8_t cmd,
                 uint8_t* buf, size_t buf_size)
{
    ds1621_arduino* const pd = (ds1621_arduino*)dev->platform_dev;

    // Send the command, keep the bus so a repeated START is issued
    pd->i2c->beginTransmission(dev->i2c_addr);
    pd->i2c->write(cmd);
    if (pd->i2c->endTransmission(false) != 0)
    {
        return false;
    }

    if (pd->i2c->requestFrom(dev->i2c_addr, (uint8_t)buf_size) != buf_size)
    {
        return false;
    }

    size_t received = 0;
    const uint32_t t1 = millis();
    while (received < buf_size && (millis() - t1) < DS1621_I2C_TIMEOUT_MS)
    {
        if (pd->i2c->available())
        {
            buf[received++] = (uint8_t)pd->i2c->read();
        }
    }
    return received == buf_size;
}

//-----------------------------------------------------------------------------
void ds1621_delay_ms(uint32_t delay_ms)
{
    delay(delay_ms);
}

//-----------------------------------------------------------------------------
