/**
 *  @brief:   MAX44009 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "max44009-arduino.h"

#define MAX44009_I2C_TIMEOUT_MS 1000

//-----------------------------------------------------------------------------
bool max44009_arduino_init(max44009* const dev,
                           max44009_arduino* const arduino_dev)
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
    return max44009_init(dev);
}

//-----------------------------------------------------------------------------
bool max44009_write(const max44009* const dev, uint8_t reg_addr,
                    const uint8_t* buf, size_t buf_size)
{
    max44009_arduino* const pd = (max44009_arduino*)dev->platform_dev;

    pd->i2c->beginTransmission(dev->i2c_addr);
    if (pd->i2c->write(reg_addr) != 1)
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
bool max44009_read(const max44009* const dev, uint8_t reg_addr,
                   uint8_t* buf, size_t buf_size)
{
    max44009_arduino* const pd = (max44009_arduino*)dev->platform_dev;

    // Address the register, keep the bus so a repeated START is issued
    pd->i2c->beginTransmission(dev->i2c_addr);
    pd->i2c->write(reg_addr);
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
    while (received < buf_size && (millis() - t1) < MAX44009_I2C_TIMEOUT_MS)
    {
        if (pd->i2c->available())
        {
            buf[received++] = (uint8_t)pd->i2c->read();
        }
    }
    return received == buf_size;
}

//-----------------------------------------------------------------------------
void max44009_delay_ms(uint32_t delay_ms)
{
    delay(delay_ms);
}

//-----------------------------------------------------------------------------

