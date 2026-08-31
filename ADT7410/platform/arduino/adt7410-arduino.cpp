/**
 *  @brief:   ADT7410 digital temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "adt7410-arduino.h"

#define ADT7410_I2C_TIMEOUT_MS 1000

//-----------------------------------------------------------------------------
bool adt7410_arduino_init(adt7410* const dev,
                          adt7410_arduino* const arduino_dev)
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
    return adt7410_init(dev);
}

//-----------------------------------------------------------------------------
bool adt7410_write(const adt7410* const dev, uint8_t reg_addr,
                   const uint8_t* buf, size_t buf_size)
{
    adt7410_arduino* const pd = (adt7410_arduino*)dev->platform_dev;

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
bool adt7410_read(const adt7410* const dev, uint8_t reg_addr,
                  uint8_t* buf, size_t buf_size)
{
    adt7410_arduino* const pd = (adt7410_arduino*)dev->platform_dev;

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
    while (received < buf_size && (millis() - t1) < ADT7410_I2C_TIMEOUT_MS)
    {
        if (pd->i2c->available())
        {
            buf[received++] = (uint8_t)pd->i2c->read();
        }
    }
    return received == buf_size;
}

//-----------------------------------------------------------------------------
void adt7410_delay_ms(uint32_t delay_ms)
{
    delay(delay_ms);
}

//-----------------------------------------------------------------------------
