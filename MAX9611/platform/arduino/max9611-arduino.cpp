/**
 *  @brief:   MAX9611 high-side current-sense amplifier with 12-bit ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "max9611-arduino.h"

#define MAX9611_I2C_TIMEOUT_MS 1000

//-----------------------------------------------------------------------------
bool max9611_arduino_init(max9611* const dev,
                          max9611_arduino* const arduino_dev)
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
    return max9611_init(dev);
}

//-----------------------------------------------------------------------------
bool max9611_write(const max9611* const dev, uint8_t reg_addr,
                   const uint8_t* buf, size_t buf_size)
{
    max9611_arduino* const pd = (max9611_arduino*)dev->platform_dev;

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
bool max9611_read(const max9611* const dev, uint8_t reg_addr,
                  uint8_t* buf, size_t buf_size)
{
    max9611_arduino* const pd = (max9611_arduino*)dev->platform_dev;

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
    while (received < buf_size && (millis() - t1) < MAX9611_I2C_TIMEOUT_MS)
    {
        if (pd->i2c->available())
        {
            buf[received++] = (uint8_t)pd->i2c->read();
        }
    }
    return received == buf_size;
}

//-----------------------------------------------------------------------------
void max9611_delay_ms(uint32_t delay_ms)
{
    delay(delay_ms);
}

//-----------------------------------------------------------------------------
