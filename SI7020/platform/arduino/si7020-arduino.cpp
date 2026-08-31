/**
 *  @brief:   SI7020 relative humidity and temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "si7020-arduino.h"

#define SI7020_I2C_TIMEOUT_MS 1000

//-----------------------------------------------------------------------------
bool si7020_arduino_init(si7020* const dev, si7020_arduino* const arduino_dev)
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
    return si7020_init(dev);
}

//-----------------------------------------------------------------------------
bool si7020_write_cmd(const si7020* const dev, const uint8_t* cmd,
                      size_t cmd_size)
{
    si7020_arduino* const pd = (si7020_arduino*)dev->platform_dev;

    pd->i2c->beginTransmission(dev->i2c_addr);
    if (pd->i2c->write(cmd, cmd_size) != cmd_size)
    {
        pd->i2c->endTransmission();
        return false;
    }
    return pd->i2c->endTransmission() == 0;
}

//-----------------------------------------------------------------------------
bool si7020_read_raw(const si7020* const dev, uint8_t* buf, size_t buf_size)
{
    si7020_arduino* const pd = (si7020_arduino*)dev->platform_dev;

    // The device answers the previously latched command, nothing is sent here
    if (pd->i2c->requestFrom(dev->i2c_addr, (uint8_t)buf_size) != buf_size)
    {
        return false;
    }

    size_t received = 0;
    const uint32_t t1 = millis();
    while (received < buf_size && (millis() - t1) < SI7020_I2C_TIMEOUT_MS)
    {
        if (pd->i2c->available())
        {
            buf[received++] = (uint8_t)pd->i2c->read();
        }
    }
    return received == buf_size;
}

//-----------------------------------------------------------------------------
void si7020_delay_ms(uint32_t delay_ms)
{
    delay(delay_ms);
}

//-----------------------------------------------------------------------------
