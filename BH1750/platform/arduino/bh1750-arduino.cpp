/**
 *  @brief:   BH1750 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-10-02
 *  @license: MIT
 */

#include "bh1750-arduino.h"

//-------------------------------------------------------------------------------
bool bh1750_arduino_init(bh1750 *const dev, bh1750_arduino *const arduino_dev)
{
    arduino_dev->i2c->begin();
    arduino_dev->i2c->setClock(400000);
    dev->platform_dev = arduino_dev;
    return bh1750_init(dev);
}
//------------------------------------------------------------------------------
bool bh1750_write_instruction(bh1750 *const dev, uint8_t instruction)
{
    bh1750_arduino *const pd = (bh1750_arduino*)dev->platform_dev;
    pd->i2c->beginTransmission(dev->i2c_addr);
    pd->i2c->write(instruction);
    return pd->i2c->endTransmission() == 0; // true on success
}

//------------------------------------------------------------------------------
bool bh1750_read(const bh1750 *const dev, uint8_t *buf, size_t buf_size)
{
    const size_t timeout_ms = 1000; // 1 second timeout
    size_t recv_data_cntr = 0;
    bh1750_arduino *const pd = (bh1750_arduino*)dev->platform_dev;

    pd->i2c->beginTransmission(dev->i2c_addr);
    // Read data
    pd->i2c->requestFrom(dev->i2c_addr, buf_size);
    const size_t t1 = millis();
    for (; pd->i2c->available() && (timeout_ms == 0 || (millis() - t1) < timeout_ms);
            recv_data_cntr++)
    {
        buf[recv_data_cntr] = pd->i2c->read();
    }
    return pd->i2c->endTransmission() == 0 && recv_data_cntr == buf_size;
}

//------------------------------------------------------------------------------
