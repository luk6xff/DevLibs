/**
 *  @brief:   HM-11 Bluetooth Low Energy 4.0 module library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "hm11-arduino.h"

//-----------------------------------------------------------------------------
bool hm11_arduino_init(hm11* const dev, hm11_arduino* const arduino_dev)
{
    if (!dev || !arduino_dev || !arduino_dev->serial)
    {
        return false;
    }

    if (arduino_dev->baudrate != 0)
    {
        dev->baudrate = arduino_dev->baudrate;
    }

    dev->platform_dev = arduino_dev;
    return hm11_init(dev);
}

//-----------------------------------------------------------------------------
bool hm11_uart_write(const hm11* const dev, const uint8_t* buf, size_t len)
{
    const hm11_arduino* const pd = (const hm11_arduino*)dev->platform_dev;

    return pd->serial->write(buf, len) == len;
}

//-----------------------------------------------------------------------------
int hm11_uart_read_byte(const hm11* const dev)
{
    const hm11_arduino* const pd = (const hm11_arduino*)dev->platform_dev;

    return pd->serial->read();
}

//-----------------------------------------------------------------------------
bool hm11_uart_readable(const hm11* const dev)
{
    const hm11_arduino* const pd = (const hm11_arduino*)dev->platform_dev;

    return pd->serial->available() > 0;
}

//-----------------------------------------------------------------------------
void hm11_delay_ms(uint32_t delay_ms)
{
    delay(delay_ms);
}

//-----------------------------------------------------------------------------
uint32_t hm11_get_ms_ticks(void)
{
    return (uint32_t)millis();
}

//-----------------------------------------------------------------------------
