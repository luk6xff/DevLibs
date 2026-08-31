/**
 *  @brief:   BTM222 Bluetooth SPP module library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "btm222-arduino.h"

//-----------------------------------------------------------------------------
bool btm222_arduino_init(btm222* const dev, btm222_arduino* const arduino_dev)
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
    return btm222_init(dev);
}

//-----------------------------------------------------------------------------
bool btm222_uart_write(const btm222* const dev, const uint8_t* buf, size_t len)
{
    const btm222_arduino* const pd = (const btm222_arduino*)dev->platform_dev;

    return pd->serial->write(buf, len) == len;
}

//-----------------------------------------------------------------------------
int btm222_uart_read_byte(const btm222* const dev)
{
    const btm222_arduino* const pd = (const btm222_arduino*)dev->platform_dev;

    return pd->serial->read();
}

//-----------------------------------------------------------------------------
bool btm222_uart_readable(const btm222* const dev)
{
    const btm222_arduino* const pd = (const btm222_arduino*)dev->platform_dev;

    return pd->serial->available() > 0;
}

//-----------------------------------------------------------------------------
void btm222_delay_ms(uint32_t delay_ms)
{
    delay(delay_ms);
}

//-----------------------------------------------------------------------------
uint32_t btm222_get_ms_ticks(void)
{
    return (uint32_t)millis();
}

//-----------------------------------------------------------------------------
