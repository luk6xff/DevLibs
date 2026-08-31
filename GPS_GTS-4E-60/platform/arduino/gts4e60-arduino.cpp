/**
 *  @brief:   GTS-4E-60 (Fibocom) GPS module library, NMEA 0183 parser
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "gts4e60-arduino.h"

//-----------------------------------------------------------------------------
bool gts4e60_arduino_init(gts4e60* const dev,
                          gts4e60_arduino* const arduino_dev)
{
    if (!dev || !arduino_dev || !arduino_dev->serial)
    {
        return false;
    }

    if (arduino_dev->baudrate != 0)
    {
        dev->baudrate = arduino_dev->baudrate;
    }

    if (arduino_dev->shutdown_pin != GTS4E60_ARDUINO_NO_PIN)
    {
        pinMode(arduino_dev->shutdown_pin, OUTPUT);
    }

    dev->platform_dev = arduino_dev;
    return gts4e60_init(dev);
}

//-----------------------------------------------------------------------------
bool gts4e60_uart_write(const gts4e60* const dev, const uint8_t* buf,
                        size_t len)
{
    const gts4e60_arduino* const pd = (const gts4e60_arduino*)dev->platform_dev;

    return pd->serial->write(buf, len) == len;
}

//-----------------------------------------------------------------------------
int gts4e60_uart_read_byte(const gts4e60* const dev)
{
    const gts4e60_arduino* const pd = (const gts4e60_arduino*)dev->platform_dev;

    return pd->serial->read();
}

//-----------------------------------------------------------------------------
bool gts4e60_uart_readable(const gts4e60* const dev)
{
    const gts4e60_arduino* const pd = (const gts4e60_arduino*)dev->platform_dev;

    return pd->serial->available() > 0;
}

//-----------------------------------------------------------------------------
bool gts4e60_set_shutdown_pin(const gts4e60* const dev, bool state)
{
    const gts4e60_arduino* const pd = (const gts4e60_arduino*)dev->platform_dev;

    if (pd->shutdown_pin == GTS4E60_ARDUINO_NO_PIN)
    {
        return true;
    }

    const bool level = pd->shutdown_active_high ? state : !state;
    digitalWrite(pd->shutdown_pin, level ? HIGH : LOW);
    return true;
}

//-----------------------------------------------------------------------------
void gts4e60_delay_ms(uint32_t delay_ms)
{
    delay(delay_ms);
}

//-----------------------------------------------------------------------------
uint32_t gts4e60_get_ms_ticks(void)
{
    return (uint32_t)millis();
}

//-----------------------------------------------------------------------------
