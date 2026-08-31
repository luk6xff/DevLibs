/**
 *  @brief:   GTS-4E-60 (Fibocom) GPS module library, NMEA 0183 parser
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "gts4e60-mbed.h"

//-----------------------------------------------------------------------------
/**
 * @brief Free running timer used as the millisecond time base.
 */
static Timer& gts4e60_mbed_timer(void)
{
    static Timer timer;
    static bool started = false;

    if (!started)
    {
        timer.start();
        started = true;
    }
    return timer;
}

//-----------------------------------------------------------------------------
bool gts4e60_mbed_init(gts4e60* const dev, gts4e60_mbed* const mbed_dev)
{
    if (!dev || !mbed_dev || !mbed_dev->serial)
    {
        return false;
    }

    if (mbed_dev->baudrate != 0)
    {
        mbed_dev->serial->baud((int)mbed_dev->baudrate);
        dev->baudrate = mbed_dev->baudrate;
    }

    (void)gts4e60_mbed_timer();
    dev->platform_dev = mbed_dev;
    return gts4e60_init(dev);
}

//-----------------------------------------------------------------------------
bool gts4e60_uart_write(const gts4e60* const dev, const uint8_t* buf,
                        size_t len)
{
    const gts4e60_mbed* const pd = (const gts4e60_mbed*)dev->platform_dev;

    for (size_t i = 0; i < len; ++i)
    {
        if (pd->serial->putc((int)buf[i]) < 0)
        {
            return false;
        }
    }
    return true;
}

//-----------------------------------------------------------------------------
int gts4e60_uart_read_byte(const gts4e60* const dev)
{
    const gts4e60_mbed* const pd = (const gts4e60_mbed*)dev->platform_dev;

    if (!pd->serial->readable())
    {
        return -1;
    }
    return pd->serial->getc() & 0xFF;
}

//-----------------------------------------------------------------------------
bool gts4e60_uart_readable(const gts4e60* const dev)
{
    const gts4e60_mbed* const pd = (const gts4e60_mbed*)dev->platform_dev;

    return pd->serial->readable() != 0;
}

//-----------------------------------------------------------------------------
bool gts4e60_set_shutdown_pin(const gts4e60* const dev, bool state)
{
    const gts4e60_mbed* const pd = (const gts4e60_mbed*)dev->platform_dev;

    if (!pd->shutdown_pin)
    {
        return true;
    }

    const bool level = pd->shutdown_active_high ? state : !state;
    pd->shutdown_pin->write(level ? 1 : 0);
    return true;
}

//-----------------------------------------------------------------------------
void gts4e60_delay_ms(uint32_t delay_ms)
{
    wait_us(delay_ms * 1000);
}

//-----------------------------------------------------------------------------
uint32_t gts4e60_get_ms_ticks(void)
{
    return (uint32_t)gts4e60_mbed_timer().read_ms();
}

//-----------------------------------------------------------------------------
