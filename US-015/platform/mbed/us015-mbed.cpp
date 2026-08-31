/**
 *  @brief:   US-015 (HC-SR04 compatible) ultrasonic distance sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "us015-mbed.h"

//-----------------------------------------------------------------------------
bool us015_mbed_init(us015* const dev, us015_mbed* const mbed_dev)
{
    if (!dev || !mbed_dev || !mbed_dev->trigger || !mbed_dev->echo)
    {
        return false;
    }

    *(mbed_dev->trigger) = 0;

    dev->platform_dev = mbed_dev;
    return us015_init(dev);
}

//-----------------------------------------------------------------------------
void us015_set_trigger_pin(const us015* const dev, bool state)
{
    const us015_mbed* const pd = (const us015_mbed*)dev->platform_dev;

    *(pd->trigger) = state ? 1 : 0;
}

//-----------------------------------------------------------------------------
bool us015_get_echo_pin(const us015* const dev)
{
    const us015_mbed* const pd = (const us015_mbed*)dev->platform_dev;

    return pd->echo->read() != 0;
}

//-----------------------------------------------------------------------------
void us015_delay_us(uint32_t delay_us)
{
    wait_us(delay_us);
}

//-----------------------------------------------------------------------------
uint32_t us015_get_us_ticks(void)
{
    // Free running microsecond ticker, wrap around is handled by the core
    return (uint32_t)us_ticker_read();
}

//-----------------------------------------------------------------------------
