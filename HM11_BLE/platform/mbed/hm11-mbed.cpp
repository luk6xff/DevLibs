/**
 *  @brief:   HM-11 Bluetooth Low Energy 4.0 module library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "hm11-mbed.h"

// Device serviced by the receive interrupt handler
static hm11* g_hm11_irq_dev = NULL;

//-----------------------------------------------------------------------------
/**
 * @brief Free running timer used as the millisecond time base.
 */
static Timer& hm11_mbed_timer(void)
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
static void hm11_mbed_rx_isr(void)
{
    hm11* const dev = g_hm11_irq_dev;
    if (!dev)
    {
        return;
    }

    const hm11_mbed* const pd = (const hm11_mbed*)dev->platform_dev;
    while (pd->serial->readable())
    {
        (void)hm11_rx_push(dev, (uint8_t)(pd->serial->getc() & 0xFF));
    }
}

//-----------------------------------------------------------------------------
bool hm11_mbed_init(hm11* const dev, hm11_mbed* const mbed_dev)
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

    (void)hm11_mbed_timer();
    dev->platform_dev = mbed_dev;
    return hm11_init(dev);
}

//-----------------------------------------------------------------------------
bool hm11_mbed_attach_rx_irq(hm11* const dev)
{
    if (!dev || !dev->platform_dev)
    {
        return false;
    }

    const hm11_mbed* const pd = (const hm11_mbed*)dev->platform_dev;
    g_hm11_irq_dev = dev;
    pd->serial->attach(&hm11_mbed_rx_isr, RawSerial::RxIrq);
    return true;
}

//-----------------------------------------------------------------------------
bool hm11_uart_write(const hm11* const dev, const uint8_t* buf, size_t len)
{
    const hm11_mbed* const pd = (const hm11_mbed*)dev->platform_dev;

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
int hm11_uart_read_byte(const hm11* const dev)
{
    const hm11_mbed* const pd = (const hm11_mbed*)dev->platform_dev;

    if (!pd->serial->readable())
    {
        return -1;
    }
    return pd->serial->getc() & 0xFF;
}

//-----------------------------------------------------------------------------
bool hm11_uart_readable(const hm11* const dev)
{
    const hm11_mbed* const pd = (const hm11_mbed*)dev->platform_dev;

    return pd->serial->readable() != 0;
}

//-----------------------------------------------------------------------------
void hm11_delay_ms(uint32_t delay_ms)
{
    wait_us(delay_ms * 1000);
}

//-----------------------------------------------------------------------------
uint32_t hm11_get_ms_ticks(void)
{
    return (uint32_t)hm11_mbed_timer().read_ms();
}

//-----------------------------------------------------------------------------
