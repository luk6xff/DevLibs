// rf22-mbed.cpp
//
// MBED platform layer for the portable rf22 core.
//
// Derived from the RF22 library:
//   Copyright (C) 2011 Mike McCauley
//   ported to mbed by Karl Zweimueller
//   modified by Lukasz Uszko luszko@op.pl
//
// This software is Copyright (C) 2011 Mike McCauley. Use is subject to license
// conditions. The main licensing options available are GPL V2 or Commercial.
// See LICENSE.txt for the full statement. This file is NOT MIT licensed.
//
// Restructured for the DevLibs portable-core/platform layout by luk6xff, 2020.

#include "rf22-mbed.h"

// The RF22 wants mode 0, MSB first
#define RF22_MBED_DEFAULT_SPI_FREQ 1000000

// Free running millisecond source shared by every rf22 instance
static Timer rf22_mbed_timer;
static bool rf22_mbed_timer_started = false;

//-----------------------------------------------------------------------------
bool rf22_mbed_init(rf22* const dev, rf22_mbed* const mbed_dev)
{
    if (!dev || !mbed_dev || !mbed_dev->spi || !mbed_dev->cs || !mbed_dev->irq)
    {
        return false;
    }

    if (mbed_dev->spi_freq == 0)
    {
        mbed_dev->spi_freq = RF22_MBED_DEFAULT_SPI_FREQ;
    }

    // 8 bit frames, SPI mode 0
    mbed_dev->spi->format(8, 0);
    mbed_dev->spi->frequency((int)mbed_dev->spi_freq);

    *(mbed_dev->cs) = 1;
    if (mbed_dev->sdn)
    {
        *(mbed_dev->sdn) = 1; // Held in shutdown until init powers it up
    }

    if (!rf22_mbed_timer_started)
    {
        rf22_mbed_timer.start();
        rf22_mbed_timer_started = true;
    }

    dev->platform_dev = mbed_dev;
    return rf22_init(dev);
}

//-----------------------------------------------------------------------------
bool rf22_spi_transfer(const rf22* const dev, const uint8_t* tx, uint8_t* rx,
                       size_t len)
{
    const rf22_mbed* const pd = (const rf22_mbed*)dev->platform_dev;

    if (!pd || len == 0)
    {
        return false;
    }

    for (size_t i = 0; i < len; ++i)
    {
        const int out = tx ? (int)tx[i] : 0;
        const uint8_t in = (uint8_t)pd->spi->write(out);
        if (rx)
        {
            rx[i] = in;
        }
    }

    return true;
}

//-----------------------------------------------------------------------------
void rf22_set_cs_pin(const rf22* const dev, bool state)
{
    const rf22_mbed* const pd = (const rf22_mbed*)dev->platform_dev;

    *(pd->cs) = state ? 1 : 0;
}

//-----------------------------------------------------------------------------
void rf22_set_sdn_pin(const rf22* const dev, bool state)
{
    const rf22_mbed* const pd = (const rf22_mbed*)dev->platform_dev;

    if (!pd->sdn)
    {
        return; // SDN tied low in hardware
    }
    *(pd->sdn) = state ? 1 : 0;
}

//-----------------------------------------------------------------------------
bool rf22_get_irq_pin(const rf22* const dev)
{
    const rf22_mbed* const pd = (const rf22_mbed*)dev->platform_dev;

    return pd->irq->read() != 0;
}

//-----------------------------------------------------------------------------
void rf22_delay_ms(uint32_t delay_ms)
{
    wait_us(delay_ms * 1000);
}

//-----------------------------------------------------------------------------
uint32_t rf22_get_ms_ticks(void)
{
    if (!rf22_mbed_timer_started)
    {
        rf22_mbed_timer.start();
        rf22_mbed_timer_started = true;
    }

    return (uint32_t)rf22_mbed_timer.read_ms();
}

//-----------------------------------------------------------------------------
