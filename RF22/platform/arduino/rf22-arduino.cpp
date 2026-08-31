// rf22-arduino.cpp
//
// ARDUINO platform layer for the portable rf22 core.
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

#include "rf22-arduino.h"

// The RF22 wants mode 0, MSB first. 1 MHz is safe on every Arduino.
#define RF22_ARDUINO_DEFAULT_SPI_FREQ 1000000

//-----------------------------------------------------------------------------
bool rf22_arduino_init(rf22* const dev, rf22_arduino* const arduino_dev)
{
    if (!dev || !arduino_dev || !arduino_dev->spi)
    {
        return false;
    }

    if (arduino_dev->spi_freq == 0)
    {
        arduino_dev->spi_freq = RF22_ARDUINO_DEFAULT_SPI_FREQ;
    }

    pinMode(arduino_dev->cs_pin, OUTPUT);
    digitalWrite(arduino_dev->cs_pin, HIGH);
    pinMode(arduino_dev->irq_pin, INPUT_PULLUP);

    if (arduino_dev->sdn_pin != RF22_ARDUINO_PIN_UNUSED)
    {
        pinMode(arduino_dev->sdn_pin, OUTPUT);
        digitalWrite(arduino_dev->sdn_pin, HIGH); // Held in shutdown until init
    }

    arduino_dev->spi->begin();

    dev->platform_dev = arduino_dev;
    return rf22_init(dev);
}

//-----------------------------------------------------------------------------
bool rf22_spi_transfer(const rf22* const dev, const uint8_t* tx, uint8_t* rx,
                       size_t len)
{
    const rf22_arduino* const pd = (const rf22_arduino*)dev->platform_dev;

    if (!pd || len == 0)
    {
        return false;
    }

    pd->spi->beginTransaction(SPISettings(pd->spi_freq, MSBFIRST, SPI_MODE0));
    for (size_t i = 0; i < len; ++i)
    {
        const uint8_t out = tx ? tx[i] : (uint8_t)0;
        const uint8_t in = pd->spi->transfer(out);
        if (rx)
        {
            rx[i] = in;
        }
    }
    pd->spi->endTransaction();

    return true;
}

//-----------------------------------------------------------------------------
void rf22_set_cs_pin(const rf22* const dev, bool state)
{
    const rf22_arduino* const pd = (const rf22_arduino*)dev->platform_dev;

    digitalWrite(pd->cs_pin, state ? HIGH : LOW);
}

//-----------------------------------------------------------------------------
void rf22_set_sdn_pin(const rf22* const dev, bool state)
{
    const rf22_arduino* const pd = (const rf22_arduino*)dev->platform_dev;

    if (pd->sdn_pin == RF22_ARDUINO_PIN_UNUSED)
    {
        return; // SDN tied low in hardware
    }
    digitalWrite(pd->sdn_pin, state ? HIGH : LOW);
}

//-----------------------------------------------------------------------------
bool rf22_get_irq_pin(const rf22* const dev)
{
    const rf22_arduino* const pd = (const rf22_arduino*)dev->platform_dev;

    return digitalRead(pd->irq_pin) != LOW;
}

//-----------------------------------------------------------------------------
void rf22_delay_ms(uint32_t delay_ms)
{
    delay(delay_ms);
}

//-----------------------------------------------------------------------------
uint32_t rf22_get_ms_ticks(void)
{
    return (uint32_t)millis();
}

//-----------------------------------------------------------------------------
