/**
 *  @brief:   AD9106 quad, low power, 12-bit, 180 MSPS DAC and waveform generator library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "ad9106-arduino.h"

// Fallback clock used when the descriptor does not define one
#define AD9106_ARDUINO_DEFAULT_SPI_FREQ 1000000

//-----------------------------------------------------------------------------
static int ad9106_arduino_pin_num(const ad9106_arduino* const pd, ad9106_pin pin)
{
    switch (pin)
    {
        case AD9106_PIN_CS:
            return pd->cs_pin;
        case AD9106_PIN_RESET:
            return pd->reset_pin;
        case AD9106_PIN_TRIGGER:
            return pd->trigger_pin;
        default:
            return -1;
    }
}

//-----------------------------------------------------------------------------
bool ad9106_arduino_init(ad9106* const dev, ad9106_arduino* const arduino_dev)
{
    if (!dev || !arduino_dev || !arduino_dev->spi)
    {
        return false;
    }

    if (arduino_dev->spi_freq == 0)
    {
        arduino_dev->spi_freq = AD9106_ARDUINO_DEFAULT_SPI_FREQ;
    }

    pinMode(arduino_dev->cs_pin, OUTPUT);
    digitalWrite(arduino_dev->cs_pin, HIGH);
    if (arduino_dev->reset_pin >= 0)
    {
        pinMode(arduino_dev->reset_pin, OUTPUT);
        digitalWrite(arduino_dev->reset_pin, HIGH);
    }
    if (arduino_dev->trigger_pin >= 0)
    {
        pinMode(arduino_dev->trigger_pin, OUTPUT);
        digitalWrite(arduino_dev->trigger_pin, HIGH);
    }

    // The AD9106 SPI port runs in mode 0 (CPOL=0, CPHA=0), MSB first
    arduino_dev->spi->begin();

    dev->platform_dev = arduino_dev;
    return ad9106_init(dev);
}

//-----------------------------------------------------------------------------
bool ad9106_spi_transfer(const ad9106* const dev, const uint8_t* tx,
                         uint8_t* rx, size_t len)
{
    const ad9106_arduino* const pd = (const ad9106_arduino*)dev->platform_dev;

    if (!pd || !tx || !rx)
    {
        return false;
    }

    // SPI mode 0 (CPOL=0, CPHA=0) as required by the AD9106
    pd->spi->beginTransaction(SPISettings(pd->spi_freq, MSBFIRST, SPI_MODE0));
    for (size_t i = 0; i < len; ++i)
    {
        rx[i] = pd->spi->transfer(tx[i]);
    }
    pd->spi->endTransaction();
    return true;
}

//-----------------------------------------------------------------------------
void ad9106_set_pin(const ad9106* const dev, ad9106_pin pin, bool state)
{
    const ad9106_arduino* const pd = (const ad9106_arduino*)dev->platform_dev;

    if (!pd)
    {
        return;
    }

    const int pin_num = ad9106_arduino_pin_num(pd, pin);
    if (pin_num < 0)
    {
        // Line not wired on this board
        return;
    }
    digitalWrite(pin_num, state ? HIGH : LOW);
}

//-----------------------------------------------------------------------------
void ad9106_delay_ms(uint32_t delay_ms)
{
    delay(delay_ms);
}

//-----------------------------------------------------------------------------
