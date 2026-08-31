/**
 *  @brief:   LTC2460 ultra-tiny 16-bit delta-sigma ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "ltc2460-arduino.h"

// Fallback clock used when the descriptor does not define one
#define LTC2460_ARDUINO_DEFAULT_SPI_FREQ 1000000

//-----------------------------------------------------------------------------
bool ltc2460_arduino_init(ltc2460* const dev,
                          ltc2460_arduino* const arduino_dev)
{
    if (!dev || !arduino_dev || !arduino_dev->spi)
    {
        return false;
    }

    if (arduino_dev->spi_freq == 0)
    {
        arduino_dev->spi_freq = LTC2460_ARDUINO_DEFAULT_SPI_FREQ;
    }

    pinMode(arduino_dev->cs_pin, OUTPUT);
    digitalWrite(arduino_dev->cs_pin, HIGH);
    arduino_dev->spi->begin();

    dev->platform_dev = arduino_dev;
    return ltc2460_init(dev);
}

//-----------------------------------------------------------------------------
bool ltc2460_spi_transfer(const ltc2460* const dev, const uint8_t* tx,
                          uint8_t* rx, size_t len)
{
    const ltc2460_arduino* const pd = (const ltc2460_arduino*)dev->platform_dev;

    if (!tx || !rx)
    {
        return false;
    }

    pd->spi->beginTransaction(SPISettings(pd->spi_freq, MSBFIRST, SPI_MODE0));
    for (size_t i = 0; i < len; ++i)
    {
        rx[i] = pd->spi->transfer(tx[i]);
    }
    pd->spi->endTransaction();
    return true;
}

//-----------------------------------------------------------------------------
void ltc2460_set_cs_pin(const ltc2460* const dev, bool state)
{
    const ltc2460_arduino* const pd = (const ltc2460_arduino*)dev->platform_dev;

    digitalWrite(pd->cs_pin, state ? HIGH : LOW);
}

//-----------------------------------------------------------------------------
void ltc2460_delay_ms(uint32_t delay_ms)
{
    delay(delay_ms);
}

//-----------------------------------------------------------------------------
