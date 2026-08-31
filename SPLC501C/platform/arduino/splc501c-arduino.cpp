/**
 *  @brief:   SPLC501C 128x64 monochrome graphic LCD library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "splc501c-arduino.h"

#define SPLC501C_ARDUINO_DEFAULT_SPI_FREQ 4000000UL

//-----------------------------------------------------------------------------
bool splc501c_arduino_init(splc501c* const dev,
                           splc501c_arduino* const arduino_dev)
{
    if (!dev || !arduino_dev || !arduino_dev->spi)
    {
        return false;
    }

    if (arduino_dev->spi_freq == 0)
    {
        arduino_dev->spi_freq = SPLC501C_ARDUINO_DEFAULT_SPI_FREQ;
    }

    pinMode(arduino_dev->cs_pin, OUTPUT);
    pinMode(arduino_dev->a0_pin, OUTPUT);
    pinMode(arduino_dev->reset_pin, OUTPUT);
    digitalWrite(arduino_dev->cs_pin, HIGH);
    digitalWrite(arduino_dev->a0_pin, LOW);
    digitalWrite(arduino_dev->reset_pin, HIGH);

    arduino_dev->spi->begin();

    dev->platform_dev = arduino_dev;
    return splc501c_init(dev);
}

//-----------------------------------------------------------------------------
bool splc501c_write_bytes(const splc501c* const dev, const uint8_t* buf,
                          size_t len)
{
    const splc501c_arduino* const pd = (const splc501c_arduino*)dev->platform_dev;

    if (!buf || len == 0)
    {
        return false;
    }

    pd->spi->beginTransaction(SPISettings(pd->spi_freq, MSBFIRST, SPI_MODE0));
    for (size_t i = 0; i < len; i++)
    {
        // transfer(buf, len) would write back into the caller's const buffer
        (void)pd->spi->transfer(buf[i]);
    }
    pd->spi->endTransaction();
    return true;
}

//-----------------------------------------------------------------------------
void splc501c_set_pin(const splc501c* const dev, splc501c_pin pin, bool state)
{
    const splc501c_arduino* const pd = (const splc501c_arduino*)dev->platform_dev;

    int gpio;
    switch (pin)
    {
        case SPLC501C_PIN_A0:
            gpio = pd->a0_pin;
            break;
        case SPLC501C_PIN_RESET:
            gpio = pd->reset_pin;
            break;
        case SPLC501C_PIN_CS:
        default:
            gpio = pd->cs_pin;
            break;
    }
    digitalWrite(gpio, state ? HIGH : LOW);
}

//-----------------------------------------------------------------------------
void splc501c_delay_ms(uint32_t delay_ms)
{
    delay(delay_ms);
}

//-----------------------------------------------------------------------------
