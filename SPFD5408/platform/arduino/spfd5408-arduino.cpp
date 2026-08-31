/**
 *  @brief:   SPFD5408 240x320 TFT LCD controller library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "spfd5408-arduino.h"

//-----------------------------------------------------------------------------
bool spfd5408_arduino_init(spfd5408* const dev,
                           spfd5408_arduino* const arduino_dev)
{
    if (!dev || !arduino_dev)
    {
        return false;
    }

    for (int i = 0; i < SPFD5408_ARDUINO_DATA_PINS; i++)
    {
        pinMode(arduino_dev->data_pins[i], OUTPUT);
        digitalWrite(arduino_dev->data_pins[i], LOW);
    }

    pinMode(arduino_dev->rs_pin, OUTPUT);
    pinMode(arduino_dev->wr_pin, OUTPUT);
    pinMode(arduino_dev->rd_pin, OUTPUT);
    pinMode(arduino_dev->cs_pin, OUTPUT);
    pinMode(arduino_dev->reset_pin, OUTPUT);

    digitalWrite(arduino_dev->rs_pin, HIGH);
    digitalWrite(arduino_dev->wr_pin, HIGH);
    digitalWrite(arduino_dev->rd_pin, HIGH);
    digitalWrite(arduino_dev->cs_pin, HIGH);
    digitalWrite(arduino_dev->reset_pin, HIGH);

    dev->platform_dev = arduino_dev;
    return spfd5408_init(dev);
}

//-----------------------------------------------------------------------------
void spfd5408_write_bus(const spfd5408* const dev, uint16_t value)
{
    const spfd5408_arduino* const pd = (const spfd5408_arduino*)dev->platform_dev;

    for (int i = 0; i < SPFD5408_ARDUINO_DATA_PINS; i++)
    {
        digitalWrite(pd->data_pins[i],
                     ((value >> i) & 0x0001u) ? HIGH : LOW);
    }
}

//-----------------------------------------------------------------------------
void spfd5408_set_pin(const spfd5408* const dev, spfd5408_pin pin, bool state)
{
    const spfd5408_arduino* const pd = (const spfd5408_arduino*)dev->platform_dev;

    int gpio;
    switch (pin)
    {
        case SPFD5408_PIN_RS:
            gpio = pd->rs_pin;
            break;
        case SPFD5408_PIN_WR:
            gpio = pd->wr_pin;
            break;
        case SPFD5408_PIN_RD:
            gpio = pd->rd_pin;
            break;
        case SPFD5408_PIN_RESET:
            gpio = pd->reset_pin;
            break;
        case SPFD5408_PIN_CS:
        default:
            gpio = pd->cs_pin;
            break;
    }
    digitalWrite(gpio, state ? HIGH : LOW);
}

//-----------------------------------------------------------------------------
void spfd5408_delay_ms(uint32_t delay_ms)
{
    delay(delay_ms);
}

//-----------------------------------------------------------------------------
