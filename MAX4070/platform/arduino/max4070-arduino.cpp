/**
 *  @brief:   MAX4070 bidirectional high-side current-sense amplifier library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "max4070-arduino.h"

//-----------------------------------------------------------------------------
bool max4070_arduino_init(max4070* const dev,
                          max4070_arduino* const arduino_dev)
{
    if (!dev || !arduino_dev)
    {
        return false;
    }

    pinMode(arduino_dev->adc_pin, INPUT);
    dev->platform_dev = arduino_dev;
    return max4070_init(dev);
}

//-----------------------------------------------------------------------------
bool max4070_read_adc(const max4070* const dev, uint16_t* raw_adc_value)
{
    const max4070_arduino* const pd = (const max4070_arduino*)dev->platform_dev;

    const int raw = analogRead(pd->adc_pin);
    if (raw < 0)
    {
        return false;
    }
    *raw_adc_value = (uint16_t)raw;
    return true;
}

//-----------------------------------------------------------------------------
