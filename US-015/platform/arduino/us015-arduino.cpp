/**
 *  @brief:   US-015 (HC-SR04 compatible) ultrasonic distance sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "us015-arduino.h"

//-----------------------------------------------------------------------------
bool us015_arduino_init(us015* const dev, us015_arduino* const arduino_dev)
{
    if (!dev || !arduino_dev)
    {
        return false;
    }

    pinMode(arduino_dev->trigger_pin, OUTPUT);
    pinMode(arduino_dev->echo_pin, INPUT);
    digitalWrite(arduino_dev->trigger_pin, LOW);

    dev->platform_dev = arduino_dev;
    return us015_init(dev);
}

//-----------------------------------------------------------------------------
void us015_set_trigger_pin(const us015* const dev, bool state)
{
    const us015_arduino* const pd = (const us015_arduino*)dev->platform_dev;

    digitalWrite(pd->trigger_pin, state ? HIGH : LOW);
}

//-----------------------------------------------------------------------------
bool us015_get_echo_pin(const us015* const dev)
{
    const us015_arduino* const pd = (const us015_arduino*)dev->platform_dev;

    return digitalRead(pd->echo_pin) == HIGH;
}

//-----------------------------------------------------------------------------
void us015_delay_us(uint32_t delay_us)
{
    delayMicroseconds(delay_us);
}

//-----------------------------------------------------------------------------
uint32_t us015_get_us_ticks(void)
{
    return micros();
}

//-----------------------------------------------------------------------------
