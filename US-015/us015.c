/**
 *  @brief:   US-015 (HC-SR04 compatible) ultrasonic distance sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "us015.h"

//-----------------------------------------------------------------------------
static uint32_t us015_timeout(const us015* const dev)
{
    return (dev->timeout_us != 0) ? dev->timeout_us : US015_DEFAULT_TIMEOUT_US;
}

//-----------------------------------------------------------------------------
bool us015_init(us015* const dev)
{
    if (!dev)
    {
        return false;
    }

    us015_set_trigger_pin(dev, false);
    us015_delay_us(US015_TRIGGER_SETTLE_US);
    return true;
}

//-----------------------------------------------------------------------------
bool us015_read_pulse_width(const us015* const dev, uint32_t* pulse_width_us)
{
    if (!dev || !pulse_width_us)
    {
        return false;
    }

    const uint32_t timeout = us015_timeout(dev);

    // A short HIGH pulse makes the sensor emit a burst of eight 40 kHz cycles
    us015_set_trigger_pin(dev, false);
    us015_delay_us(US015_TRIGGER_SETTLE_US);
    us015_set_trigger_pin(dev, true);
    us015_delay_us(US015_TRIGGER_PULSE_US);
    us015_set_trigger_pin(dev, false);

    // Wait for the rising edge of the echo pulse
    const uint32_t wait_start = us015_get_us_ticks();
    while (!us015_get_echo_pin(dev))
    {
        if ((us015_get_us_ticks() - wait_start) > timeout)
        {
            return false;
        }
    }

    // Wait for the falling edge, the pulse width encodes the distance
    const uint32_t echo_start = us015_get_us_ticks();
    while (us015_get_echo_pin(dev))
    {
        if ((us015_get_us_ticks() - echo_start) > timeout)
        {
            return false;
        }
    }

    *pulse_width_us = us015_get_us_ticks() - echo_start;
    return true;
}

//-----------------------------------------------------------------------------
bool us015_read_distance_cm(const us015* const dev, float* distance_cm)
{
    if (!dev || !distance_cm)
    {
        return false;
    }

    uint32_t pulse_width_us = 0;
    if (!us015_read_pulse_width(dev, &pulse_width_us))
    {
        return false;
    }

    *distance_cm = us015_pulse_width_to_cm(pulse_width_us);
    return true;
}

//-----------------------------------------------------------------------------
bool us015_read_distance_inch(const us015* const dev, float* distance_inch)
{
    if (!dev || !distance_inch)
    {
        return false;
    }

    uint32_t pulse_width_us = 0;
    if (!us015_read_pulse_width(dev, &pulse_width_us))
    {
        return false;
    }

    *distance_inch = us015_pulse_width_to_inch(pulse_width_us);
    return true;
}

//-----------------------------------------------------------------------------
float us015_pulse_width_to_cm(uint32_t pulse_width_us)
{
    return (float)pulse_width_us / US015_US_PER_CM;
}

//-----------------------------------------------------------------------------
float us015_pulse_width_to_inch(uint32_t pulse_width_us)
{
    return (float)pulse_width_us / US015_US_PER_INCH;
}

//-----------------------------------------------------------------------------
uint32_t us015_distance_cm_to_timeout_us(uint32_t distance_cm)
{
    return (uint32_t)((float)distance_cm * US015_US_PER_CM);
}

//-----------------------------------------------------------------------------
