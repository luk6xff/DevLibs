/**
 *  @brief:   MAX4070 bidirectional high-side current-sense amplifier library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "max4070.h"

//-----------------------------------------------------------------------------
static bool max4070_config_valid(const max4070* const dev)
{
    return dev->adc_vref > 0.0f &&
           dev->adc_max_value != 0 &&
           dev->adc_samples_num != 0 &&
           dev->gain > 0.0f &&
           dev->shunt_resistance > 0.0f;
}

//-----------------------------------------------------------------------------
bool max4070_init(max4070* const dev)
{
    if (!dev)
    {
        return false;
    }

    dev->out_voltage = MAX4070_INVALID_VOLTAGE;
    dev->current = MAX4070_INVALID_CURRENT;
    return max4070_config_valid(dev);
}

//-----------------------------------------------------------------------------
bool max4070_read_output_voltage(max4070* const dev, float* voltage)
{
    if (!dev || !max4070_config_valid(dev))
    {
        return false;
    }

    // The product of two 16-bit values times the sample count still fits here
    uint32_t sum = 0;
    for (uint16_t i = 0; i < dev->adc_samples_num; ++i)
    {
        uint16_t raw = 0;
        if (!max4070_read_adc(dev, &raw))
        {
            dev->out_voltage = MAX4070_INVALID_VOLTAGE;
            return false;
        }
        sum += raw;
    }

    const float average = (float)sum / (float)dev->adc_samples_num;
    dev->out_voltage = (average / (float)dev->adc_max_value) * dev->adc_vref;
    if (voltage)
    {
        *voltage = dev->out_voltage;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool max4070_read_current(max4070* const dev, float* current)
{
    if (!dev)
    {
        return false;
    }

    float out_voltage = 0.0f;
    if (!max4070_read_output_voltage(dev, &out_voltage))
    {
        dev->current = MAX4070_INVALID_CURRENT;
        return false;
    }

    // Bidirectional sensing: the output swings around the reference level
    dev->current = (out_voltage - dev->ref_voltage) /
                   (dev->gain * dev->shunt_resistance);
    if (current)
    {
        *current = dev->current;
    }
    return true;
}

//-----------------------------------------------------------------------------
float max4070_get_output_voltage(const max4070* const dev)
{
    if (!dev)
    {
        return MAX4070_INVALID_VOLTAGE;
    }
    return dev->out_voltage;
}

//-----------------------------------------------------------------------------
float max4070_get_current(const max4070* const dev)
{
    if (!dev)
    {
        return MAX4070_INVALID_CURRENT;
    }
    return dev->current;
}

//-----------------------------------------------------------------------------
float max4070_scale_divider_voltage(float measured_voltage, float r_top_ohms,
                                    float r_bottom_ohms)
{
    if (r_top_ohms < 0.0f || r_bottom_ohms <= 0.0f)
    {
        return MAX4070_INVALID_VOLTAGE;
    }
    return measured_voltage * ((r_top_ohms + r_bottom_ohms) / r_bottom_ohms);
}

//-----------------------------------------------------------------------------
