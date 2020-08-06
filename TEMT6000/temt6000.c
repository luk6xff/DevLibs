/**
 *  @brief:   TEMT6000 - TEMT6000 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-08-06
 *  @license: MIT
 */

#include "temt6000.h"
#include <string.h> // memset

//------------------------------------------------------------------------------
// @brief DEFINES FUNCTIONS
//------------------------------------------------------------------------------
/**
 * @brief Resistor value solderd on TEMT6000 sparkfun breakout board:
 *        https://learn.sparkfun.com/tutorials/temt6000-ambient-light-sensor-hookup-guide
 *        https://esphome.io/cookbook/temt6000.html
 */
#define TEMT6000_RESISTOR_OHMS  10000 //[Ohm]
//------------------------------------------------------------------------------
// @brief PRIVATE FUNCTIONS
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/**
 * @brief Computes luminance value from raw adc result.
 *        1 lux[lx] = 2[uA] measured on the resistor
 *
 * @param[in] raw_adc_result
 * @param[in] adc_max_val - resolution of ADC
 * @param[in] vdd_val - sensor power supply voltage
 * @param[in] raw_adc_result
 *
 * @return computed light illuminance in lux unit [lx]
 */
static float compute_result_formula(const uint64_t raw_adc_result, const uint16_t adc_max_val, const float vdd_val)
{
    return ((((float)raw_adc_result/ (float)adc_max_val) * (float)vdd_val) / TEMT6000_RESISTOR_OHMS) * 2000000.0f;
}



//------------------------------------------------------------------------------
// @brief FUNCTIONS DEFINITIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void temt6000_init(temt6000* const dev)
{
    // EMPTY
}

//------------------------------------------------------------------------------
float temt6000_get_light(temt6000* const dev)
{
    uint64_t average_adc = 0;
    uint16_t samples_buffer[dev->adc_samples_num];
    memset(samples_buffer, 0, sizeof(samples_buffer));
    // Read all the samples
    for(size_t i = 0; i < dev->adc_samples_num; i++)
    {
        uint16_t raw_val;
        if (temt6000_read_raw(dev, &raw_val))
        {
            samples_buffer[i] = raw_val;
        }
    }
    // Compute average
    for(size_t i = 0; i < dev->adc_samples_num; i++)
    {
        average_adc += samples_buffer[i];
    }
    average_adc /= dev->adc_samples_num;

    return compute_result_formula(average_adc, dev->adc_max_value, dev->vdd_value);
}

//------------------------------------------------------------------------------
