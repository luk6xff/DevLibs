/**
 *  @brief:   MAX4070 bidirectional high-side current-sense amplifier library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "max4070-cube-hal.h"

#define MAX4070_CUBE_DEFAULT_TIMEOUT_MS 100

//-----------------------------------------------------------------------------
bool max4070_cube_hal_init(max4070* const dev,
                           max4070_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->adc)
    {
        return false;
    }

    if (cube_hal_dev->timeout_ms == 0)
    {
        cube_hal_dev->timeout_ms = MAX4070_CUBE_DEFAULT_TIMEOUT_MS;
    }
    dev->platform_dev = cube_hal_dev;
    return max4070_init(dev);
}

//-----------------------------------------------------------------------------
bool max4070_read_adc(const max4070* const dev, uint16_t* raw_adc_value)
{
    const max4070_cube_hal* const pd =
        (const max4070_cube_hal*)dev->platform_dev;

    if (HAL_ADC_Start(pd->adc) != HAL_OK)
    {
        return false;
    }

    if (HAL_ADC_PollForConversion(pd->adc, pd->timeout_ms) != HAL_OK)
    {
        HAL_ADC_Stop(pd->adc);
        return false;
    }

    *raw_adc_value = (uint16_t)HAL_ADC_GetValue(pd->adc);
    return HAL_ADC_Stop(pd->adc) == HAL_OK;
}

//-----------------------------------------------------------------------------
