/**
 *  @brief:   MAX4070 bidirectional high-side current-sense amplifier library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "max4070-mbed.h"

//-----------------------------------------------------------------------------
bool max4070_mbed_init(max4070* const dev, max4070_mbed* const mbed_dev)
{
    if (!dev || !mbed_dev || !mbed_dev->adc)
    {
        return false;
    }

    dev->platform_dev = mbed_dev;
    return max4070_init(dev);
}

//-----------------------------------------------------------------------------
bool max4070_read_adc(const max4070* const dev, uint16_t* raw_adc_value)
{
    const max4070_mbed* const pd = (const max4070_mbed*)dev->platform_dev;

    *raw_adc_value = pd->adc->read_u16();
    return true;
}

//-----------------------------------------------------------------------------
