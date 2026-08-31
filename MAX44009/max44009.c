/**
 *  @brief:   MAX44009 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "max44009.h"

// CONFIG register bit masks
#define MAX44009_CFG_CONT_BIT   (1u << 7)
#define MAX44009_CFG_MANUAL_BIT (1u << 6)
#define MAX44009_CFG_CDR_BIT    (1u << 3)
#define MAX44009_CFG_TIM_MASK   0x07u

// LSB of the lux register in [lx], see the datasheet "Lux Reading" section
#define MAX44009_LUX_LSB        0.045f

// Exponent value reported when the measurement is out of range
#define MAX44009_EXP_OVERRANGE  0x0Fu

//-----------------------------------------------------------------------------
bool max44009_init(max44009* const dev)
{
    if (!dev)
    {
        return false;
    }

    const max44009_config cfg = {
        .continuous_mode = false,
        .manual_mode = false,
        .current_div_ratio = false,
        .integration_time = MAX44009_INTEGRATION_TIME_800MS,
    };

    if (!max44009_set_config(dev, &cfg))
    {
        return false;
    }

    // Give the sensor time to complete the first conversion
    max44009_delay_ms(1000);
    return true;
}

//-----------------------------------------------------------------------------
bool max44009_set_config(const max44009* const dev,
                         const max44009_config* const cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    uint8_t reg = 0;
    if (cfg->continuous_mode)
    {
        reg |= MAX44009_CFG_CONT_BIT;
    }
    if (cfg->manual_mode)
    {
        reg |= MAX44009_CFG_MANUAL_BIT;
        if (cfg->current_div_ratio)
        {
            reg |= MAX44009_CFG_CDR_BIT;
        }
        reg |= ((uint8_t)cfg->integration_time & MAX44009_CFG_TIM_MASK);
    }

    return max44009_write(dev, MAX44009_REG_CONFIG, &reg, 1);
}

//-----------------------------------------------------------------------------
bool max44009_get_config(const max44009* const dev, max44009_config* const cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    uint8_t reg = 0;
    if (!max44009_read(dev, MAX44009_REG_CONFIG, &reg, 1))
    {
        return false;
    }

    cfg->continuous_mode = (reg & MAX44009_CFG_CONT_BIT) != 0;
    cfg->manual_mode = (reg & MAX44009_CFG_MANUAL_BIT) != 0;
    cfg->current_div_ratio = (reg & MAX44009_CFG_CDR_BIT) != 0;
    cfg->integration_time =
        (max44009_integration_time)(reg & MAX44009_CFG_TIM_MASK);
    return true;
}

//-----------------------------------------------------------------------------
bool max44009_get_lux(const max44009* const dev, float* lux)
{
    if (!dev || !lux)
    {
        return false;
    }

    // The high byte latches the low byte, so both must be read in one burst
    uint8_t raw[2] = {0};
    if (!max44009_read(dev, MAX44009_REG_LUX_HIGH, raw, sizeof(raw)))
    {
        *lux = -1.0f;
        return false;
    }

    const uint8_t exponent = (raw[0] & 0xF0u) >> 4;
    if (exponent == MAX44009_EXP_OVERRANGE)
    {
        *lux = -1.0f;
        return false;
    }

    const uint32_t mantissa = ((uint32_t)(raw[0] & 0x0Fu) << 4) |
                              (uint32_t)(raw[1] & 0x0Fu);
    *lux = (float)(mantissa << exponent) * MAX44009_LUX_LSB;
    return true;
}

//-----------------------------------------------------------------------------
