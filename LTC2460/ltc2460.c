/**
 *  @brief:   LTC2460 ultra-tiny 16-bit delta-sigma ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "ltc2460.h"

//-----------------------------------------------------------------------------
bool ltc2460_init(ltc2460* const dev)
{
    if (!dev || dev->vref <= 0.0f || dev->divider_ratio < 0.0f)
    {
        return false;
    }

    // Leave the device deselected before the first transfer
    ltc2460_set_cs_pin(dev, true);

    if (!ltc2460_set_speed(dev, LTC2460_SPEED_1X))
    {
        return false;
    }

    // The first result is only valid once a full conversion has completed
    ltc2460_delay_ms(LTC2460_CONVERSION_TIME_MS);
    return true;
}

//-----------------------------------------------------------------------------
bool ltc2460_transfer(const ltc2460* const dev, uint16_t cfg, uint16_t* raw)
{
    if (!dev)
    {
        return false;
    }

    const uint8_t tx[2] = {(uint8_t)(cfg >> 8), (uint8_t)(cfg & 0x00FFu)};
    uint8_t rx[2] = {0, 0};

    ltc2460_set_cs_pin(dev, false);
    const bool ok = ltc2460_spi_transfer(dev, tx, rx, sizeof(tx));
    ltc2460_set_cs_pin(dev, true);

    if (!ok)
    {
        return false;
    }

    if (raw)
    {
        *raw = (uint16_t)(((uint16_t)rx[0] << 8) | (uint16_t)rx[1]);
    }
    return true;
}

//-----------------------------------------------------------------------------
bool ltc2460_read_raw(const ltc2460* const dev, uint16_t* raw)
{
    if (!dev || !raw)
    {
        return false;
    }

    // EN cleared, so the current SLP/SPD settings are kept untouched
    return ltc2460_transfer(dev, 0x0000u, raw);
}

//-----------------------------------------------------------------------------
bool ltc2460_read_voltage(const ltc2460* const dev, float* voltage)
{
    if (!dev || !voltage)
    {
        return false;
    }

    uint16_t raw = 0;
    if (!ltc2460_read_raw(dev, &raw))
    {
        return false;
    }

    *voltage = ltc2460_raw_to_voltage(dev, raw);
    return true;
}

//-----------------------------------------------------------------------------
bool ltc2460_set_speed(const ltc2460* const dev, ltc2460_speed speed)
{
    if (!dev)
    {
        return false;
    }

    uint16_t cfg = LTC2460_CFG_EN;
    if (speed == LTC2460_SPEED_2X)
    {
        cfg |= LTC2460_CFG_SPD;
    }
    return ltc2460_transfer(dev, cfg, NULL);
}

//-----------------------------------------------------------------------------
bool ltc2460_set_sleep(const ltc2460* const dev, bool enable)
{
    if (!dev)
    {
        return false;
    }

    uint16_t cfg = LTC2460_CFG_EN;
    if (enable)
    {
        cfg |= LTC2460_CFG_SLP;
    }
    return ltc2460_transfer(dev, cfg, NULL);
}

//-----------------------------------------------------------------------------
float ltc2460_raw_to_voltage(const ltc2460* const dev, uint16_t raw)
{
    if (!dev)
    {
        return 0.0f;
    }

    const float adc_voltage = (dev->vref / LTC2460_FULL_SCALE) * (float)raw;
    return adc_voltage * (dev->divider_ratio + 1.0f);
}

//-----------------------------------------------------------------------------
