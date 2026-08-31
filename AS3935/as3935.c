/**
 *  @brief:   AS3935 Franklin lightning sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "as3935.h"

// Payload expected by the PRESET_DEFAULT and CALIB_RCO direct commands
#define AS3935_DIRECT_CMD_VALUE     0x96

// The AS3935 needs 2 ms after the IRQ line goes high before the interrupt
// source register holds a valid value, see the datasheet "Interrupt Management"
#define AS3935_IRQ_VALID_DELAY_MS   2

// Time needed by the device to reload the factory defaults
#define AS3935_RESET_DELAY_MS       2

// Time needed by the internal oscillators to settle after a calibration
#define AS3935_CALIB_DELAY_MS       3

// Time needed by the AFE to settle after leaving the power down state
#define AS3935_POWER_UP_DELAY_MS    3

//-----------------------------------------------------------------------------
static uint8_t as3935_mask_shift(uint8_t mask)
{
    uint8_t shift = 0;
    while (mask != 0 && (mask & 0x01u) == 0)
    {
        mask = (uint8_t)(mask >> 1);
        shift++;
    }
    return shift;
}

//-----------------------------------------------------------------------------
static bool as3935_read_field(const as3935* const dev, uint8_t reg_addr,
                              uint8_t mask, uint8_t* const value)
{
    uint8_t reg = 0;
    if (!as3935_read(dev, reg_addr, &reg, 1))
    {
        return false;
    }

    *value = (uint8_t)((reg & mask) >> as3935_mask_shift(mask));
    return true;
}

//-----------------------------------------------------------------------------
static bool as3935_write_field(const as3935* const dev, uint8_t reg_addr,
                               uint8_t mask, uint8_t value)
{
    uint8_t reg = 0;
    if (!as3935_read(dev, reg_addr, &reg, 1))
    {
        return false;
    }

    const uint8_t shifted = (uint8_t)((value << as3935_mask_shift(mask)) & mask);
    reg = (uint8_t)((reg & (uint8_t)~mask) | shifted);
    return as3935_write(dev, reg_addr, &reg, 1);
}

//-----------------------------------------------------------------------------
static bool as3935_direct_command(const as3935* const dev, uint8_t reg_addr)
{
    const uint8_t value = AS3935_DIRECT_CMD_VALUE;
    return as3935_write(dev, reg_addr, &value, 1);
}

//-----------------------------------------------------------------------------
bool as3935_init(as3935* const dev)
{
    if (!dev)
    {
        return false;
    }

    if (!as3935_reset(dev))
    {
        return false;
    }

    if (!as3935_set_tune_cap(dev, AS3935_DEFAULT_TUNE_CAP))
    {
        return false;
    }

    if (!as3935_power_up(dev))
    {
        return false;
    }

    const as3935_config cfg = {
        .afe_gain = AS3935_AFE_GAIN_INDOOR,
        .noise_floor = AS3935_NOISE_FLOOR_860_62,
        .min_lightnings = AS3935_MIN_LIGHTNINGS_1,
        .watchdog_threshold = 2,
        .spike_rejection = 2,
        .tune_cap = AS3935_DEFAULT_TUNE_CAP,
        .mask_disturbers = false,
    };

    return as3935_set_config(dev, &cfg);
}

//-----------------------------------------------------------------------------
bool as3935_set_config(const as3935* const dev, const as3935_config* const cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    return as3935_set_afe_gain(dev, cfg->afe_gain) &&
           as3935_set_noise_floor(dev, cfg->noise_floor) &&
           as3935_set_min_lightnings(dev, cfg->min_lightnings) &&
           as3935_set_watchdog_threshold(dev, cfg->watchdog_threshold) &&
           as3935_set_spike_rejection(dev, cfg->spike_rejection) &&
           as3935_set_tune_cap(dev, cfg->tune_cap) &&
           as3935_set_mask_disturbers(dev, cfg->mask_disturbers);
}

//-----------------------------------------------------------------------------
bool as3935_get_config(const as3935* const dev, as3935_config* const cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    return as3935_get_afe_gain(dev, &cfg->afe_gain) &&
           as3935_get_noise_floor(dev, &cfg->noise_floor) &&
           as3935_get_min_lightnings(dev, &cfg->min_lightnings) &&
           as3935_get_watchdog_threshold(dev, &cfg->watchdog_threshold) &&
           as3935_get_spike_rejection(dev, &cfg->spike_rejection) &&
           as3935_get_tune_cap(dev, &cfg->tune_cap) &&
           as3935_get_mask_disturbers(dev, &cfg->mask_disturbers);
}

//-----------------------------------------------------------------------------
bool as3935_reset(const as3935* const dev)
{
    if (!dev)
    {
        return false;
    }

    if (!as3935_direct_command(dev, AS3935_REG_PRESET_DEFAULT))
    {
        return false;
    }

    as3935_delay_ms(AS3935_RESET_DELAY_MS);
    return true;
}

//-----------------------------------------------------------------------------
bool as3935_calibrate_rco(const as3935* const dev)
{
    if (!dev)
    {
        return false;
    }

    if (!as3935_direct_command(dev, AS3935_REG_CALIB_RCO))
    {
        return false;
    }

    // The TRCO has to be routed to the IRQ pin for the calibration to complete
    if (!as3935_write_field(dev, AS3935_REG_TUNE, AS3935_MASK_DISP_TRCO, 1))
    {
        return false;
    }

    as3935_delay_ms(AS3935_CALIB_DELAY_MS);
    return as3935_write_field(dev, AS3935_REG_TUNE, AS3935_MASK_DISP_TRCO, 0);
}

//-----------------------------------------------------------------------------
bool as3935_power_up(const as3935* const dev)
{
    if (!dev)
    {
        return false;
    }

    if (!as3935_write_field(dev, AS3935_REG_AFE_GAIN, AS3935_MASK_PWD, 0))
    {
        return false;
    }

    as3935_delay_ms(AS3935_POWER_UP_DELAY_MS);
    return as3935_calibrate_rco(dev);
}

//-----------------------------------------------------------------------------
bool as3935_power_down(const as3935* const dev)
{
    if (!dev)
    {
        return false;
    }

    return as3935_write_field(dev, AS3935_REG_AFE_GAIN, AS3935_MASK_PWD, 1);
}

//-----------------------------------------------------------------------------
bool as3935_set_afe_gain(const as3935* const dev, as3935_afe_gain gain)
{
    if (!dev)
    {
        return false;
    }

    return as3935_write_field(dev, AS3935_REG_AFE_GAIN, AS3935_MASK_AFE_GB,
                              (uint8_t)gain);
}

//-----------------------------------------------------------------------------
bool as3935_get_afe_gain(const as3935* const dev, as3935_afe_gain* const gain)
{
    if (!dev || !gain)
    {
        return false;
    }

    uint8_t value = 0;
    if (!as3935_read_field(dev, AS3935_REG_AFE_GAIN, AS3935_MASK_AFE_GB, &value))
    {
        return false;
    }

    *gain = (as3935_afe_gain)value;
    return true;
}

//-----------------------------------------------------------------------------
bool as3935_set_noise_floor(const as3935* const dev, as3935_noise_floor level)
{
    if (!dev)
    {
        return false;
    }

    return as3935_write_field(dev, AS3935_REG_THRESHOLD, AS3935_MASK_NF_LEV,
                              (uint8_t)level);
}

//-----------------------------------------------------------------------------
bool as3935_get_noise_floor(const as3935* const dev,
                            as3935_noise_floor* const level)
{
    if (!dev || !level)
    {
        return false;
    }

    uint8_t value = 0;
    if (!as3935_read_field(dev, AS3935_REG_THRESHOLD, AS3935_MASK_NF_LEV, &value))
    {
        return false;
    }

    *level = (as3935_noise_floor)value;
    return true;
}

//-----------------------------------------------------------------------------
bool as3935_set_watchdog_threshold(const as3935* const dev, uint8_t wdth)
{
    if (!dev || wdth > AS3935_MASK_WDTH)
    {
        return false;
    }

    return as3935_write_field(dev, AS3935_REG_THRESHOLD, AS3935_MASK_WDTH, wdth);
}

//-----------------------------------------------------------------------------
bool as3935_get_watchdog_threshold(const as3935* const dev, uint8_t* const wdth)
{
    if (!dev || !wdth)
    {
        return false;
    }

    return as3935_read_field(dev, AS3935_REG_THRESHOLD, AS3935_MASK_WDTH, wdth);
}

//-----------------------------------------------------------------------------
bool as3935_set_spike_rejection(const as3935* const dev, uint8_t srej)
{
    if (!dev || srej > AS3935_MASK_SREJ)
    {
        return false;
    }

    return as3935_write_field(dev, AS3935_REG_LIGHTNING, AS3935_MASK_SREJ, srej);
}

//-----------------------------------------------------------------------------
bool as3935_get_spike_rejection(const as3935* const dev, uint8_t* const srej)
{
    if (!dev || !srej)
    {
        return false;
    }

    return as3935_read_field(dev, AS3935_REG_LIGHTNING, AS3935_MASK_SREJ, srej);
}

//-----------------------------------------------------------------------------
bool as3935_set_min_lightnings(const as3935* const dev,
                               as3935_min_lightnings min)
{
    if (!dev)
    {
        return false;
    }

    return as3935_write_field(dev, AS3935_REG_LIGHTNING, AS3935_MASK_MIN_LIGH,
                              (uint8_t)min);
}

//-----------------------------------------------------------------------------
bool as3935_get_min_lightnings(const as3935* const dev,
                               as3935_min_lightnings* const min)
{
    if (!dev || !min)
    {
        return false;
    }

    uint8_t value = 0;
    if (!as3935_read_field(dev, AS3935_REG_LIGHTNING, AS3935_MASK_MIN_LIGH,
                           &value))
    {
        return false;
    }

    *min = (as3935_min_lightnings)value;
    return true;
}

//-----------------------------------------------------------------------------
bool as3935_set_mask_disturbers(const as3935* const dev, bool mask)
{
    if (!dev)
    {
        return false;
    }

    return as3935_write_field(dev, AS3935_REG_INT, AS3935_MASK_MASK_DIST,
                              mask ? 1 : 0);
}

//-----------------------------------------------------------------------------
bool as3935_get_mask_disturbers(const as3935* const dev, bool* const mask)
{
    if (!dev || !mask)
    {
        return false;
    }

    uint8_t value = 0;
    if (!as3935_read_field(dev, AS3935_REG_INT, AS3935_MASK_MASK_DIST, &value))
    {
        return false;
    }

    *mask = (value != 0);
    return true;
}

//-----------------------------------------------------------------------------
bool as3935_set_tune_cap(const as3935* const dev, uint8_t cap)
{
    if (!dev || cap > AS3935_MASK_TUN_CAP)
    {
        return false;
    }

    return as3935_write_field(dev, AS3935_REG_TUNE, AS3935_MASK_TUN_CAP, cap);
}

//-----------------------------------------------------------------------------
bool as3935_get_tune_cap(const as3935* const dev, uint8_t* const cap)
{
    if (!dev || !cap)
    {
        return false;
    }

    return as3935_read_field(dev, AS3935_REG_TUNE, AS3935_MASK_TUN_CAP, cap);
}

//-----------------------------------------------------------------------------
bool as3935_set_lco_div_ratio(const as3935* const dev,
                              as3935_lco_div_ratio ratio)
{
    if (!dev)
    {
        return false;
    }

    return as3935_write_field(dev, AS3935_REG_INT, AS3935_MASK_LCO_FDIV,
                              (uint8_t)ratio);
}

//-----------------------------------------------------------------------------
bool as3935_display_lco(const as3935* const dev, bool enable)
{
    if (!dev)
    {
        return false;
    }

    return as3935_write_field(dev, AS3935_REG_TUNE, AS3935_MASK_DISP_LCO,
                              enable ? 1 : 0);
}

//-----------------------------------------------------------------------------
bool as3935_clear_statistics(const as3935* const dev)
{
    if (!dev)
    {
        return false;
    }

    // The statistics are cleared on a high-low-high transition of CL_STAT
    return as3935_write_field(dev, AS3935_REG_LIGHTNING, AS3935_MASK_CL_STAT, 1) &&
           as3935_write_field(dev, AS3935_REG_LIGHTNING, AS3935_MASK_CL_STAT, 0) &&
           as3935_write_field(dev, AS3935_REG_LIGHTNING, AS3935_MASK_CL_STAT, 1);
}

//-----------------------------------------------------------------------------
bool as3935_read_interrupt_source(const as3935* const dev,
                                  as3935_int_source* const src)
{
    if (!dev || !src)
    {
        return false;
    }

    as3935_delay_ms(AS3935_IRQ_VALID_DELAY_MS);

    uint8_t value = 0;
    if (!as3935_read_field(dev, AS3935_REG_INT, AS3935_MASK_INT, &value))
    {
        return false;
    }

    *src = (as3935_int_source)value;
    return true;
}

//-----------------------------------------------------------------------------
bool as3935_get_storm_distance(const as3935* const dev,
                               uint8_t* const distance_km)
{
    if (!dev || !distance_km)
    {
        return false;
    }

    return as3935_read_field(dev, AS3935_REG_DISTANCE, AS3935_MASK_DISTANCE,
                             distance_km);
}

//-----------------------------------------------------------------------------
bool as3935_get_energy(const as3935* const dev, uint32_t* const energy)
{
    if (!dev || !energy)
    {
        return false;
    }

    uint8_t raw[3] = {0};
    if (!as3935_read(dev, AS3935_REG_ENERGY_LSB, raw, sizeof(raw)))
    {
        return false;
    }

    *energy = ((uint32_t)(raw[2] & AS3935_MASK_ENERGY_MMSB) << 16) |
              ((uint32_t)raw[1] << 8) |
              (uint32_t)raw[0];
    return true;
}

//-----------------------------------------------------------------------------
