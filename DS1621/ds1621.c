/**
 *  @brief:   DS1621 digital thermometer and thermostat library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "ds1621.h"

// Typical EEPROM write cycle time, the datasheet guarantees 10 ms
#define DS1621_EEPROM_WRITE_TIME_MS 10

// Number of NVB polls performed before an EEPROM write is declared failed
#define DS1621_EEPROM_POLL_RETRIES  10

// Granularity of the conversion completion polling
#define DS1621_POLL_STEP_MS         50

//-----------------------------------------------------------------------------
/**
 * @brief Decode the 9-bit two's complement temperature format.
 *
 * The MSB carries whole degrees, bit 7 of the LSB carries the 0.5 degC step,
 * which makes the raw 16-bit word a plain fixed point value with 8 fractional
 * bits.
 */
static float ds1621_decode_temp(const uint8_t* const raw)
{
    const int16_t counts =
        (int16_t)(((uint16_t)raw[0] << 8) | (uint16_t)raw[1]);
    return (float)counts / 256.0f;
}

//-----------------------------------------------------------------------------
static void ds1621_encode_temp(float temp, uint8_t* const raw)
{
    if (temp > DS1621_TEMPERATURE_MAX)
    {
        temp = DS1621_TEMPERATURE_MAX;
    }
    else if (temp < DS1621_TEMPERATURE_MIN)
    {
        temp = DS1621_TEMPERATURE_MIN;
    }

    // Round to the nearest representable 0.5 degC step
    const float scaled = temp * 2.0f;
    const int32_t half_degrees =
        (int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
    const uint16_t counts = (uint16_t)((uint32_t)(half_degrees * 128) & 0xFFFFu);

    raw[0] = (uint8_t)(counts >> 8);
    raw[1] = (uint8_t)(counts & 0x00FFu);
}

//-----------------------------------------------------------------------------
static bool ds1621_wait_for_eeprom(const ds1621* const dev)
{
    for (uint8_t retry = 0; retry < DS1621_EEPROM_POLL_RETRIES; ++retry)
    {
        ds1621_delay_ms(DS1621_EEPROM_WRITE_TIME_MS);

        uint8_t reg = 0;
        if (!ds1621_read(dev, DS1621_CMD_ACCESS_CONFIG, &reg, 1))
        {
            return false;
        }
        if ((reg & DS1621_STATUS_NVB) == 0)
        {
            return true;
        }
    }
    return false;
}

//-----------------------------------------------------------------------------
static bool ds1621_write_trip_point(const ds1621* const dev, uint8_t cmd,
                                    float temp)
{
    uint8_t raw[2] = {0};
    ds1621_encode_temp(temp, raw);

    if (!ds1621_write(dev, cmd, raw, sizeof(raw)))
    {
        return false;
    }
    return ds1621_wait_for_eeprom(dev);
}

//-----------------------------------------------------------------------------
bool ds1621_init(ds1621* const dev)
{
    if (!dev)
    {
        return false;
    }

    dev->temperature = DS1621_INVALID_TEMPERATURE;

    const ds1621_config cfg = {
        .conversion_mode = DS1621_CONVERSION_ONE_SHOT,
        .tout_polarity = DS1621_TOUT_ACTIVE_HIGH,
    };

    if (!ds1621_set_config(dev, &cfg))
    {
        return false;
    }

    // Make sure the device really answers with what has just been programmed
    ds1621_config readback;
    if (!ds1621_get_config(dev, &readback))
    {
        return false;
    }
    return readback.conversion_mode == cfg.conversion_mode &&
           readback.tout_polarity == cfg.tout_polarity;
}

//-----------------------------------------------------------------------------
bool ds1621_set_config(const ds1621* const dev, const ds1621_config* const cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    uint8_t reg = 0;
    if (cfg->tout_polarity == DS1621_TOUT_ACTIVE_HIGH)
    {
        reg |= DS1621_CFG_POL;
    }
    if (cfg->conversion_mode == DS1621_CONVERSION_ONE_SHOT)
    {
        reg |= DS1621_CFG_ONE_SHOT;
    }

    if (!ds1621_write(dev, DS1621_CMD_ACCESS_CONFIG, &reg, 1))
    {
        return false;
    }
    return ds1621_wait_for_eeprom(dev);
}

//-----------------------------------------------------------------------------
bool ds1621_get_config(const ds1621* const dev, ds1621_config* const cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    uint8_t reg = 0;
    if (!ds1621_read(dev, DS1621_CMD_ACCESS_CONFIG, &reg, 1))
    {
        return false;
    }

    cfg->tout_polarity = ((reg & DS1621_CFG_POL) != 0) ?
                         DS1621_TOUT_ACTIVE_HIGH : DS1621_TOUT_ACTIVE_LOW;
    cfg->conversion_mode = ((reg & DS1621_CFG_ONE_SHOT) != 0) ?
                           DS1621_CONVERSION_ONE_SHOT : DS1621_CONVERSION_CONTINUOUS;
    return true;
}

//-----------------------------------------------------------------------------
bool ds1621_read_status(const ds1621* const dev, ds1621_status* const status)
{
    if (!dev || !status)
    {
        return false;
    }

    uint8_t reg = 0;
    if (!ds1621_read(dev, DS1621_CMD_ACCESS_CONFIG, &reg, 1))
    {
        return false;
    }

    status->conversion_done = (reg & DS1621_STATUS_DONE) != 0;
    status->temp_high_flag = (reg & DS1621_STATUS_THF) != 0;
    status->temp_low_flag = (reg & DS1621_STATUS_TLF) != 0;
    status->eeprom_busy = (reg & DS1621_STATUS_NVB) != 0;
    return true;
}

//-----------------------------------------------------------------------------
bool ds1621_clear_alarm_flags(const ds1621* const dev)
{
    if (!dev)
    {
        return false;
    }

    uint8_t reg = 0;
    if (!ds1621_read(dev, DS1621_CMD_ACCESS_CONFIG, &reg, 1))
    {
        return false;
    }

    reg = (uint8_t)(reg & (uint8_t)~(DS1621_STATUS_THF | DS1621_STATUS_TLF));
    if (!ds1621_write(dev, DS1621_CMD_ACCESS_CONFIG, &reg, 1))
    {
        return false;
    }
    return ds1621_wait_for_eeprom(dev);
}

//-----------------------------------------------------------------------------
bool ds1621_start_conversion(const ds1621* const dev)
{
    if (!dev)
    {
        return false;
    }
    return ds1621_write(dev, DS1621_CMD_START_CONVERT, NULL, 0);
}

//-----------------------------------------------------------------------------
bool ds1621_stop_conversion(const ds1621* const dev)
{
    if (!dev)
    {
        return false;
    }
    return ds1621_write(dev, DS1621_CMD_STOP_CONVERT, NULL, 0);
}

//-----------------------------------------------------------------------------
bool ds1621_read_temperature(ds1621* const dev, float* temp)
{
    if (!dev)
    {
        return false;
    }

    uint8_t raw[2] = {0};
    if (!ds1621_read(dev, DS1621_CMD_READ_TEMPERATURE, raw, sizeof(raw)))
    {
        dev->temperature = DS1621_INVALID_TEMPERATURE;
        return false;
    }

    dev->temperature = ds1621_decode_temp(raw);
    if (temp)
    {
        *temp = dev->temperature;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool ds1621_read_temperature_hires(ds1621* const dev, float* temp)
{
    if (!dev)
    {
        return false;
    }

    uint8_t raw[2] = {0};
    uint8_t counter = 0;
    uint8_t slope = 0;

    if (!ds1621_read(dev, DS1621_CMD_READ_TEMPERATURE, raw, sizeof(raw)) ||
        !ds1621_read(dev, DS1621_CMD_READ_COUNTER, &counter, 1) ||
        !ds1621_read(dev, DS1621_CMD_READ_SLOPE, &slope, 1) ||
        slope == 0)
    {
        dev->temperature = DS1621_INVALID_TEMPERATURE;
        return false;
    }

    // The 0.5 degC bit must be dropped before the slope correction is applied
    const float truncated = (float)(int8_t)raw[0];
    dev->temperature = truncated - 0.25f +
                       (((float)slope - (float)counter) / (float)slope);
    if (temp)
    {
        *temp = dev->temperature;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool ds1621_measure_temperature(ds1621* const dev, bool hires, float* temp)
{
    if (!dev)
    {
        return false;
    }

    if (!ds1621_start_conversion(dev))
    {
        return false;
    }

    for (uint32_t elapsed = 0; elapsed <= DS1621_CONVERSION_TIME_MS;
         elapsed += DS1621_POLL_STEP_MS)
    {
        ds1621_status status;
        if (!ds1621_read_status(dev, &status))
        {
            return false;
        }
        if (status.conversion_done)
        {
            return hires ? ds1621_read_temperature_hires(dev, temp)
                         : ds1621_read_temperature(dev, temp);
        }
        ds1621_delay_ms(DS1621_POLL_STEP_MS);
    }

    dev->temperature = DS1621_INVALID_TEMPERATURE;
    return false;
}

//-----------------------------------------------------------------------------
float ds1621_get_temperature(const ds1621* const dev)
{
    if (!dev)
    {
        return DS1621_INVALID_TEMPERATURE;
    }
    return dev->temperature;
}

//-----------------------------------------------------------------------------
bool ds1621_set_thermostat(const ds1621* const dev, float th, float tl)
{
    if (!dev)
    {
        return false;
    }

    return ds1621_write_trip_point(dev, DS1621_CMD_ACCESS_TH, th) &&
           ds1621_write_trip_point(dev, DS1621_CMD_ACCESS_TL, tl);
}

//-----------------------------------------------------------------------------
bool ds1621_get_thermostat(const ds1621* const dev, float* th, float* tl)
{
    if (!dev)
    {
        return false;
    }

    uint8_t raw[2] = {0};
    if (th)
    {
        if (!ds1621_read(dev, DS1621_CMD_ACCESS_TH, raw, sizeof(raw)))
        {
            return false;
        }
        *th = ds1621_decode_temp(raw);
    }
    if (tl)
    {
        if (!ds1621_read(dev, DS1621_CMD_ACCESS_TL, raw, sizeof(raw)))
        {
            return false;
        }
        *tl = ds1621_decode_temp(raw);
    }
    return true;
}

//-----------------------------------------------------------------------------
