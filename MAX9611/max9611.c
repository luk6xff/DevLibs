/**
 *  @brief:   MAX9611 high-side current-sense amplifier with 12-bit ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "max9611.h"

// CTRL1 register bit masks
#define MAX9611_CTRL1_MODE_POS   5
#define MAX9611_CTRL1_MODE_MASK  0xE0u
#define MAX9611_CTRL1_LR_BIT     0x10u
#define MAX9611_CTRL1_SHDN_BIT   0x08u
#define MAX9611_CTRL1_MUX_MASK   0x07u

// CTRL2 register bit masks
#define MAX9611_CTRL2_DTIM_BIT   0x08u
#define MAX9611_CTRL2_RTIM_BIT   0x04u

// Sign bit of the 9-bit two's complement temperature reading
#define MAX9611_TEMP_SIGN_BIT    0x0100u
#define MAX9611_TEMP_MODULO      512

//-----------------------------------------------------------------------------
static bool max9611_mux_to_data_reg(max9611_mux mux, uint8_t* reg)
{
    switch (mux)
    {
        case MAX9611_MUX_CSA_GAIN_1X:
        case MAX9611_MUX_CSA_GAIN_4X:
        case MAX9611_MUX_CSA_GAIN_8X:
            *reg = (uint8_t)MAX9611_REG_CSA_MSB;
            break;
        case MAX9611_MUX_RS_COMMON_MODE:
            *reg = (uint8_t)MAX9611_REG_RS_MSB;
            break;
        case MAX9611_MUX_OUT_VOLTAGE:
            *reg = (uint8_t)MAX9611_REG_OUT_MSB;
            break;
        case MAX9611_MUX_SET_VOLTAGE:
            *reg = (uint8_t)MAX9611_REG_SET_MSB;
            break;
        case MAX9611_MUX_TEMPERATURE:
            *reg = (uint8_t)MAX9611_REG_TEMP_MSB;
            break;
        case MAX9611_MUX_ALL_CHANNELS:
        default:
            return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
// The data registers are left aligned, the LSB register carries the remaining
// bits in its most significant positions.
static uint16_t max9611_combine_12bit(uint8_t msb, uint8_t lsb)
{
    return (uint16_t)(((uint16_t)msb << 4) | (uint16_t)((lsb >> 4) & 0x0Fu));
}

//-----------------------------------------------------------------------------
static uint16_t max9611_combine_9bit(uint8_t msb, uint8_t lsb)
{
    return (uint16_t)(((uint16_t)msb << 1) | (uint16_t)((lsb >> 7) & 0x01u));
}

//-----------------------------------------------------------------------------
static float max9611_csa_lsb_uv(max9611_gain gain)
{
    switch (gain)
    {
        case MAX9611_GAIN_4X:
            return MAX9611_CSA_LSB_UV_GAIN_4X;
        case MAX9611_GAIN_8X:
            return MAX9611_CSA_LSB_UV_GAIN_8X;
        case MAX9611_GAIN_1X:
        default:
            return MAX9611_CSA_LSB_UV_GAIN_1X;
    }
}

//-----------------------------------------------------------------------------
bool max9611_init(max9611* const dev)
{
    if (!dev || dev->shunt_res_ohm <= 0.0f)
    {
        return false;
    }

    const max9611_config cfg = {
        .mode = MAX9611_MODE_NORMAL,
        .shutdown = false,
        .mux = (max9611_mux)dev->gain, // Gain values map 1:1 onto the CSA mux channels
        .watchdog_delay = MAX9611_WATCHDOG_DELAY_1MS,
        .watchdog_retry = MAX9611_WATCHDOG_RETRY_50MS,
    };

    if (!max9611_set_config(dev, &cfg))
    {
        return false;
    }

    max9611_delay_ms(MAX9611_CONVERSION_TIME_MS);
    return true;
}

//-----------------------------------------------------------------------------
bool max9611_set_config(const max9611* const dev,
                        const max9611_config* const cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    uint8_t ctrl1 = (uint8_t)((((uint8_t)cfg->mode << MAX9611_CTRL1_MODE_POS) &
                               MAX9611_CTRL1_MODE_MASK) |
                              ((uint8_t)cfg->mux & MAX9611_CTRL1_MUX_MASK));
    if (cfg->shutdown)
    {
        ctrl1 |= MAX9611_CTRL1_SHDN_BIT;
    }

    uint8_t ctrl2 = 0;
    if (cfg->watchdog_delay == MAX9611_WATCHDOG_DELAY_100US)
    {
        ctrl2 |= MAX9611_CTRL2_DTIM_BIT;
    }
    if (cfg->watchdog_retry == MAX9611_WATCHDOG_RETRY_10MS)
    {
        ctrl2 |= MAX9611_CTRL2_RTIM_BIT;
    }

    // CTRL1 and CTRL2 are adjacent, so a single burst write covers both
    const uint8_t buf[2] = {ctrl1, ctrl2};
    return max9611_write(dev, (uint8_t)MAX9611_REG_CTRL1, buf, sizeof(buf));
}

//-----------------------------------------------------------------------------
bool max9611_get_config(const max9611* const dev, max9611_config* const cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    uint8_t buf[2] = {0, 0};
    if (!max9611_read(dev, (uint8_t)MAX9611_REG_CTRL1, buf, sizeof(buf)))
    {
        return false;
    }

    cfg->mode = (max9611_mode)((buf[0] & MAX9611_CTRL1_MODE_MASK) >>
                               MAX9611_CTRL1_MODE_POS);
    cfg->shutdown = (buf[0] & MAX9611_CTRL1_SHDN_BIT) != 0;
    cfg->mux = (max9611_mux)(buf[0] & MAX9611_CTRL1_MUX_MASK);
    cfg->watchdog_delay = ((buf[1] & MAX9611_CTRL2_DTIM_BIT) != 0)
                              ? MAX9611_WATCHDOG_DELAY_100US
                              : MAX9611_WATCHDOG_DELAY_1MS;
    cfg->watchdog_retry = ((buf[1] & MAX9611_CTRL2_RTIM_BIT) != 0)
                              ? MAX9611_WATCHDOG_RETRY_10MS
                              : MAX9611_WATCHDOG_RETRY_50MS;
    return true;
}

//-----------------------------------------------------------------------------
bool max9611_select_channel(const max9611* const dev, max9611_mux mux)
{
    if (!dev)
    {
        return false;
    }

    uint8_t ctrl1 = 0;
    if (!max9611_read(dev, (uint8_t)MAX9611_REG_CTRL1, &ctrl1, 1))
    {
        return false;
    }

    const uint8_t new_ctrl1 =
        (uint8_t)((ctrl1 & (uint8_t)~MAX9611_CTRL1_MUX_MASK) |
                  ((uint8_t)mux & MAX9611_CTRL1_MUX_MASK));

    if (new_ctrl1 != ctrl1)
    {
        if (!max9611_write(dev, (uint8_t)MAX9611_REG_CTRL1, &new_ctrl1, 1))
        {
            return false;
        }
    }

    // Let the ADC finish a conversion of the freshly selected channel
    max9611_delay_ms(MAX9611_CONVERSION_TIME_MS);
    return true;
}

//-----------------------------------------------------------------------------
bool max9611_set_gain(max9611* const dev, max9611_gain gain)
{
    if (!dev)
    {
        return false;
    }

    if (!max9611_select_channel(dev, (max9611_mux)gain))
    {
        return false;
    }

    dev->gain = gain;
    return true;
}

//-----------------------------------------------------------------------------
bool max9611_read_raw(const max9611* const dev, max9611_mux mux, uint16_t* raw)
{
    if (!dev || !raw)
    {
        return false;
    }

    uint8_t data_reg = 0;
    if (!max9611_mux_to_data_reg(mux, &data_reg))
    {
        return false;
    }

    if (!max9611_select_channel(dev, mux))
    {
        return false;
    }

    uint8_t buf[2] = {0, 0};
    if (!max9611_read(dev, data_reg, buf, sizeof(buf)))
    {
        return false;
    }

    *raw = (mux == MAX9611_MUX_TEMPERATURE)
               ? max9611_combine_9bit(buf[0], buf[1])
               : max9611_combine_12bit(buf[0], buf[1]);
    return true;
}

//-----------------------------------------------------------------------------
bool max9611_read_shunt_voltage(const max9611* const dev, float* voltage_mv)
{
    if (!dev || !voltage_mv)
    {
        return false;
    }

    uint16_t raw = 0;
    if (!max9611_read_raw(dev, (max9611_mux)dev->gain, &raw))
    {
        return false;
    }

    if (raw <= dev->csa_offset)
    {
        *voltage_mv = 0.0f;
        return true;
    }

    const uint16_t corrected = (uint16_t)(raw - dev->csa_offset);
    *voltage_mv = ((float)corrected * max9611_csa_lsb_uv(dev->gain)) / 1000.0f;
    return true;
}

//-----------------------------------------------------------------------------
bool max9611_read_current(const max9611* const dev, float* current_ma)
{
    if (!dev || !current_ma || dev->shunt_res_ohm <= 0.0f)
    {
        return false;
    }

    float voltage_mv = 0.0f;
    if (!max9611_read_shunt_voltage(dev, &voltage_mv))
    {
        return false;
    }

    // [mV] / [Ohm] gives [mA] directly
    *current_ma = voltage_mv / dev->shunt_res_ohm;
    return true;
}

//-----------------------------------------------------------------------------
bool max9611_read_input_voltage(const max9611* const dev, float* voltage_v)
{
    if (!dev || !voltage_v)
    {
        return false;
    }

    uint16_t raw = 0;
    if (!max9611_read_raw(dev, MAX9611_MUX_RS_COMMON_MODE, &raw))
    {
        return false;
    }

    *voltage_v = (float)raw * MAX9611_RS_LSB_V;
    return true;
}

//-----------------------------------------------------------------------------
bool max9611_read_out_voltage(const max9611* const dev, float* voltage_v)
{
    if (!dev || !voltage_v)
    {
        return false;
    }

    uint16_t raw = 0;
    if (!max9611_read_raw(dev, MAX9611_MUX_OUT_VOLTAGE, &raw))
    {
        return false;
    }

    *voltage_v = (float)raw * MAX9611_OUT_LSB_V;
    return true;
}

//-----------------------------------------------------------------------------
bool max9611_read_set_voltage(const max9611* const dev, float* voltage_v)
{
    if (!dev || !voltage_v)
    {
        return false;
    }

    uint16_t raw = 0;
    if (!max9611_read_raw(dev, MAX9611_MUX_SET_VOLTAGE, &raw))
    {
        return false;
    }

    *voltage_v = (float)raw * MAX9611_OUT_LSB_V;
    return true;
}

//-----------------------------------------------------------------------------
bool max9611_read_temperature(const max9611* const dev, float* temp_c)
{
    if (!dev || !temp_c)
    {
        return false;
    }

    uint16_t raw = 0;
    if (!max9611_read_raw(dev, MAX9611_MUX_TEMPERATURE, &raw))
    {
        return false;
    }

    // 9-bit two's complement reading
    int32_t value = (int32_t)raw;
    if ((raw & MAX9611_TEMP_SIGN_BIT) != 0u)
    {
        value -= MAX9611_TEMP_MODULO;
    }

    *temp_c = (float)value * MAX9611_TEMP_LSB_C;
    return true;
}

//-----------------------------------------------------------------------------
