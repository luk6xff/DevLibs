/**
 *  @brief:   DS2782 stand-alone battery fuel gauge library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "ds2782.h"

// Function commands accepted by DS2782_REG_FUNC_COMMAND, followed by the block
#define DS2782_CMD_COPY_DATA        0x44
#define DS2782_CMD_RECALL_DATA      0xB8

// Programming time of an EEPROM block, see tEEC in the datasheet
#define DS2782_EEPROM_COPY_DELAY_MS 20

// Time needed to reload the shadow RAM from the EEPROM cells
#define DS2782_EEPROM_RECALL_DELAY_MS 1

// Voltage across the sense resistor per LSB of the current registers [uV]
#define DS2782_CURRENT_LSB_UV       1.5625f

// Voltage-hours across the sense resistor per LSB of the ACR register [uVh]
#define DS2782_ACR_LSB_UVH          6.25f

// Battery voltage per LSB of the VOLT register [mV]
#define DS2782_VOLTAGE_LSB_MV       4.88f

// Temperature per LSB of the TEMP register [degC]
#define DS2782_TEMP_LSB_C           0.125f

// Charge per LSB of the RAAC and RSAC registers [mAh]
#define DS2782_ABS_CAPACITY_LSB_MAH 1.6f

// Age scalar per LSB of the AS register [%]
#define DS2782_AGE_SCALAR_LSB_PCT   0.78125f

// The 11-bit registers are left justified within their 16-bit word
#define DS2782_11BIT_SHIFT          5
#define DS2782_11BIT_SIGN           0x0400
#define DS2782_11BIT_RANGE          0x0800

#define DS2782_16BIT_SIGN           0x8000
#define DS2782_16BIT_RANGE          0x10000

//-----------------------------------------------------------------------------
static uint16_t ds2782_rsense(const ds2782* const dev)
{
    if (dev->rsense_mohm == 0)
    {
        return (uint16_t)DS2782_DEFAULT_RSENSE_MOHM;
    }
    return dev->rsense_mohm;
}

//-----------------------------------------------------------------------------
static bool ds2782_read_u16(const ds2782* const dev, uint8_t reg_addr,
                            uint16_t* const value)
{
    uint8_t raw[2] = {0};
    if (!ds2782_read(dev, reg_addr, raw, sizeof(raw)))
    {
        return false;
    }

    *value = (uint16_t)(((uint16_t)raw[0] << 8) | (uint16_t)raw[1]);
    return true;
}

//-----------------------------------------------------------------------------
static int32_t ds2782_sign_extend_16(uint16_t raw)
{
    int32_t value = (int32_t)raw;
    if ((value & DS2782_16BIT_SIGN) != 0)
    {
        value -= DS2782_16BIT_RANGE;
    }
    return value;
}

//-----------------------------------------------------------------------------
static int32_t ds2782_sign_extend_11(uint16_t raw)
{
    int32_t value = (int32_t)(raw >> DS2782_11BIT_SHIFT);
    if ((value & DS2782_11BIT_SIGN) != 0)
    {
        value -= DS2782_11BIT_RANGE;
    }
    return value;
}

//-----------------------------------------------------------------------------
static bool ds2782_read_current_reg(const ds2782* const dev, uint8_t reg_addr,
                                    float* const current_ma)
{
    uint16_t raw = 0;
    if (!ds2782_read_u16(dev, reg_addr, &raw))
    {
        return false;
    }

    const float counts = (float)ds2782_sign_extend_16(raw);
    *current_ma = counts * DS2782_CURRENT_LSB_UV / (float)ds2782_rsense(dev);
    return true;
}

//-----------------------------------------------------------------------------
static bool ds2782_read_abs_capacity_reg(const ds2782* const dev,
                                         uint8_t reg_addr,
                                         float* const charge_mah)
{
    uint16_t raw = 0;
    if (!ds2782_read_u16(dev, reg_addr, &raw))
    {
        return false;
    }

    *charge_mah = (float)raw * DS2782_ABS_CAPACITY_LSB_MAH;
    return true;
}

//-----------------------------------------------------------------------------
static bool ds2782_read_u8(const ds2782* const dev, uint8_t reg_addr,
                           uint8_t* const value)
{
    return ds2782_read(dev, reg_addr, value, 1);
}

//-----------------------------------------------------------------------------
static bool ds2782_function_command(const ds2782* const dev, uint8_t command,
                                    ds2782_eeprom_block block)
{
    const uint8_t payload[2] = {command, (uint8_t)block};

    return ds2782_write(dev, DS2782_REG_FUNC_COMMAND, payload, sizeof(payload));
}

//-----------------------------------------------------------------------------
bool ds2782_init(ds2782* const dev)
{
    if (!dev)
    {
        return false;
    }

    if (dev->rsense_mohm == 0)
    {
        dev->rsense_mohm = (uint16_t)DS2782_DEFAULT_RSENSE_MOHM;
    }

    // The DS2782 has no ID register, reading the status proves it answers
    uint8_t status = 0;
    return ds2782_read_status(dev, &status);
}

//-----------------------------------------------------------------------------
bool ds2782_read_status(const ds2782* const dev, uint8_t* const status)
{
    if (!dev || !status)
    {
        return false;
    }

    return ds2782_read_u8(dev, DS2782_REG_STATUS, status);
}

//-----------------------------------------------------------------------------
bool ds2782_read_voltage(const ds2782* const dev, float* const voltage_mv)
{
    if (!dev || !voltage_mv)
    {
        return false;
    }

    uint16_t raw = 0;
    if (!ds2782_read_u16(dev, DS2782_REG_VOLT_MSB, &raw))
    {
        return false;
    }

    *voltage_mv = (float)ds2782_sign_extend_11(raw) * DS2782_VOLTAGE_LSB_MV;
    return true;
}

//-----------------------------------------------------------------------------
bool ds2782_read_current(const ds2782* const dev, float* const current_ma)
{
    if (!dev || !current_ma)
    {
        return false;
    }

    return ds2782_read_current_reg(dev, DS2782_REG_CURRENT_MSB, current_ma);
}

//-----------------------------------------------------------------------------
bool ds2782_read_average_current(const ds2782* const dev,
                                 float* const current_ma)
{
    if (!dev || !current_ma)
    {
        return false;
    }

    return ds2782_read_current_reg(dev, DS2782_REG_IAVG_MSB, current_ma);
}

//-----------------------------------------------------------------------------
bool ds2782_read_accumulated_current(const ds2782* const dev,
                                     float* const charge_mah)
{
    if (!dev || !charge_mah)
    {
        return false;
    }

    uint16_t raw = 0;
    if (!ds2782_read_u16(dev, DS2782_REG_ACR_MSB, &raw))
    {
        return false;
    }

    const float counts = (float)ds2782_sign_extend_16(raw);
    *charge_mah = counts * DS2782_ACR_LSB_UVH / (float)ds2782_rsense(dev);
    return true;
}

//-----------------------------------------------------------------------------
bool ds2782_write_accumulated_current(const ds2782* const dev,
                                      float charge_mah)
{
    if (!dev)
    {
        return false;
    }

    float counts = charge_mah * (float)ds2782_rsense(dev) / DS2782_ACR_LSB_UVH;
    if (counts > 32767.0f)
    {
        counts = 32767.0f;
    }
    else if (counts < -32768.0f)
    {
        counts = -32768.0f;
    }

    const uint16_t raw = (uint16_t)((int32_t)counts & 0xFFFF);
    const uint8_t buf[2] = {(uint8_t)(raw >> 8), (uint8_t)(raw & 0xFFu)};

    return ds2782_write(dev, DS2782_REG_ACR_MSB, buf, sizeof(buf));
}

//-----------------------------------------------------------------------------
bool ds2782_read_temperature(const ds2782* const dev, float* const temp_c)
{
    if (!dev || !temp_c)
    {
        return false;
    }

    uint16_t raw = 0;
    if (!ds2782_read_u16(dev, DS2782_REG_TEMP_MSB, &raw))
    {
        return false;
    }

    *temp_c = (float)ds2782_sign_extend_11(raw) * DS2782_TEMP_LSB_C;
    return true;
}

//-----------------------------------------------------------------------------
bool ds2782_read_active_relative_capacity(const ds2782* const dev,
                                          uint8_t* const percent)
{
    if (!dev || !percent)
    {
        return false;
    }

    return ds2782_read_u8(dev, DS2782_REG_RARC, percent);
}

//-----------------------------------------------------------------------------
bool ds2782_read_standby_relative_capacity(const ds2782* const dev,
                                           uint8_t* const percent)
{
    if (!dev || !percent)
    {
        return false;
    }

    return ds2782_read_u8(dev, DS2782_REG_RSRC, percent);
}

//-----------------------------------------------------------------------------
bool ds2782_read_active_absolute_capacity(const ds2782* const dev,
                                          float* const charge_mah)
{
    if (!dev || !charge_mah)
    {
        return false;
    }

    return ds2782_read_abs_capacity_reg(dev, DS2782_REG_RAAC_MSB, charge_mah);
}

//-----------------------------------------------------------------------------
bool ds2782_read_standby_absolute_capacity(const ds2782* const dev,
                                           float* const charge_mah)
{
    if (!dev || !charge_mah)
    {
        return false;
    }

    return ds2782_read_abs_capacity_reg(dev, DS2782_REG_RSAC_MSB, charge_mah);
}

//-----------------------------------------------------------------------------
bool ds2782_read_age_scalar(const ds2782* const dev, float* const percent)
{
    if (!dev || !percent)
    {
        return false;
    }

    uint8_t raw = 0;
    if (!ds2782_read_u8(dev, DS2782_REG_AS, &raw))
    {
        return false;
    }

    *percent = (float)raw * DS2782_AGE_SCALAR_LSB_PCT;
    return true;
}

//-----------------------------------------------------------------------------
bool ds2782_read_eeprom(const ds2782* const dev, uint8_t reg_addr,
                        uint8_t* const buf, size_t buf_size)
{
    if (!dev || !buf || buf_size == 0)
    {
        return false;
    }

    return ds2782_read(dev, reg_addr, buf, buf_size);
}

//-----------------------------------------------------------------------------
bool ds2782_write_eeprom(const ds2782* const dev, uint8_t reg_addr,
                         const uint8_t* const buf, size_t buf_size)
{
    if (!dev || !buf || buf_size == 0)
    {
        return false;
    }

    return ds2782_write(dev, reg_addr, buf, buf_size);
}

//-----------------------------------------------------------------------------
bool ds2782_copy_eeprom_block(const ds2782* const dev,
                              ds2782_eeprom_block block)
{
    if (!dev)
    {
        return false;
    }

    if (!ds2782_function_command(dev, DS2782_CMD_COPY_DATA, block))
    {
        return false;
    }

    ds2782_delay_ms(DS2782_EEPROM_COPY_DELAY_MS);
    return true;
}

//-----------------------------------------------------------------------------
bool ds2782_recall_eeprom_block(const ds2782* const dev,
                                ds2782_eeprom_block block)
{
    if (!dev)
    {
        return false;
    }

    if (!ds2782_function_command(dev, DS2782_CMD_RECALL_DATA, block))
    {
        return false;
    }

    ds2782_delay_ms(DS2782_EEPROM_RECALL_DELAY_MS);
    return true;
}

//-----------------------------------------------------------------------------
