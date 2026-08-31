/**
 *  @brief:   SI7020 relative humidity and temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "si7020.h"

// Number of bytes returned by a measurement: MSB, LSB and the CRC
#define SI7020_MEASUREMENT_SIZE     3

// Bytes returned by the two halves of the electronic serial number read
#define SI7020_ID_1_SIZE            8
#define SI7020_ID_2_SIZE            6

//-----------------------------------------------------------------------------
uint8_t si7020_crc8(const uint8_t* data, size_t len)
{
    if (!data)
    {
        return 0;
    }

    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            if ((crc & 0x80u) != 0)
            {
                crc = (uint8_t)((uint8_t)(crc << 1) ^ SI7020_CRC8_POLYNOMIAL);
            }
            else
            {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

//-----------------------------------------------------------------------------
static bool si7020_send_cmd(const si7020* const dev, uint8_t cmd)
{
    return si7020_write_cmd(dev, &cmd, 1);
}

//-----------------------------------------------------------------------------
static bool si7020_read_user_reg(const si7020* const dev, uint8_t* reg)
{
    if (!si7020_send_cmd(dev, SI7020_CMD_READ_USER_REG1))
    {
        return false;
    }
    return si7020_read_raw(dev, reg, 1);
}

//-----------------------------------------------------------------------------
/**
 * @brief Run a no hold master mode conversion and collect the CRC checked result.
 */
static bool si7020_measure(const si7020* const dev, uint8_t cmd, uint16_t* raw)
{
    if (!si7020_send_cmd(dev, cmd))
    {
        return false;
    }

    // The bus is released during the conversion, so simply wait it out
    si7020_delay_ms(SI7020_CONVERSION_TIME_MS);

    uint8_t buf[SI7020_MEASUREMENT_SIZE] = {0};
    if (!si7020_read_raw(dev, buf, sizeof(buf)))
    {
        return false;
    }
    if (si7020_crc8(buf, 2) != buf[2])
    {
        return false;
    }

    *raw = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    return true;
}

//-----------------------------------------------------------------------------
static float si7020_raw_to_humidity(uint16_t raw)
{
    float rh = ((125.0f * (float)raw) / 65536.0f) - 6.0f;

    // The transfer function slightly overshoots at both ends of the range
    if (rh < 0.0f)
    {
        rh = 0.0f;
    }
    else if (rh > 100.0f)
    {
        rh = 100.0f;
    }
    return rh;
}

//-----------------------------------------------------------------------------
static float si7020_raw_to_temperature(uint16_t raw)
{
    return ((175.72f * (float)raw) / 65536.0f) - 46.85f;
}

//-----------------------------------------------------------------------------
bool si7020_init(si7020* const dev)
{
    if (!dev)
    {
        return false;
    }

    if (!si7020_reset(dev))
    {
        return false;
    }

    uint8_t serial[SI7020_SERIAL_NUMBER_SIZE] = {0};
    if (!si7020_read_serial_number(dev, serial, sizeof(serial)))
    {
        return false;
    }
    if (serial[4] != SI7020_DEVICE_ID)
    {
        return false;
    }

    const si7020_config cfg = {
        .resolution = SI7020_RESOLUTION_RH12_TEMP14,
        .heater_enabled = false,
    };
    return si7020_set_config(dev, &cfg);
}

//-----------------------------------------------------------------------------
bool si7020_set_config(const si7020* const dev, const si7020_config* const cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    uint8_t reg = 0;
    if (!si7020_read_user_reg(dev, &reg))
    {
        return false;
    }

    // Keep the reserved bits untouched, only the resolution and heater change
    reg = (uint8_t)(reg & (uint8_t)~(SI7020_USER_REG1_RES_MASK |
                                     SI7020_USER_REG1_HTRE));
    reg = (uint8_t)(reg | ((uint8_t)cfg->resolution & SI7020_USER_REG1_RES_MASK));
    if (cfg->heater_enabled)
    {
        reg |= SI7020_USER_REG1_HTRE;
    }

    const uint8_t frame[2] = {SI7020_CMD_WRITE_USER_REG1, reg};
    return si7020_write_cmd(dev, frame, sizeof(frame));
}

//-----------------------------------------------------------------------------
bool si7020_get_config(const si7020* const dev, si7020_config* const cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    uint8_t reg = 0;
    if (!si7020_read_user_reg(dev, &reg))
    {
        return false;
    }

    cfg->resolution =
        (si7020_resolution)(reg & SI7020_USER_REG1_RES_MASK);
    cfg->heater_enabled = (reg & SI7020_USER_REG1_HTRE) != 0;
    return true;
}

//-----------------------------------------------------------------------------
bool si7020_read_humidity(si7020* const dev, float* humidity)
{
    if (!dev)
    {
        return false;
    }

    uint16_t raw = 0;
    if (!si7020_measure(dev, SI7020_CMD_MEASURE_RH_NO_HOLD, &raw))
    {
        dev->humidity = SI7020_INVALID_HUMIDITY;
        return false;
    }

    dev->humidity = si7020_raw_to_humidity(raw);
    if (humidity)
    {
        *humidity = dev->humidity;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool si7020_read_temperature(si7020* const dev, float* temp)
{
    if (!dev)
    {
        return false;
    }

    uint16_t raw = 0;
    if (!si7020_measure(dev, SI7020_CMD_MEASURE_TEMP_NO_HOLD, &raw))
    {
        dev->temperature = SI7020_INVALID_TEMPERATURE;
        return false;
    }

    dev->temperature = si7020_raw_to_temperature(raw);
    if (temp)
    {
        *temp = dev->temperature;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool si7020_read_temperature_from_last_humidity(si7020* const dev, float* temp)
{
    if (!dev)
    {
        return false;
    }

    // This command returns a cached value, it has no CRC byte appended
    uint8_t buf[2] = {0};
    if (!si7020_send_cmd(dev, SI7020_CMD_READ_TEMP_FROM_RH) ||
        !si7020_read_raw(dev, buf, sizeof(buf)))
    {
        dev->temperature = SI7020_INVALID_TEMPERATURE;
        return false;
    }

    const uint16_t raw = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    dev->temperature = si7020_raw_to_temperature(raw);
    if (temp)
    {
        *temp = dev->temperature;
    }
    return true;
}

//-----------------------------------------------------------------------------
float si7020_get_humidity(const si7020* const dev)
{
    if (!dev)
    {
        return SI7020_INVALID_HUMIDITY;
    }
    return dev->humidity;
}

//-----------------------------------------------------------------------------
float si7020_get_temperature(const si7020* const dev)
{
    if (!dev)
    {
        return SI7020_INVALID_TEMPERATURE;
    }
    return dev->temperature;
}

//-----------------------------------------------------------------------------
bool si7020_is_vdd_ok(const si7020* const dev, bool* vdd_ok)
{
    if (!dev || !vdd_ok)
    {
        return false;
    }

    uint8_t reg = 0;
    if (!si7020_read_user_reg(dev, &reg))
    {
        return false;
    }

    *vdd_ok = (reg & SI7020_USER_REG1_VDDS) == 0;
    return true;
}

//-----------------------------------------------------------------------------
bool si7020_read_firmware_revision(const si7020* const dev, uint8_t* rev)
{
    if (!dev || !rev)
    {
        return false;
    }

    const uint8_t cmd[2] = {SI7020_CMD_READ_FW_REV_BYTE1,
                            SI7020_CMD_READ_FW_REV_BYTE2};
    if (!si7020_write_cmd(dev, cmd, sizeof(cmd)))
    {
        return false;
    }
    return si7020_read_raw(dev, rev, 1);
}

//-----------------------------------------------------------------------------
bool si7020_read_serial_number(const si7020* const dev, uint8_t* serial,
                               size_t serial_size)
{
    if (!dev || !serial || serial_size != SI7020_SERIAL_NUMBER_SIZE)
    {
        return false;
    }

    // First half: SNA_3, CRC, SNA_2, CRC, SNA_1, CRC, SNA_0, CRC
    const uint8_t cmd1[2] = {SI7020_CMD_READ_ID_1_BYTE1,
                             SI7020_CMD_READ_ID_1_BYTE2};
    uint8_t buf1[SI7020_ID_1_SIZE] = {0};
    if (!si7020_write_cmd(dev, cmd1, sizeof(cmd1)) ||
        !si7020_read_raw(dev, buf1, sizeof(buf1)))
    {
        return false;
    }

    // Second half: SNB_3, SNB_2, CRC, SNB_1, SNB_0, CRC
    const uint8_t cmd2[2] = {SI7020_CMD_READ_ID_2_BYTE1,
                             SI7020_CMD_READ_ID_2_BYTE2};
    uint8_t buf2[SI7020_ID_2_SIZE] = {0};
    if (!si7020_write_cmd(dev, cmd2, sizeof(cmd2)) ||
        !si7020_read_raw(dev, buf2, sizeof(buf2)))
    {
        return false;
    }

    serial[0] = buf1[0];
    serial[1] = buf1[2];
    serial[2] = buf1[4];
    serial[3] = buf1[6];
    serial[4] = buf2[0];
    serial[5] = buf2[1];
    serial[6] = buf2[3];
    serial[7] = buf2[4];
    return true;
}

//-----------------------------------------------------------------------------
bool si7020_reset(si7020* const dev)
{
    if (!dev)
    {
        return false;
    }

    dev->temperature = SI7020_INVALID_TEMPERATURE;
    dev->humidity = SI7020_INVALID_HUMIDITY;

    if (!si7020_send_cmd(dev, SI7020_CMD_RESET))
    {
        return false;
    }

    si7020_delay_ms(SI7020_RESET_TIME_MS);
    return true;
}

//-----------------------------------------------------------------------------
