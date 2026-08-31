/**
 *  @brief:   MPU6050 6-axis motion tracking device (gyroscope + accelerometer) library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "mpu6050.h"

// PWR_MGMT_1 register bits
#define MPU6050_PWR1_DEVICE_RESET   0x80u
#define MPU6050_PWR1_SLEEP          0x40u
#define MPU6050_PWR1_CLKSEL_MASK    0x07u

// PWR_MGMT_2 register bits
#define MPU6050_PWR2_STBY_XA        0x20u
#define MPU6050_PWR2_STBY_YA        0x10u
#define MPU6050_PWR2_STBY_ZA        0x08u
#define MPU6050_PWR2_STBY_XG        0x04u
#define MPU6050_PWR2_STBY_YG        0x02u
#define MPU6050_PWR2_STBY_ZG        0x01u
#define MPU6050_PWR2_STBY_MASK      0x3Fu

// CONFIG register bits
#define MPU6050_CONFIG_DLPF_MASK    0x07u

// GYRO_CONFIG / ACCEL_CONFIG register bits
#define MPU6050_CFG_FS_MASK         0x18u
#define MPU6050_CFG_FS_SHIFT        3u
#define MPU6050_CFG_SELF_TEST_MASK  0xE0u

// USER_CTRL register bits
#define MPU6050_USER_CTRL_FIFO_EN   0x40u
#define MPU6050_USER_CTRL_FIFO_RST  0x04u

// INT_PIN_CFG register bits
#define MPU6050_INT_PIN_ACTIVE_LOW  0x80u
#define MPU6050_INT_PIN_OPEN_DRAIN  0x40u
#define MPU6050_INT_PIN_LATCH_EN    0x20u
#define MPU6050_INT_PIN_RD_CLEAR    0x10u

// Mask of the interrupt sources exposed by mpu6050_int_source
#define MPU6050_INT_SOURCE_MASK     0x59u

// SIGNAL_PATH_RESET register bits, resets gyro, accel and temperature paths
#define MPU6050_SIGNAL_PATH_RESET_ALL 0x07u

// MOT_DETECT_CTRL register, 3 ms accelerometer power on delay
#define MPU6050_MOT_CTRL_ACCEL_DELAY_3MS 0x30u

// Temperature conversion constants, see the register map "TEMP_OUT" section
#define MPU6050_TEMP_SENSITIVITY    340.0f
#define MPU6050_TEMP_OFFSET_DEGC    36.53f

// Bounded wait used while polling the self clearing DEVICE_RESET bit
#define MPU6050_RESET_POLL_MS       10u
#define MPU6050_RESET_POLL_TRIES    20u

// Time the sensors need to stabilize after a self-test bit is toggled
#define MPU6050_SELF_TEST_SETTLE_MS 250u

// Number of samples averaged for one self-test measurement
#define MPU6050_SELF_TEST_SAMPLES   10u
#define MPU6050_SELF_TEST_PERIOD_MS 5u

// Maximum allowed deviation from the factory trim in percent
#define MPU6050_SELF_TEST_MAX_DEV   14.0f

// Gyroscope factory trim: 25 * 131 * 1.046^(code-1), see the register map
#define MPU6050_GYRO_FT_BASE        3275.0f
#define MPU6050_GYRO_FT_STEP        1.046f

// Accelerometer factory trim: 4096 * 0.34 * (0.92/0.34)^((code-1)/30)
#define MPU6050_ACCEL_FT_BASE       0.34f
#define MPU6050_ACCEL_FT_STEP       1.034f
#define MPU6050_ACCEL_FT_SCALE      4096.0f

// Gyroscope output rate feeding the sample rate divider
#define MPU6050_GYRO_RATE_NO_DLPF   8000u
#define MPU6050_GYRO_RATE_DLPF      1000u


//-----------------------------------------------------------------------------
static bool mpu6050_write_reg(const mpu6050* const dev, uint8_t reg,
                              uint8_t val)
{
    return mpu6050_write(dev, reg, &val, 1);
}

//-----------------------------------------------------------------------------
static bool mpu6050_read_reg(const mpu6050* const dev, uint8_t reg,
                             uint8_t* val)
{
    return mpu6050_read(dev, reg, val, 1);
}

//-----------------------------------------------------------------------------
static bool mpu6050_update_reg(const mpu6050* const dev, uint8_t reg,
                               uint8_t mask, uint8_t val)
{
    uint8_t cur = 0;
    if (!mpu6050_read_reg(dev, reg, &cur))
    {
        return false;
    }

    const uint8_t updated = (uint8_t)((cur & (uint8_t)(~mask)) | (val & mask));
    if (updated == cur)
    {
        return true;
    }
    return mpu6050_write_reg(dev, reg, updated);
}

//-----------------------------------------------------------------------------
static int16_t mpu6050_be16_to_int16(const uint8_t* buf)
{
    return (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

//-----------------------------------------------------------------------------
static bool mpu6050_read_axes(const mpu6050* const dev, uint8_t reg_addr,
                              mpu6050_axes_raw* const axes)
{
    uint8_t buf[6] = {0};
    if (!mpu6050_read(dev, reg_addr, buf, sizeof(buf)))
    {
        return false;
    }

    axes->x = mpu6050_be16_to_int16(&buf[0]);
    axes->y = mpu6050_be16_to_int16(&buf[2]);
    axes->z = mpu6050_be16_to_int16(&buf[4]);
    return true;
}

//-----------------------------------------------------------------------------
static float mpu6050_accel_range_sensitivity(mpu6050_accel_range range)
{
    switch (range)
    {
        case MPU6050_ACCEL_RANGE_2G:  return 16384.0f;
        case MPU6050_ACCEL_RANGE_4G:  return 8192.0f;
        case MPU6050_ACCEL_RANGE_8G:  return 4096.0f;
        case MPU6050_ACCEL_RANGE_16G: return 2048.0f;
        default:                      return 0.0f;
    }
}

//-----------------------------------------------------------------------------
static float mpu6050_gyro_range_sensitivity(mpu6050_gyro_range range)
{
    switch (range)
    {
        case MPU6050_GYRO_RANGE_250DPS:  return 131.0f;
        case MPU6050_GYRO_RANGE_500DPS:  return 65.5f;
        case MPU6050_GYRO_RANGE_1000DPS: return 32.8f;
        case MPU6050_GYRO_RANGE_2000DPS: return 16.4f;
        default:                         return 0.0f;
    }
}

//-----------------------------------------------------------------------------
bool mpu6050_init(mpu6050* const dev)
{
    if (!dev)
    {
        return false;
    }

    uint8_t id = 0;
    if (!mpu6050_read_id(dev, &id) || id != MPU6050_WHO_AM_I_VALUE)
    {
        return false;
    }

    if (!mpu6050_reset(dev))
    {
        return false;
    }

    // The device comes out of reset asleep, wake it up before configuring it
    if (!mpu6050_set_sleep(dev, false))
    {
        return false;
    }

    const mpu6050_config cfg = {
        .sample_rate_div = 9, // 1 kHz / (1 + 9) = 100 Hz
        .dlpf = MPU6050_DLPF_BW_42HZ,
        .gyro_range = MPU6050_GYRO_RANGE_250DPS,
        .accel_range = MPU6050_ACCEL_RANGE_2G,
        .clock_source = MPU6050_CLOCK_PLL_X_GYRO,
    };

    if (!mpu6050_set_config(dev, &cfg))
    {
        return false;
    }

    // Leave the FIFO and every interrupt source disabled
    if (!mpu6050_set_fifo_enabled(dev, false) ||
        !mpu6050_set_fifo_sources(dev, 0) ||
        !mpu6050_write_reg(dev, MPU6050_REG_INT_ENABLE, 0))
    {
        return false;
    }

    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_reset(mpu6050* const dev)
{
    if (!dev)
    {
        return false;
    }

    if (!mpu6050_write_reg(dev, MPU6050_REG_PWR_MGMT_1,
                           MPU6050_PWR1_DEVICE_RESET))
    {
        return false;
    }

    // DEVICE_RESET clears itself once the reset sequence has completed
    bool done = false;
    for (uint8_t i = 0; i < MPU6050_RESET_POLL_TRIES; ++i)
    {
        mpu6050_delay_ms(MPU6050_RESET_POLL_MS);

        uint8_t val = 0;
        if (!mpu6050_read_reg(dev, MPU6050_REG_PWR_MGMT_1, &val))
        {
            return false;
        }
        if ((val & MPU6050_PWR1_DEVICE_RESET) == 0)
        {
            done = true;
            break;
        }
    }

    if (!done)
    {
        return false;
    }

    if (!mpu6050_reset_signal_path(dev))
    {
        return false;
    }

    // Reset restores the power on defaults of both full-scale ranges
    dev->gyro_range = MPU6050_GYRO_RANGE_250DPS;
    dev->accel_range = MPU6050_ACCEL_RANGE_2G;
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_reset_signal_path(const mpu6050* const dev)
{
    if (!dev)
    {
        return false;
    }

    if (!mpu6050_write_reg(dev, MPU6050_REG_SIGNAL_PATH_RESET,
                           MPU6050_SIGNAL_PATH_RESET_ALL))
    {
        return false;
    }
    mpu6050_delay_ms(MPU6050_RESET_POLL_MS);
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_read_id(const mpu6050* const dev, uint8_t* id)
{
    if (!dev || !id)
    {
        return false;
    }
    return mpu6050_read_reg(dev, MPU6050_REG_WHO_AM_I, id);
}

//-----------------------------------------------------------------------------
bool mpu6050_set_config(mpu6050* const dev, const mpu6050_config* const cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    if (!mpu6050_set_clock_source(dev, cfg->clock_source) ||
        !mpu6050_set_sample_rate_divider(dev, cfg->sample_rate_div) ||
        !mpu6050_set_dlpf(dev, cfg->dlpf) ||
        !mpu6050_set_gyro_range(dev, cfg->gyro_range) ||
        !mpu6050_set_accel_range(dev, cfg->accel_range))
    {
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_get_config(mpu6050* const dev, mpu6050_config* const cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    if (!mpu6050_get_clock_source(dev, &cfg->clock_source) ||
        !mpu6050_get_sample_rate_divider(dev, &cfg->sample_rate_div) ||
        !mpu6050_get_dlpf(dev, &cfg->dlpf) ||
        !mpu6050_get_gyro_range(dev, &cfg->gyro_range) ||
        !mpu6050_get_accel_range(dev, &cfg->accel_range))
    {
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_set_sample_rate_divider(const mpu6050* const dev, uint8_t div)
{
    if (!dev)
    {
        return false;
    }
    return mpu6050_write_reg(dev, MPU6050_REG_SMPLRT_DIV, div);
}

//-----------------------------------------------------------------------------
bool mpu6050_get_sample_rate_divider(const mpu6050* const dev, uint8_t* div)
{
    if (!dev || !div)
    {
        return false;
    }
    return mpu6050_read_reg(dev, MPU6050_REG_SMPLRT_DIV, div);
}

//-----------------------------------------------------------------------------
bool mpu6050_get_sample_rate(const mpu6050* const dev, uint16_t* rate)
{
    if (!dev || !rate)
    {
        return false;
    }

    mpu6050_dlpf dlpf = MPU6050_DLPF_BW_256HZ;
    uint8_t div = 0;
    if (!mpu6050_get_dlpf(dev, &dlpf) ||
        !mpu6050_get_sample_rate_divider(dev, &div))
    {
        return false;
    }

    const uint32_t base = (dlpf == MPU6050_DLPF_BW_256HZ)
                              ? MPU6050_GYRO_RATE_NO_DLPF
                              : MPU6050_GYRO_RATE_DLPF;
    *rate = (uint16_t)(base / ((uint32_t)div + 1u));
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_set_dlpf(const mpu6050* const dev, mpu6050_dlpf dlpf)
{
    if (!dev || dlpf > MPU6050_DLPF_BW_5HZ)
    {
        return false;
    }
    return mpu6050_update_reg(dev, MPU6050_REG_CONFIG, MPU6050_CONFIG_DLPF_MASK,
                              (uint8_t)dlpf);
}

//-----------------------------------------------------------------------------
bool mpu6050_get_dlpf(const mpu6050* const dev, mpu6050_dlpf* dlpf)
{
    if (!dev || !dlpf)
    {
        return false;
    }

    uint8_t val = 0;
    if (!mpu6050_read_reg(dev, MPU6050_REG_CONFIG, &val))
    {
        return false;
    }
    *dlpf = (mpu6050_dlpf)(val & MPU6050_CONFIG_DLPF_MASK);
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_set_gyro_range(mpu6050* const dev, mpu6050_gyro_range range)
{
    if (!dev || range > MPU6050_GYRO_RANGE_2000DPS)
    {
        return false;
    }

    const uint8_t val = (uint8_t)((uint8_t)range << MPU6050_CFG_FS_SHIFT);
    if (!mpu6050_update_reg(dev, MPU6050_REG_GYRO_CONFIG, MPU6050_CFG_FS_MASK,
                            val))
    {
        return false;
    }
    dev->gyro_range = range;
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_get_gyro_range(mpu6050* const dev, mpu6050_gyro_range* range)
{
    if (!dev)
    {
        return false;
    }

    uint8_t val = 0;
    if (!mpu6050_read_reg(dev, MPU6050_REG_GYRO_CONFIG, &val))
    {
        return false;
    }

    dev->gyro_range = (mpu6050_gyro_range)((val & MPU6050_CFG_FS_MASK) >>
                                           MPU6050_CFG_FS_SHIFT);
    if (range)
    {
        *range = dev->gyro_range;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_set_accel_range(mpu6050* const dev, mpu6050_accel_range range)
{
    if (!dev || range > MPU6050_ACCEL_RANGE_16G)
    {
        return false;
    }

    const uint8_t val = (uint8_t)((uint8_t)range << MPU6050_CFG_FS_SHIFT);
    if (!mpu6050_update_reg(dev, MPU6050_REG_ACCEL_CONFIG, MPU6050_CFG_FS_MASK,
                            val))
    {
        return false;
    }
    dev->accel_range = range;
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_get_accel_range(mpu6050* const dev, mpu6050_accel_range* range)
{
    if (!dev)
    {
        return false;
    }

    uint8_t val = 0;
    if (!mpu6050_read_reg(dev, MPU6050_REG_ACCEL_CONFIG, &val))
    {
        return false;
    }

    dev->accel_range = (mpu6050_accel_range)((val & MPU6050_CFG_FS_MASK) >>
                                             MPU6050_CFG_FS_SHIFT);
    if (range)
    {
        *range = dev->accel_range;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_set_clock_source(const mpu6050* const dev,
                              mpu6050_clock_source src)
{
    // Value 6 is reserved and must never be written
    if (!dev || (src > MPU6050_CLOCK_PLL_EXT_19MHZ && src != MPU6050_CLOCK_STOP))
    {
        return false;
    }
    return mpu6050_update_reg(dev, MPU6050_REG_PWR_MGMT_1,
                              MPU6050_PWR1_CLKSEL_MASK, (uint8_t)src);
}

//-----------------------------------------------------------------------------
bool mpu6050_get_clock_source(const mpu6050* const dev,
                              mpu6050_clock_source* src)
{
    if (!dev || !src)
    {
        return false;
    }

    uint8_t val = 0;
    if (!mpu6050_read_reg(dev, MPU6050_REG_PWR_MGMT_1, &val))
    {
        return false;
    }
    *src = (mpu6050_clock_source)(val & MPU6050_PWR1_CLKSEL_MASK);
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_set_sleep(const mpu6050* const dev, bool sleep)
{
    if (!dev)
    {
        return false;
    }

    if (!mpu6050_update_reg(dev, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_SLEEP,
                            sleep ? MPU6050_PWR1_SLEEP : 0u))
    {
        return false;
    }

    if (!sleep)
    {
        // Oscillator and regulator start-up time
        mpu6050_delay_ms(MPU6050_RESET_POLL_MS);
    }
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_get_sleep(const mpu6050* const dev, bool* sleep)
{
    if (!dev || !sleep)
    {
        return false;
    }

    uint8_t val = 0;
    if (!mpu6050_read_reg(dev, MPU6050_REG_PWR_MGMT_1, &val))
    {
        return false;
    }
    *sleep = (val & MPU6050_PWR1_SLEEP) != 0;
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_set_standby(const mpu6050* const dev, bool accel_x, bool accel_y,
                         bool accel_z, bool gyro_x, bool gyro_y, bool gyro_z)
{
    if (!dev)
    {
        return false;
    }

    uint8_t val = 0;
    if (accel_x)
    {
        val |= MPU6050_PWR2_STBY_XA;
    }
    if (accel_y)
    {
        val |= MPU6050_PWR2_STBY_YA;
    }
    if (accel_z)
    {
        val |= MPU6050_PWR2_STBY_ZA;
    }
    if (gyro_x)
    {
        val |= MPU6050_PWR2_STBY_XG;
    }
    if (gyro_y)
    {
        val |= MPU6050_PWR2_STBY_YG;
    }
    if (gyro_z)
    {
        val |= MPU6050_PWR2_STBY_ZG;
    }

    return mpu6050_update_reg(dev, MPU6050_REG_PWR_MGMT_2,
                              MPU6050_PWR2_STBY_MASK, val);
}

//-----------------------------------------------------------------------------
bool mpu6050_read_raw_accel(const mpu6050* const dev,
                            mpu6050_axes_raw* const accel)
{
    if (!dev || !accel)
    {
        return false;
    }
    return mpu6050_read_axes(dev, MPU6050_REG_ACCEL_XOUT_H, accel);
}

//-----------------------------------------------------------------------------
bool mpu6050_read_raw_gyro(const mpu6050* const dev,
                           mpu6050_axes_raw* const gyro)
{
    if (!dev || !gyro)
    {
        return false;
    }
    return mpu6050_read_axes(dev, MPU6050_REG_GYRO_XOUT_H, gyro);
}

//-----------------------------------------------------------------------------
bool mpu6050_read_raw_temperature(const mpu6050* const dev, int16_t* temp)
{
    if (!dev || !temp)
    {
        return false;
    }

    uint8_t buf[2] = {0};
    if (!mpu6050_read(dev, MPU6050_REG_TEMP_OUT_H, buf, sizeof(buf)))
    {
        return false;
    }
    *temp = mpu6050_be16_to_int16(buf);
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_read_raw_all(const mpu6050* const dev,
                          mpu6050_raw_data* const data)
{
    if (!dev || !data)
    {
        return false;
    }

    uint8_t buf[MPU6050_BURST_LEN] = {0};
    if (!mpu6050_read(dev, MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf)))
    {
        return false;
    }

    data->accel.x = mpu6050_be16_to_int16(&buf[0]);
    data->accel.y = mpu6050_be16_to_int16(&buf[2]);
    data->accel.z = mpu6050_be16_to_int16(&buf[4]);
    data->temperature = mpu6050_be16_to_int16(&buf[6]);
    data->gyro.x = mpu6050_be16_to_int16(&buf[8]);
    data->gyro.y = mpu6050_be16_to_int16(&buf[10]);
    data->gyro.z = mpu6050_be16_to_int16(&buf[12]);
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_read_accel(const mpu6050* const dev, mpu6050_axes* const accel)
{
    if (!dev || !accel)
    {
        return false;
    }

    const float sens = mpu6050_get_accel_sensitivity(dev);
    if (sens == 0.0f)
    {
        return false;
    }

    mpu6050_axes_raw raw = {0, 0, 0};
    if (!mpu6050_read_raw_accel(dev, &raw))
    {
        return false;
    }

    accel->x = (float)raw.x / sens;
    accel->y = (float)raw.y / sens;
    accel->z = (float)raw.z / sens;
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_read_gyro(const mpu6050* const dev, mpu6050_axes* const gyro)
{
    if (!dev || !gyro)
    {
        return false;
    }

    const float sens = mpu6050_get_gyro_sensitivity(dev);
    if (sens == 0.0f)
    {
        return false;
    }

    mpu6050_axes_raw raw = {0, 0, 0};
    if (!mpu6050_read_raw_gyro(dev, &raw))
    {
        return false;
    }

    gyro->x = (float)raw.x / sens;
    gyro->y = (float)raw.y / sens;
    gyro->z = (float)raw.z / sens;
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_read_temperature(const mpu6050* const dev, float* temp)
{
    if (!dev || !temp)
    {
        return false;
    }

    int16_t raw = 0;
    if (!mpu6050_read_raw_temperature(dev, &raw))
    {
        return false;
    }

    *temp = ((float)raw / MPU6050_TEMP_SENSITIVITY) + MPU6050_TEMP_OFFSET_DEGC;
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_read_all(const mpu6050* const dev, mpu6050_data* const data)
{
    if (!dev || !data)
    {
        return false;
    }

    mpu6050_raw_data raw;
    if (!mpu6050_read_raw_all(dev, &raw))
    {
        return false;
    }
    return mpu6050_convert(dev, &raw, data);
}

//-----------------------------------------------------------------------------
bool mpu6050_convert(const mpu6050* const dev,
                     const mpu6050_raw_data* const raw,
                     mpu6050_data* const data)
{
    if (!dev || !raw || !data)
    {
        return false;
    }

    const float accel_sens = mpu6050_get_accel_sensitivity(dev);
    const float gyro_sens = mpu6050_get_gyro_sensitivity(dev);
    if (accel_sens == 0.0f || gyro_sens == 0.0f)
    {
        return false;
    }

    data->accel.x = (float)raw->accel.x / accel_sens;
    data->accel.y = (float)raw->accel.y / accel_sens;
    data->accel.z = (float)raw->accel.z / accel_sens;
    data->gyro.x = (float)raw->gyro.x / gyro_sens;
    data->gyro.y = (float)raw->gyro.y / gyro_sens;
    data->gyro.z = (float)raw->gyro.z / gyro_sens;
    data->temperature =
        ((float)raw->temperature / MPU6050_TEMP_SENSITIVITY) +
        MPU6050_TEMP_OFFSET_DEGC;
    return true;
}

//-----------------------------------------------------------------------------
float mpu6050_get_accel_sensitivity(const mpu6050* const dev)
{
    if (!dev)
    {
        return 0.0f;
    }
    return mpu6050_accel_range_sensitivity(dev->accel_range);
}

//-----------------------------------------------------------------------------
float mpu6050_get_gyro_sensitivity(const mpu6050* const dev)
{
    if (!dev)
    {
        return 0.0f;
    }
    return mpu6050_gyro_range_sensitivity(dev->gyro_range);
}

//-----------------------------------------------------------------------------
bool mpu6050_set_fifo_sources(const mpu6050* const dev, uint8_t sources)
{
    if (!dev)
    {
        return false;
    }
    return mpu6050_write_reg(dev, MPU6050_REG_FIFO_EN, sources);
}

//-----------------------------------------------------------------------------
bool mpu6050_get_fifo_sources(const mpu6050* const dev, uint8_t* sources)
{
    if (!dev || !sources)
    {
        return false;
    }
    return mpu6050_read_reg(dev, MPU6050_REG_FIFO_EN, sources);
}

//-----------------------------------------------------------------------------
bool mpu6050_set_fifo_enabled(const mpu6050* const dev, bool enable)
{
    if (!dev)
    {
        return false;
    }
    return mpu6050_update_reg(dev, MPU6050_REG_USER_CTRL,
                              MPU6050_USER_CTRL_FIFO_EN,
                              enable ? MPU6050_USER_CTRL_FIFO_EN : 0u);
}

//-----------------------------------------------------------------------------
bool mpu6050_reset_fifo(const mpu6050* const dev)
{
    if (!dev)
    {
        return false;
    }

    uint8_t cur = 0;
    if (!mpu6050_read_reg(dev, MPU6050_REG_USER_CTRL, &cur))
    {
        return false;
    }

    // The FIFO must be disabled while its pointers are being reset
    const uint8_t stopped = (uint8_t)(cur & (uint8_t)(~MPU6050_USER_CTRL_FIFO_EN));
    if (!mpu6050_write_reg(dev, MPU6050_REG_USER_CTRL, stopped) ||
        !mpu6050_write_reg(dev, MPU6050_REG_USER_CTRL,
                           (uint8_t)(stopped | MPU6050_USER_CTRL_FIFO_RST)))
    {
        return false;
    }
    return mpu6050_write_reg(dev, MPU6050_REG_USER_CTRL, cur);
}

//-----------------------------------------------------------------------------
bool mpu6050_get_fifo_count(const mpu6050* const dev, uint16_t* count)
{
    if (!dev || !count)
    {
        return false;
    }

    uint8_t buf[2] = {0};
    if (!mpu6050_read(dev, MPU6050_REG_FIFO_COUNT_H, buf, sizeof(buf)))
    {
        return false;
    }

    const uint16_t raw = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    *count = (raw > MPU6050_FIFO_SIZE) ? (uint16_t)MPU6050_FIFO_SIZE : raw;
    return true;
}

//-----------------------------------------------------------------------------
bool mpu6050_read_fifo(const mpu6050* const dev, uint8_t* buf, size_t buf_size)
{
    if (!dev || !buf || buf_size == 0 || buf_size > MPU6050_FIFO_SIZE)
    {
        return false;
    }

    uint16_t count = 0;
    if (!mpu6050_get_fifo_count(dev, &count))
    {
        return false;
    }
    if ((size_t)count < buf_size)
    {
        return false;
    }

    return mpu6050_read(dev, MPU6050_REG_FIFO_R_W, buf, buf_size);
}

//-----------------------------------------------------------------------------
bool mpu6050_set_int_config(const mpu6050* const dev,
                            const mpu6050_int_config* const cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    uint8_t pin_cfg = 0;
    if (cfg->active_low)
    {
        pin_cfg |= MPU6050_INT_PIN_ACTIVE_LOW;
    }
    if (cfg->open_drain)
    {
        pin_cfg |= MPU6050_INT_PIN_OPEN_DRAIN;
    }
    if (cfg->latch_enabled)
    {
        pin_cfg |= MPU6050_INT_PIN_LATCH_EN;
    }
    if (cfg->clear_on_any_read)
    {
        pin_cfg |= MPU6050_INT_PIN_RD_CLEAR;
    }

    const uint8_t pin_mask =
        (uint8_t)(MPU6050_INT_PIN_ACTIVE_LOW | MPU6050_INT_PIN_OPEN_DRAIN |
                  MPU6050_INT_PIN_LATCH_EN | MPU6050_INT_PIN_RD_CLEAR);
    if (!mpu6050_update_reg(dev, MPU6050_REG_INT_PIN_CFG, pin_mask, pin_cfg))
    {
        return false;
    }

    return mpu6050_write_reg(dev, MPU6050_REG_INT_ENABLE,
                             (uint8_t)(cfg->enabled_sources &
                                       MPU6050_INT_SOURCE_MASK));
}

//-----------------------------------------------------------------------------
bool mpu6050_get_int_status(const mpu6050* const dev, uint8_t* status)
{
    if (!dev || !status)
    {
        return false;
    }
    return mpu6050_read_reg(dev, MPU6050_REG_INT_STATUS, status);
}

//-----------------------------------------------------------------------------
bool mpu6050_set_motion_detection(const mpu6050* const dev, uint8_t threshold,
                                  uint8_t duration)
{
    if (!dev)
    {
        return false;
    }

    if (!mpu6050_write_reg(dev, MPU6050_REG_MOT_THR, threshold) ||
        !mpu6050_write_reg(dev, MPU6050_REG_MOT_DUR, duration))
    {
        return false;
    }

    // Motion detection needs the accelerometer power on delay to be non zero
    return mpu6050_write_reg(dev, MPU6050_REG_MOT_DETECT_CTRL,
                             MPU6050_MOT_CTRL_ACCEL_DELAY_3MS);
}

//-----------------------------------------------------------------------------
static float mpu6050_gyro_factory_trim(uint8_t code)
{
    if (code == 0)
    {
        return 0.0f;
    }

    // Equivalent to 25 * 131 * 1.046^(code-1), code is a 5 bit field
    float ft = MPU6050_GYRO_FT_BASE;
    for (uint8_t i = 1; i < code; ++i)
    {
        ft *= MPU6050_GYRO_FT_STEP;
    }
    return ft;
}

//-----------------------------------------------------------------------------
static float mpu6050_accel_factory_trim(uint8_t code)
{
    if (code == 0)
    {
        return 0.0f;
    }

    // Equivalent to 4096 * 0.34 * (0.92/0.34)^((code-1)/30), code is 5 bits
    float ft = MPU6050_ACCEL_FT_BASE;
    for (uint8_t i = 1; i < code; ++i)
    {
        ft *= MPU6050_ACCEL_FT_STEP;
    }
    return MPU6050_ACCEL_FT_SCALE * ft;
}

//-----------------------------------------------------------------------------
static float mpu6050_trim_deviation(float response, float factory_trim)
{
    if (factory_trim == 0.0f)
    {
        // No factory trim programmed for this axis, nothing to compare against
        return 0.0f;
    }
    return 100.0f * (response - factory_trim) / factory_trim;
}

//-----------------------------------------------------------------------------
static bool mpu6050_self_test_average(const mpu6050* const dev,
                                      mpu6050_axes* const accel,
                                      mpu6050_axes* const gyro)
{
    int32_t accel_sum[3] = {0, 0, 0};
    int32_t gyro_sum[3] = {0, 0, 0};

    for (uint8_t i = 0; i < MPU6050_SELF_TEST_SAMPLES; ++i)
    {
        mpu6050_raw_data raw;
        if (!mpu6050_read_raw_all(dev, &raw))
        {
            return false;
        }

        accel_sum[0] += raw.accel.x;
        accel_sum[1] += raw.accel.y;
        accel_sum[2] += raw.accel.z;
        gyro_sum[0] += raw.gyro.x;
        gyro_sum[1] += raw.gyro.y;
        gyro_sum[2] += raw.gyro.z;

        mpu6050_delay_ms(MPU6050_SELF_TEST_PERIOD_MS);
    }

    const float samples = (float)MPU6050_SELF_TEST_SAMPLES;
    accel->x = (float)accel_sum[0] / samples;
    accel->y = (float)accel_sum[1] / samples;
    accel->z = (float)accel_sum[2] / samples;
    gyro->x = (float)gyro_sum[0] / samples;
    gyro->y = (float)gyro_sum[1] / samples;
    gyro->z = (float)gyro_sum[2] / samples;
    return true;
}

//-----------------------------------------------------------------------------
static bool mpu6050_axis_passed(float deviation)
{
    const float abs_dev = (deviation < 0.0f) ? -deviation : deviation;
    return abs_dev <= MPU6050_SELF_TEST_MAX_DEV;
}

//-----------------------------------------------------------------------------
bool mpu6050_self_test(mpu6050* const dev,
                       mpu6050_self_test_result* const result)
{
    if (!dev || !result)
    {
        return false;
    }

    // The factory trim values are only valid for +/-250 dps and +/-8 g
    if (!mpu6050_set_gyro_range(dev, MPU6050_GYRO_RANGE_250DPS) ||
        !mpu6050_set_accel_range(dev, MPU6050_ACCEL_RANGE_8G))
    {
        return false;
    }
    mpu6050_delay_ms(MPU6050_SELF_TEST_SETTLE_MS);

    mpu6050_axes accel_off, gyro_off;
    if (!mpu6050_self_test_average(dev, &accel_off, &gyro_off))
    {
        return false;
    }

    // Drive all six axes with their self-test actuators
    const uint8_t gyro_st = (uint8_t)(MPU6050_CFG_SELF_TEST_MASK |
                                      ((uint8_t)MPU6050_GYRO_RANGE_250DPS
                                       << MPU6050_CFG_FS_SHIFT));
    const uint8_t accel_st = (uint8_t)(MPU6050_CFG_SELF_TEST_MASK |
                                       ((uint8_t)MPU6050_ACCEL_RANGE_8G
                                        << MPU6050_CFG_FS_SHIFT));
    if (!mpu6050_write_reg(dev, MPU6050_REG_GYRO_CONFIG, gyro_st) ||
        !mpu6050_write_reg(dev, MPU6050_REG_ACCEL_CONFIG, accel_st))
    {
        return false;
    }
    mpu6050_delay_ms(MPU6050_SELF_TEST_SETTLE_MS);

    mpu6050_axes accel_on, gyro_on;
    const bool measured = mpu6050_self_test_average(dev, &accel_on, &gyro_on);

    // Always switch the actuators off again, even when a measurement failed
    const bool restored =
        mpu6050_set_gyro_range(dev, MPU6050_GYRO_RANGE_250DPS) &&
        mpu6050_set_accel_range(dev, MPU6050_ACCEL_RANGE_8G) &&
        mpu6050_update_reg(dev, MPU6050_REG_GYRO_CONFIG,
                           MPU6050_CFG_SELF_TEST_MASK, 0u) &&
        mpu6050_update_reg(dev, MPU6050_REG_ACCEL_CONFIG,
                           MPU6050_CFG_SELF_TEST_MASK, 0u);
    if (!measured || !restored)
    {
        return false;
    }

    uint8_t trim[4] = {0};
    if (!mpu6050_read(dev, MPU6050_REG_SELF_TEST_X, trim, sizeof(trim)))
    {
        return false;
    }

    const uint8_t xg_code = (uint8_t)(trim[0] & 0x1Fu);
    const uint8_t yg_code = (uint8_t)(trim[1] & 0x1Fu);
    const uint8_t zg_code = (uint8_t)(trim[2] & 0x1Fu);
    const uint8_t xa_code =
        (uint8_t)(((trim[0] & 0xE0u) >> 3) | ((trim[3] & 0x30u) >> 4));
    const uint8_t ya_code =
        (uint8_t)(((trim[1] & 0xE0u) >> 3) | ((trim[3] & 0x0Cu) >> 2));
    const uint8_t za_code =
        (uint8_t)(((trim[2] & 0xE0u) >> 3) | (trim[3] & 0x03u));

    // The Y gyroscope factory trim is negative by definition
    result->gyro_deviation.x = mpu6050_trim_deviation(
        gyro_on.x - gyro_off.x, mpu6050_gyro_factory_trim(xg_code));
    result->gyro_deviation.y = mpu6050_trim_deviation(
        gyro_on.y - gyro_off.y, -mpu6050_gyro_factory_trim(yg_code));
    result->gyro_deviation.z = mpu6050_trim_deviation(
        gyro_on.z - gyro_off.z, mpu6050_gyro_factory_trim(zg_code));

    result->accel_deviation.x = mpu6050_trim_deviation(
        accel_on.x - accel_off.x, mpu6050_accel_factory_trim(xa_code));
    result->accel_deviation.y = mpu6050_trim_deviation(
        accel_on.y - accel_off.y, mpu6050_accel_factory_trim(ya_code));
    result->accel_deviation.z = mpu6050_trim_deviation(
        accel_on.z - accel_off.z, mpu6050_accel_factory_trim(za_code));

    result->gyro_passed = mpu6050_axis_passed(result->gyro_deviation.x) &&
                          mpu6050_axis_passed(result->gyro_deviation.y) &&
                          mpu6050_axis_passed(result->gyro_deviation.z);
    result->accel_passed = mpu6050_axis_passed(result->accel_deviation.x) &&
                           mpu6050_axis_passed(result->accel_deviation.y) &&
                           mpu6050_axis_passed(result->accel_deviation.z);
    return true;
}

//-----------------------------------------------------------------------------
