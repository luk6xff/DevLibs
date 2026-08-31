/**
 *  @brief:   MPU6050 6-axis motion tracking device (gyroscope + accelerometer) library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MPU6050_H__
#define __MPU6050_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief I2C slave addresses (7-bit, unshifted), selected with the AD0 pin
 */
#define MPU6050_I2C_ADDR_AD0_LOW    0x68  // AD0 pin connected to GND
#define MPU6050_I2C_ADDR_AD0_HIGH   0x69  // AD0 pin connected to VDD

/** Value returned by the WHO_AM_I register, identical for both addresses */
#define MPU6050_WHO_AM_I_VALUE      0x68

/** Size of the internal FIFO in bytes */
#define MPU6050_FIFO_SIZE           1024

/** Number of bytes returned by a burst read starting at ACCEL_XOUT_H */
#define MPU6050_BURST_LEN           14

/**
 * @brief Device register map
 */
typedef enum
{
    MPU6050_REG_SELF_TEST_X       = 0x0D,
    MPU6050_REG_SELF_TEST_Y       = 0x0E,
    MPU6050_REG_SELF_TEST_Z       = 0x0F,
    MPU6050_REG_SELF_TEST_A       = 0x10,
    MPU6050_REG_SMPLRT_DIV        = 0x19,
    MPU6050_REG_CONFIG            = 0x1A,
    MPU6050_REG_GYRO_CONFIG       = 0x1B,
    MPU6050_REG_ACCEL_CONFIG      = 0x1C,
    MPU6050_REG_MOT_THR           = 0x1F,
    MPU6050_REG_MOT_DUR           = 0x20,
    MPU6050_REG_FIFO_EN           = 0x23,
    MPU6050_REG_I2C_MST_CTRL      = 0x24,
    MPU6050_REG_INT_PIN_CFG       = 0x37,
    MPU6050_REG_INT_ENABLE        = 0x38,
    MPU6050_REG_INT_STATUS        = 0x3A,
    MPU6050_REG_ACCEL_XOUT_H      = 0x3B,
    MPU6050_REG_ACCEL_XOUT_L      = 0x3C,
    MPU6050_REG_ACCEL_YOUT_H      = 0x3D,
    MPU6050_REG_ACCEL_YOUT_L      = 0x3E,
    MPU6050_REG_ACCEL_ZOUT_H      = 0x3F,
    MPU6050_REG_ACCEL_ZOUT_L      = 0x40,
    MPU6050_REG_TEMP_OUT_H        = 0x41,
    MPU6050_REG_TEMP_OUT_L        = 0x42,
    MPU6050_REG_GYRO_XOUT_H       = 0x43,
    MPU6050_REG_GYRO_XOUT_L       = 0x44,
    MPU6050_REG_GYRO_YOUT_H       = 0x45,
    MPU6050_REG_GYRO_YOUT_L       = 0x46,
    MPU6050_REG_GYRO_ZOUT_H       = 0x47,
    MPU6050_REG_GYRO_ZOUT_L       = 0x48,
    MPU6050_REG_MOT_DETECT_STATUS = 0x61,
    MPU6050_REG_SIGNAL_PATH_RESET = 0x68,
    MPU6050_REG_MOT_DETECT_CTRL   = 0x69,
    MPU6050_REG_USER_CTRL         = 0x6A,
    MPU6050_REG_PWR_MGMT_1        = 0x6B,
    MPU6050_REG_PWR_MGMT_2        = 0x6C,
    MPU6050_REG_FIFO_COUNT_H      = 0x72,
    MPU6050_REG_FIFO_COUNT_L      = 0x73,
    MPU6050_REG_FIFO_R_W          = 0x74,
    MPU6050_REG_WHO_AM_I          = 0x75,
} mpu6050_reg;

/**
 * @brief Digital low pass filter setting (DLPF_CFG in the CONFIG register).
 *
 * The enumerator names follow the gyroscope bandwidth. Settings other than
 * MPU6050_DLPF_BW_256HZ drop the gyroscope output rate from 8 kHz to 1 kHz,
 * which is the base rate the sample rate divider is applied to.
 */
typedef enum
{
    MPU6050_DLPF_BW_256HZ = 0, // accel 260 Hz, gyro output rate 8 kHz
    MPU6050_DLPF_BW_188HZ = 1,
    MPU6050_DLPF_BW_98HZ  = 2,
    MPU6050_DLPF_BW_42HZ  = 3,
    MPU6050_DLPF_BW_20HZ  = 4,
    MPU6050_DLPF_BW_10HZ  = 5,
    MPU6050_DLPF_BW_5HZ   = 6,
} mpu6050_dlpf;

/**
 * @brief Gyroscope full-scale range (FS_SEL in the GYRO_CONFIG register)
 */
typedef enum
{
    MPU6050_GYRO_RANGE_250DPS  = 0,
    MPU6050_GYRO_RANGE_500DPS  = 1,
    MPU6050_GYRO_RANGE_1000DPS = 2,
    MPU6050_GYRO_RANGE_2000DPS = 3,
} mpu6050_gyro_range;

/**
 * @brief Accelerometer full-scale range (AFS_SEL in the ACCEL_CONFIG register)
 */
typedef enum
{
    MPU6050_ACCEL_RANGE_2G  = 0,
    MPU6050_ACCEL_RANGE_4G  = 1,
    MPU6050_ACCEL_RANGE_8G  = 2,
    MPU6050_ACCEL_RANGE_16G = 3,
} mpu6050_accel_range;

/**
 * @brief Clock source (CLKSEL in the PWR_MGMT_1 register)
 */
typedef enum
{
    MPU6050_CLOCK_INTERNAL_8MHZ = 0,
    MPU6050_CLOCK_PLL_X_GYRO    = 1,
    MPU6050_CLOCK_PLL_Y_GYRO    = 2,
    MPU6050_CLOCK_PLL_Z_GYRO    = 3,
    MPU6050_CLOCK_PLL_EXT_32KHZ = 4,
    MPU6050_CLOCK_PLL_EXT_19MHZ = 5,
    MPU6050_CLOCK_STOP          = 7,
} mpu6050_clock_source;

/**
 * @brief Sources that can be pushed into the FIFO (FIFO_EN register bits)
 */
typedef enum
{
    MPU6050_FIFO_SRC_SLV0  = 0x01,
    MPU6050_FIFO_SRC_SLV1  = 0x02,
    MPU6050_FIFO_SRC_SLV2  = 0x04,
    MPU6050_FIFO_SRC_ACCEL = 0x08, // pushes all 6 accelerometer bytes
    MPU6050_FIFO_SRC_ZGYRO = 0x10,
    MPU6050_FIFO_SRC_YGYRO = 0x20,
    MPU6050_FIFO_SRC_XGYRO = 0x40,
    MPU6050_FIFO_SRC_TEMP  = 0x80,
    MPU6050_FIFO_SRC_GYRO  = 0x70, // all three gyroscope axes
} mpu6050_fifo_source;

/**
 * @brief Interrupt sources (INT_ENABLE / INT_STATUS register bits)
 */
typedef enum
{
    MPU6050_INT_DATA_RDY    = 0x01,
    MPU6050_INT_I2C_MST     = 0x08,
    MPU6050_INT_FIFO_OFLOW  = 0x10,
    MPU6050_INT_MOTION      = 0x40,
} mpu6050_int_source;

/**
 * @brief Device configuration
 */
typedef struct
{
    uint8_t sample_rate_div;           // SMPLRT_DIV, sample rate = base rate / (1 + div)
    mpu6050_dlpf dlpf;                 // Digital low pass filter setting
    mpu6050_gyro_range gyro_range;     // Gyroscope full-scale range
    mpu6050_accel_range accel_range;   // Accelerometer full-scale range
    mpu6050_clock_source clock_source; // Clock source
} mpu6050_config;

/**
 * @brief Interrupt pin and interrupt source configuration
 */
typedef struct
{
    bool active_low;          // true: INT pin is active low
    bool open_drain;          // true: INT pin is open drain, false: push-pull
    bool latch_enabled;       // true: INT pin stays asserted until cleared
    bool clear_on_any_read;   // true: any register read clears the interrupt
    uint8_t enabled_sources;  // Bitmask built from mpu6050_int_source values
} mpu6050_int_config;

/**
 * @brief Raw, unscaled three axis sample in hardware units
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} mpu6050_axes_raw;

/**
 * @brief Scaled three axis sample
 */
typedef struct
{
    float x;
    float y;
    float z;
} mpu6050_axes;

/**
 * @brief All sensor outputs in hardware units, filled by a single burst read
 */
typedef struct
{
    mpu6050_axes_raw accel;
    int16_t temperature;
    mpu6050_axes_raw gyro;
} mpu6050_raw_data;

/**
 * @brief All sensor outputs converted to physical units
 */
typedef struct
{
    mpu6050_axes accel; // Acceleration in [g]
    float temperature;  // Die temperature in [degC]
    mpu6050_axes gyro;  // Angular rate in [dps]
} mpu6050_data;

/**
 * @brief Result of mpu6050_self_test()
 */
typedef struct
{
    // Deviation from the factory trim in percent, one entry per axis
    mpu6050_axes accel_deviation;
    mpu6050_axes gyro_deviation;
    bool accel_passed; // true if all three accelerometer axes are within tolerance
    bool gyro_passed;  // true if all three gyroscope axes are within tolerance
} mpu6050_self_test_result;

/**
 * @brief mpu6050 dev object
 */
typedef struct
{
    uint8_t i2c_addr;                // 7-bit I2C slave address
    void* platform_dev;              // Platform descriptor, set by mpu6050_<platform>_init()
    mpu6050_gyro_range gyro_range;   // Cached gyro range, maintained by the library
    mpu6050_accel_range accel_range; // Cached accel range, maintained by the library
} mpu6050;

/**
 * @brief Initialize the device and apply the default configuration.
 *
 * Resets the device, verifies the WHO_AM_I register, wakes the chip up and
 * applies: PLL with X gyroscope reference, DLPF 42 Hz, sample rate 100 Hz,
 * gyroscope +/-250 dps, accelerometer +/-2 g, FIFO and interrupts disabled.
 *
 * @param dev mpu6050 device object
 * @return True on success, false otherwise
 */
bool mpu6050_init(mpu6050* const dev);

/**
 * @brief Perform a device reset and wait until it completes.
 *
 * After the reset all registers hold their power-on values and the device is
 * asleep, so mpu6050_set_config() (or mpu6050_init()) has to be called again.
 *
 * @param dev mpu6050 device object
 * @return True on success, false otherwise
 */
bool mpu6050_reset(mpu6050* const dev);

/**
 * @brief Reset the analog and digital signal paths of all sensors.
 *
 * @param dev mpu6050 device object
 * @return True on success, false otherwise
 */
bool mpu6050_reset_signal_path(const mpu6050* const dev);

/**
 * @brief Read the WHO_AM_I register.
 *
 * @param      dev mpu6050 device object
 * @param[out] id  Register content, MPU6050_WHO_AM_I_VALUE for a genuine device
 * @return True on success, false otherwise
 */
bool mpu6050_read_id(const mpu6050* const dev, uint8_t* id);

/**
 * @brief Apply a full device configuration.
 *
 * @param dev mpu6050 device object
 * @param cfg Configuration to be applied
 * @return True on success, false otherwise
 */
bool mpu6050_set_config(mpu6050* const dev, const mpu6050_config* const cfg);

/**
 * @brief Read back the current device configuration.
 *
 * @param      dev mpu6050 device object
 * @param[out] cfg Configuration read from the device
 * @return True on success, false otherwise
 */
bool mpu6050_get_config(mpu6050* const dev, mpu6050_config* const cfg);

/**
 * @brief Set the sample rate divider.
 *
 * Sample rate = gyroscope output rate / (1 + div), where the gyroscope output
 * rate is 8 kHz for MPU6050_DLPF_BW_256HZ and 1 kHz for every other setting.
 *
 * @param dev mpu6050 device object
 * @param div Sample rate divider
 * @return True on success, false otherwise
 */
bool mpu6050_set_sample_rate_divider(const mpu6050* const dev, uint8_t div);

/**
 * @brief Read the sample rate divider.
 *
 * @param      dev mpu6050 device object
 * @param[out] div Sample rate divider
 * @return True on success, false otherwise
 */
bool mpu6050_get_sample_rate_divider(const mpu6050* const dev, uint8_t* div);

/**
 * @brief Compute the currently configured sample rate.
 *
 * @param      dev  mpu6050 device object
 * @param[out] rate Sample rate in [Hz]
 * @return True on success, false otherwise
 */
bool mpu6050_get_sample_rate(const mpu6050* const dev, uint16_t* rate);

/**
 * @brief Set the digital low pass filter.
 *
 * @param dev  mpu6050 device object
 * @param dlpf Filter setting to be applied
 * @return True on success, false otherwise
 */
bool mpu6050_set_dlpf(const mpu6050* const dev, mpu6050_dlpf dlpf);

/**
 * @brief Read the digital low pass filter setting.
 *
 * @param      dev  mpu6050 device object
 * @param[out] dlpf Current filter setting
 * @return True on success, false otherwise
 */
bool mpu6050_get_dlpf(const mpu6050* const dev, mpu6050_dlpf* dlpf);

/**
 * @brief Set the gyroscope full-scale range.
 *
 * @param dev   mpu6050 device object
 * @param range Range to be applied
 * @return True on success, false otherwise
 */
bool mpu6050_set_gyro_range(mpu6050* const dev, mpu6050_gyro_range range);

/**
 * @brief Read the gyroscope full-scale range from the device.
 *
 * The cached value inside `dev` is refreshed as well.
 *
 * @param      dev   mpu6050 device object
 * @param[out] range Current range, may be NULL
 * @return True on success, false otherwise
 */
bool mpu6050_get_gyro_range(mpu6050* const dev, mpu6050_gyro_range* range);

/**
 * @brief Set the accelerometer full-scale range.
 *
 * @param dev   mpu6050 device object
 * @param range Range to be applied
 * @return True on success, false otherwise
 */
bool mpu6050_set_accel_range(mpu6050* const dev, mpu6050_accel_range range);

/**
 * @brief Read the accelerometer full-scale range from the device.
 *
 * The cached value inside `dev` is refreshed as well.
 *
 * @param      dev   mpu6050 device object
 * @param[out] range Current range, may be NULL
 * @return True on success, false otherwise
 */
bool mpu6050_get_accel_range(mpu6050* const dev, mpu6050_accel_range* range);

/**
 * @brief Select the device clock source.
 *
 * @param dev mpu6050 device object
 * @param src Clock source to be used
 * @return True on success, false otherwise
 */
bool mpu6050_set_clock_source(const mpu6050* const dev,
                              mpu6050_clock_source src);

/**
 * @brief Read the currently selected clock source.
 *
 * @param      dev mpu6050 device object
 * @param[out] src Current clock source
 * @return True on success, false otherwise
 */
bool mpu6050_get_clock_source(const mpu6050* const dev,
                              mpu6050_clock_source* src);

/**
 * @brief Put the device into sleep mode or wake it up.
 *
 * @param dev   mpu6050 device object
 * @param sleep True to sleep, false to wake up
 * @return True on success, false otherwise
 */
bool mpu6050_set_sleep(const mpu6050* const dev, bool sleep);

/**
 * @brief Check whether the device is in sleep mode.
 *
 * @param      dev   mpu6050 device object
 * @param[out] sleep True if the device is asleep
 * @return True on success, false otherwise
 */
bool mpu6050_get_sleep(const mpu6050* const dev, bool* sleep);

/**
 * @brief Put individual sensor axes into standby mode.
 *
 * @param dev         mpu6050 device object
 * @param accel_x     False to put the accelerometer X axis into standby
 * @param accel_y     False to put the accelerometer Y axis into standby
 * @param accel_z     False to put the accelerometer Z axis into standby
 * @param gyro_x      False to put the gyroscope X axis into standby
 * @param gyro_y      False to put the gyroscope Y axis into standby
 * @param gyro_z      False to put the gyroscope Z axis into standby
 * @return True on success, false otherwise
 */
bool mpu6050_set_standby(const mpu6050* const dev, bool accel_x, bool accel_y,
                         bool accel_z, bool gyro_x, bool gyro_y, bool gyro_z);

/**
 * @brief Read the raw accelerometer output.
 *
 * @param      dev   mpu6050 device object
 * @param[out] accel Raw accelerometer sample in hardware units
 * @return True on success, false otherwise
 */
bool mpu6050_read_raw_accel(const mpu6050* const dev,
                            mpu6050_axes_raw* const accel);

/**
 * @brief Read the raw gyroscope output.
 *
 * @param      dev  mpu6050 device object
 * @param[out] gyro Raw gyroscope sample in hardware units
 * @return True on success, false otherwise
 */
bool mpu6050_read_raw_gyro(const mpu6050* const dev,
                           mpu6050_axes_raw* const gyro);

/**
 * @brief Read the raw temperature output.
 *
 * @param      dev  mpu6050 device object
 * @param[out] temp Raw temperature sample in hardware units
 * @return True on success, false otherwise
 */
bool mpu6050_read_raw_temperature(const mpu6050* const dev, int16_t* temp);

/**
 * @brief Read accelerometer, temperature and gyroscope in a single burst.
 *
 * The burst starts at MPU6050_REG_ACCEL_XOUT_H so all values belong to the
 * very same sample.
 *
 * @param      dev  mpu6050 device object
 * @param[out] data Raw sensor outputs in hardware units
 * @return True on success, false otherwise
 */
bool mpu6050_read_raw_all(const mpu6050* const dev,
                          mpu6050_raw_data* const data);

/**
 * @brief Read the accelerometer output scaled to [g].
 *
 * @param      dev   mpu6050 device object
 * @param[out] accel Acceleration in [g]
 * @return True on success, false otherwise
 */
bool mpu6050_read_accel(const mpu6050* const dev, mpu6050_axes* const accel);

/**
 * @brief Read the gyroscope output scaled to [dps].
 *
 * @param      dev  mpu6050 device object
 * @param[out] gyro Angular rate in [dps]
 * @return True on success, false otherwise
 */
bool mpu6050_read_gyro(const mpu6050* const dev, mpu6050_axes* const gyro);

/**
 * @brief Read the die temperature in [degC].
 *
 * @param      dev  mpu6050 device object
 * @param[out] temp Temperature in degrees Celsius
 * @return True on success, false otherwise
 */
bool mpu6050_read_temperature(const mpu6050* const dev, float* temp);

/**
 * @brief Read all sensor outputs in a single burst and scale them.
 *
 * @param      dev  mpu6050 device object
 * @param[out] data Scaled sensor outputs
 * @return True on success, false otherwise
 */
bool mpu6050_read_all(const mpu6050* const dev, mpu6050_data* const data);

/**
 * @brief Convert a raw sample set into physical units.
 *
 * Uses the full-scale ranges cached in `dev`, no bus access is performed.
 *
 * @param      dev  mpu6050 device object
 * @param      raw  Raw sensor outputs
 * @param[out] data Scaled sensor outputs
 * @return True on success, false otherwise
 */
bool mpu6050_convert(const mpu6050* const dev,
                     const mpu6050_raw_data* const raw,
                     mpu6050_data* const data);

/**
 * @brief Accelerometer sensitivity for the currently configured range.
 *
 * @param dev mpu6050 device object
 * @return Sensitivity in [LSB/g], 0.0f if `dev` is NULL
 */
float mpu6050_get_accel_sensitivity(const mpu6050* const dev);

/**
 * @brief Gyroscope sensitivity for the currently configured range.
 *
 * @param dev mpu6050 device object
 * @return Sensitivity in [LSB/dps], 0.0f if `dev` is NULL
 */
float mpu6050_get_gyro_sensitivity(const mpu6050* const dev);

/**
 * @brief Select which sensors are pushed into the FIFO.
 *
 * @param dev     mpu6050 device object
 * @param sources Bitmask built from mpu6050_fifo_source values
 * @return True on success, false otherwise
 */
bool mpu6050_set_fifo_sources(const mpu6050* const dev, uint8_t sources);

/**
 * @brief Read back the FIFO source mask.
 *
 * @param      dev     mpu6050 device object
 * @param[out] sources Bitmask built from mpu6050_fifo_source values
 * @return True on success, false otherwise
 */
bool mpu6050_get_fifo_sources(const mpu6050* const dev, uint8_t* sources);

/**
 * @brief Enable or disable the FIFO buffer.
 *
 * @param dev    mpu6050 device object
 * @param enable True to enable the FIFO
 * @return True on success, false otherwise
 */
bool mpu6050_set_fifo_enabled(const mpu6050* const dev, bool enable);

/**
 * @brief Flush the FIFO and restart it if it was enabled.
 *
 * @param dev mpu6050 device object
 * @return True on success, false otherwise
 */
bool mpu6050_reset_fifo(const mpu6050* const dev);

/**
 * @brief Read the number of bytes currently stored in the FIFO.
 *
 * @param      dev   mpu6050 device object
 * @param[out] count Number of bytes available, at most MPU6050_FIFO_SIZE
 * @return True on success, false otherwise
 */
bool mpu6050_get_fifo_count(const mpu6050* const dev, uint16_t* count);

/**
 * @brief Read bytes out of the FIFO.
 *
 * Fails if fewer than `buf_size` bytes are available so a caller never gets a
 * partially filled buffer.
 *
 * @param      dev      mpu6050 device object
 * @param[out] buf      Buffer to fill
 * @param      buf_size Number of bytes to read
 * @return True on success, false otherwise
 */
bool mpu6050_read_fifo(const mpu6050* const dev, uint8_t* buf, size_t buf_size);

/**
 * @brief Configure the interrupt pin and the enabled interrupt sources.
 *
 * @param dev mpu6050 device object
 * @param cfg Interrupt configuration to be applied
 * @return True on success, false otherwise
 */
bool mpu6050_set_int_config(const mpu6050* const dev,
                            const mpu6050_int_config* const cfg);

/**
 * @brief Read the interrupt status register.
 *
 * Reading the register clears every latched interrupt.
 *
 * @param      dev    mpu6050 device object
 * @param[out] status Bitmask built from mpu6050_int_source values
 * @return True on success, false otherwise
 */
bool mpu6050_get_int_status(const mpu6050* const dev, uint8_t* status);

/**
 * @brief Configure the motion detection interrupt.
 *
 * @param dev       mpu6050 device object
 * @param threshold Motion threshold, 1 LSB = 32 mg
 * @param duration  Number of consecutive milliseconds above the threshold
 * @return True on success, false otherwise
 */
bool mpu6050_set_motion_detection(const mpu6050* const dev, uint8_t threshold,
                                  uint8_t duration);

/**
 * @brief Run the factory self-test procedure.
 *
 * Compares the self-test response of every axis against the factory trim value
 * stored in the SELF_TEST registers. An axis passes when the deviation stays
 * within +/-14%. The device configuration is left in the state required by the
 * self-test (gyro +/-250 dps, accel +/-8 g), so reconfigure it afterwards.
 *
 * @param      dev    mpu6050 device object
 * @param[out] result Per axis deviation in percent and the pass/fail flags
 * @return True when the test could be executed, false on a bus error
 */
bool mpu6050_self_test(mpu6050* const dev,
                       mpu6050_self_test_result* const result);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Write bytes into mpu6050 device
 *
 * @param dev      mpu6050 device object
 * @param reg_addr Register address
 * @param buf      Data to be written
 * @param buf_size Number of bytes to be written
 * @return True on success, false otherwise
 */
extern bool mpu6050_write(const mpu6050* const dev, uint8_t reg_addr,
                          const uint8_t* buf, size_t buf_size);

/**
 * @brief Read bytes from mpu6050 device
 *
 * @param      dev      mpu6050 device object
 * @param      reg_addr Register address
 * @param[out] buf      Buffer to fill with read bytes
 * @param      buf_size Number of bytes to read
 * @return True on success, false otherwise
 */
extern bool mpu6050_read(const mpu6050* const dev, uint8_t reg_addr,
                         uint8_t* buf, size_t buf_size);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void mpu6050_delay_ms(uint32_t delay_ms);


#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H__ */
