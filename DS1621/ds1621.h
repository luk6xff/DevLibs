/**
 *  @brief:   DS1621 digital thermometer and thermostat library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __DS1621_H__
#define __DS1621_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief I2C slave addresses (7-bit, unshifted).
 *
 * The three address pins A2..A0 are appended to the fixed 1001b prefix, so a
 * device answers somewhere between 0x48 (all pins to GND) and 0x4F (all pins
 * to VDD).
 */
#define DS1621_I2C_ADDR_BASE        0x48
#define DS1621_I2C_ADDR(a2, a1, a0) ((uint8_t)(DS1621_I2C_ADDR_BASE      | \
                                               (((a2) & 1u) << 2)        | \
                                               (((a1) & 1u) << 1)        | \
                                               ((a0) & 1u)))

/** Value reported by ds1621_get_temperature() when no valid reading exists */
#define DS1621_INVALID_TEMPERATURE  (-999.0f)

/** Worst case 9-bit conversion time in milliseconds */
#define DS1621_CONVERSION_TIME_MS   1000

/** Temperature range covered by the 9-bit two's complement format */
#define DS1621_TEMPERATURE_MIN      (-55.0f)
#define DS1621_TEMPERATURE_MAX      (125.0f)

/**
 * @brief Device command set.
 *
 * The DS1621 has no register map, every access starts with a command byte.
 */
typedef enum
{
    DS1621_CMD_READ_TEMPERATURE = 0xAA, // Read the last result (2 bytes)
    DS1621_CMD_ACCESS_TH        = 0xA1, // Read/write the high trip point (2 bytes)
    DS1621_CMD_ACCESS_TL        = 0xA2, // Read/write the low trip point (2 bytes)
    DS1621_CMD_ACCESS_CONFIG    = 0xAC, // Read/write the config/status register (1 byte)
    DS1621_CMD_READ_COUNTER     = 0xA8, // Read the remaining slope counter (1 byte)
    DS1621_CMD_READ_SLOPE       = 0xA9, // Read the counts per degree C (1 byte)
    DS1621_CMD_START_CONVERT    = 0xEE, // Begin a conversion, no payload
    DS1621_CMD_STOP_CONVERT     = 0x22, // Abort a continuous conversion, no payload
} ds1621_cmd;

/**
 * @brief Configuration/status register bit masks
 */
#define DS1621_STATUS_DONE      0x80u // Conversion complete
#define DS1621_STATUS_THF       0x40u // Temperature has been at or above TH
#define DS1621_STATUS_TLF       0x20u // Temperature has been at or below TL
#define DS1621_STATUS_NVB       0x10u // Non-volatile memory write in progress
#define DS1621_CFG_POL          0x02u // TOUT polarity
#define DS1621_CFG_ONE_SHOT     0x01u // One-shot conversion mode

/**
 * @brief Conversion mode
 */
typedef enum
{
    DS1621_CONVERSION_CONTINUOUS = 0, // Conversions run back to back once started
    DS1621_CONVERSION_ONE_SHOT   = 1, // A single conversion per start command
} ds1621_conversion_mode;

/**
 * @brief TOUT (thermostat output) active level
 */
typedef enum
{
    DS1621_TOUT_ACTIVE_LOW  = 0,
    DS1621_TOUT_ACTIVE_HIGH = 1,
} ds1621_tout_polarity;

/**
 * @brief Device configuration
 */
typedef struct
{
    ds1621_conversion_mode conversion_mode;
    ds1621_tout_polarity   tout_polarity;
} ds1621_config;

/**
 * @brief Latched device status
 */
typedef struct
{
    bool conversion_done; // A conversion result is available
    bool temp_high_flag;  // TH was reached since the flag was last cleared
    bool temp_low_flag;   // TL was reached since the flag was last cleared
    bool eeprom_busy;     // A non-volatile write is still in progress
} ds1621_status;

/**
 * @brief ds1621 dev object
 */
typedef struct
{
    uint8_t i2c_addr;   // 7-bit I2C slave address
    void* platform_dev; // Platform descriptor, set by ds1621_<platform>_init()
    float temperature;  // Last successful reading in degC
} ds1621;

/**
 * @brief Initialize the device and apply the default configuration.
 *
 * Default: one-shot conversions, TOUT active high.
 *
 * @param dev ds1621 device object
 * @return True on success, false otherwise
 */
bool ds1621_init(ds1621* const dev);

/**
 * @brief Apply a device configuration.
 *
 * @note The configuration register is backed by EEPROM, the call blocks until
 *       the non-volatile write completes.
 *
 * @param dev ds1621 device object
 * @param cfg Configuration to be applied
 * @return True on success, false otherwise
 */
bool ds1621_set_config(const ds1621* const dev, const ds1621_config* const cfg);

/**
 * @brief Read back the current device configuration.
 *
 * @param      dev ds1621 device object
 * @param[out] cfg Configuration read from the device
 * @return True on success, false otherwise
 */
bool ds1621_get_config(const ds1621* const dev, ds1621_config* const cfg);

/**
 * @brief Read the configuration/status register flags.
 *
 * @param      dev    ds1621 device object
 * @param[out] status Status read from the device
 * @return True on success, false otherwise
 */
bool ds1621_read_status(const ds1621* const dev, ds1621_status* const status);

/**
 * @brief Clear the THF and TLF thermostat flags, leaving the rest intact.
 *
 * @param dev ds1621 device object
 * @return True on success, false otherwise
 */
bool ds1621_clear_alarm_flags(const ds1621* const dev);

/**
 * @brief Start a temperature conversion.
 *
 * @param dev ds1621 device object
 * @return True on success, false otherwise
 */
bool ds1621_start_conversion(const ds1621* const dev);

/**
 * @brief Stop a running continuous conversion.
 *
 * @param dev ds1621 device object
 * @return True on success, false otherwise
 */
bool ds1621_stop_conversion(const ds1621* const dev);

/**
 * @brief Read the last conversion result with the native 0.5 degC resolution.
 *
 * On success the value is also cached and can be retrieved later with
 * ds1621_get_temperature().
 *
 * @param      dev  ds1621 device object
 * @param[out] temp Temperature in degrees Celsius, may be NULL
 * @return True on success, false otherwise
 */
bool ds1621_read_temperature(ds1621* const dev, float* temp);

/**
 * @brief Read the last conversion result using the counter/slope algorithm.
 *
 * Resolution is improved to roughly 0.01 degC:
 * T = TRUNC(T_read) - 0.25 + (slope - counter) / slope
 *
 * @param      dev  ds1621 device object
 * @param[out] temp Temperature in degrees Celsius, may be NULL
 * @return True on success, false otherwise
 */
bool ds1621_read_temperature_hires(ds1621* const dev, float* temp);

/**
 * @brief Start a conversion, wait for it to finish and read the result.
 *
 * @param      dev   ds1621 device object
 * @param      hires True to use the counter/slope algorithm
 * @param[out] temp  Temperature in degrees Celsius, may be NULL
 * @return True on success, false otherwise
 */
bool ds1621_measure_temperature(ds1621* const dev, bool hires, float* temp);

/**
 * @brief Return the temperature cached by the last successful read.
 *
 * @param dev ds1621 device object
 * @return Temperature in degrees Celsius or DS1621_INVALID_TEMPERATURE
 */
float ds1621_get_temperature(const ds1621* const dev);

/**
 * @brief Program the thermostat trip points.
 *
 * TOUT is asserted once the temperature reaches th and is released when it
 * drops back to tl. Both values are rounded to the nearest 0.5 degC.
 *
 * @note Both trip points are backed by EEPROM, the call blocks until the
 *       non-volatile writes complete.
 *
 * @param dev ds1621 device object
 * @param th  High trip point in degrees Celsius
 * @param tl  Low trip point in degrees Celsius
 * @return True on success, false otherwise
 */
bool ds1621_set_thermostat(const ds1621* const dev, float th, float tl);

/**
 * @brief Read back the thermostat trip points.
 *
 * @param      dev ds1621 device object
 * @param[out] th  High trip point in degrees Celsius, may be NULL
 * @param[out] tl  Low trip point in degrees Celsius, may be NULL
 * @return True on success, false otherwise
 */
bool ds1621_get_thermostat(const ds1621* const dev, float* th, float* tl);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Write a command byte followed by an optional payload
 *
 * @param dev      ds1621 device object
 * @param cmd      Command byte
 * @param buf      Data to be written, may be NULL when buf_size is 0
 * @param buf_size Number of bytes to be written
 * @return True on success, false otherwise
 */
extern bool ds1621_write(const ds1621* const dev, uint8_t cmd,
                         const uint8_t* buf, size_t buf_size);

/**
 * @brief Write a command byte and read the answer back
 *
 * @param      dev      ds1621 device object
 * @param      cmd      Command byte
 * @param[out] buf      Buffer to fill with read bytes
 * @param      buf_size Number of bytes to read
 * @return True on success, false otherwise
 */
extern bool ds1621_read(const ds1621* const dev, uint8_t cmd,
                        uint8_t* buf, size_t buf_size);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void ds1621_delay_ms(uint32_t delay_ms);


#ifdef __cplusplus
}
#endif

#endif /* __DS1621_H__ */
