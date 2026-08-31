/**
 *  @brief:   MAX44009 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MAX44009_H__
#define __MAX44009_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief I2C slave addresses (7-bit, unshifted)
 */
#define MAX44009_I2C_ADDR_A0_GND    0x4A  // A0 pin connected to GND
#define MAX44009_I2C_ADDR_A0_VDD    0x4B  // A0 pin connected to VDD

/* Backwards compatible aliases */
#define MAX44009_I2C_ADDRESS1       MAX44009_I2C_ADDR_A0_GND
#define MAX44009_I2C_ADDRESS2       MAX44009_I2C_ADDR_A0_VDD

/**
 * @brief Device register map
 */
typedef enum
{
    MAX44009_REG_INT_STATUS = 0x00,
    MAX44009_REG_INT_ENABLE = 0x01,
    MAX44009_REG_CONFIG     = 0x02,
    MAX44009_REG_LUX_HIGH   = 0x03,
    MAX44009_REG_LUX_LOW    = 0x04,
    MAX44009_REG_THR_UPPER  = 0x05,
    MAX44009_REG_THR_LOWER  = 0x06,
    MAX44009_REG_THR_TIMER  = 0x07,
} max44009_reg;

/**
 * @brief ADC integration time, applied only in manual mode
 */
typedef enum
{
    MAX44009_INTEGRATION_TIME_800MS = 0x00,
    MAX44009_INTEGRATION_TIME_400MS = 0x01,
    MAX44009_INTEGRATION_TIME_200MS = 0x02,
    MAX44009_INTEGRATION_TIME_100MS = 0x03,
    MAX44009_INTEGRATION_TIME_50MS  = 0x04,
    MAX44009_INTEGRATION_TIME_25MS  = 0x05,
    MAX44009_INTEGRATION_TIME_12MS  = 0x06,
    MAX44009_INTEGRATION_TIME_6MS   = 0x07,
} max44009_integration_time;

/**
 * @brief Device configuration
 */
typedef struct
{
    bool continuous_mode;    // true: measure continuously, false: once per 800 ms
    bool manual_mode;        // true: honour current_div_ratio and integration_time
    bool current_div_ratio;  // true: only 1/8 of the photodiode current reaches the ADC
    max44009_integration_time integration_time;
} max44009_config;

/**
 * @brief max44009 dev object
 */
typedef struct
{
    uint8_t i2c_addr;   // 7-bit I2C slave address
    void* platform_dev; // Platform descriptor, assigned by max44009_<platform>_init()
} max44009;

/**
 * @brief Initialize the sensor and apply the default (automatic) configuration.
 *
 * @param dev max44009 device object
 * @return True on success, false otherwise
 */
bool max44009_init(max44009* const dev);

/**
 * @brief Apply a device configuration.
 *
 * @param dev max44009 device object
 * @param cfg Configuration to be applied
 * @return True on success, false otherwise
 */
bool max44009_set_config(const max44009* const dev,
                         const max44009_config* const cfg);

/**
 * @brief Read back the current device configuration.
 *
 * @param      dev max44009 device object
 * @param[out] cfg Configuration read from the device
 * @return True on success, false otherwise
 */
bool max44009_get_config(const max44009* const dev, max44009_config* const cfg);

/**
 * @brief Read ambient light luminance.
 *
 * @param      dev  max44009 device object
 * @param[out] lux  Ambient light luminance in lux [lx], set to -1.0f on failure
 * @return True on success, false otherwise
 */
bool max44009_get_lux(const max44009* const dev, float* lux);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Write bytes into max44009 device
 *
 * @param dev      max44009 device object
 * @param reg_addr Register address
 * @param buf      Data to be written
 * @param buf_size Number of bytes to be written
 * @return True on success, false otherwise
 */
extern bool max44009_write(const max44009* const dev, uint8_t reg_addr,
                           const uint8_t* buf, size_t buf_size);

/**
 * @brief Read bytes from max44009 device
 *
 * @param      dev      max44009 device object
 * @param      reg_addr Register address
 * @param[out] buf      Buffer to fill with read bytes
 * @param      buf_size Number of bytes to read
 * @return True on success, false otherwise
 */
extern bool max44009_read(const max44009* const dev, uint8_t reg_addr,
                          uint8_t* buf, size_t buf_size);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void max44009_delay_ms(uint32_t delay_ms);


#ifdef __cplusplus
}
#endif

#endif /* __MAX44009_H__ */

