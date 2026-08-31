/**
 *  @brief:   ADT7410 digital temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __ADT7410_H__
#define __ADT7410_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief I2C slave addresses (7-bit, unshifted), selected with the A1/A0 pins
 */
#define ADT7410_I2C_ADDR_A1_GND_A0_GND  0x48
#define ADT7410_I2C_ADDR_A1_GND_A0_VDD  0x49
#define ADT7410_I2C_ADDR_A1_VDD_A0_GND  0x4A
#define ADT7410_I2C_ADDR_A1_VDD_A0_VDD  0x4B

/** Value reported by adt7410_get_temperature() when no valid reading exists */
#define ADT7410_INVALID_TEMPERATURE     (-999.0f)

/** Manufacturer ID stored in the upper 5 bits of the ID register */
#define ADT7410_MANUFACTURER_ID         0x0C

/**
 * @brief Device register map
 */
typedef enum
{
    ADT7410_REG_TEMP_MSB   = 0x00,
    ADT7410_REG_TEMP_LSB   = 0x01,
    ADT7410_REG_STATUS     = 0x02,
    ADT7410_REG_CONFIG     = 0x03,
    ADT7410_REG_T_HIGH_MSB = 0x04,
    ADT7410_REG_T_HIGH_LSB = 0x05,
    ADT7410_REG_T_LOW_MSB  = 0x06,
    ADT7410_REG_T_LOW_LSB  = 0x07,
    ADT7410_REG_T_CRIT_MSB = 0x08,
    ADT7410_REG_T_CRIT_LSB = 0x09,
    ADT7410_REG_T_HYST     = 0x0A,
    ADT7410_REG_ID         = 0x0B,
    ADT7410_REG_RESET      = 0x2F,
} adt7410_reg;

/**
 * @brief ADC resolution
 */
typedef enum
{
    ADT7410_RESOLUTION_13_BIT = 0, // 0.0625 degC per LSB
    ADT7410_RESOLUTION_16_BIT = 1, // 0.0078 degC per LSB
} adt7410_resolution;

/**
 * @brief Operation mode
 */
typedef enum
{
    ADT7410_MODE_CONTINUOUS = 0,
    ADT7410_MODE_ONE_SHOT   = 1,
    ADT7410_MODE_1_SPS      = 2,
    ADT7410_MODE_SHUTDOWN   = 3,
} adt7410_operation_mode;

/**
 * @brief INT/CT pin behaviour
 */
typedef enum
{
    ADT7410_INT_CT_MODE_INTERRUPT  = 0,
    ADT7410_INT_CT_MODE_COMPARATOR = 1,
} adt7410_int_ct_mode;

/**
 * @brief Output pin polarity
 */
typedef enum
{
    ADT7410_PIN_ACTIVE_LOW  = 0,
    ADT7410_PIN_ACTIVE_HIGH = 1,
} adt7410_pin_polarity;

/**
 * @brief Number of out-of-limit measurements needed to assert INT/CT
 */
typedef enum
{
    ADT7410_FAULT_QUEUE_1 = 0,
    ADT7410_FAULT_QUEUE_2 = 1,
    ADT7410_FAULT_QUEUE_3 = 2,
    ADT7410_FAULT_QUEUE_4 = 3,
} adt7410_fault_queue;

/**
 * @brief Device configuration
 */
typedef struct
{
    adt7410_fault_queue     fault_queue;
    adt7410_pin_polarity    ct_pin_polarity;
    adt7410_pin_polarity    int_pin_polarity;
    adt7410_int_ct_mode     int_ct_mode;
    adt7410_operation_mode  operation_mode;
    adt7410_resolution      resolution;
} adt7410_config;

/**
 * @brief adt7410 dev object
 */
typedef struct
{
    uint8_t i2c_addr;              // 7-bit I2C slave address
    void* platform_dev;            // Platform descriptor, set by adt7410_<platform>_init()
    adt7410_resolution resolution; // Cached resolution, maintained by the library
    float temperature;             // Last successful reading in degC
} adt7410;

/**
 * @brief Initialize the sensor and apply the default configuration.
 *
 * Default: continuous conversion, 16-bit resolution, interrupt mode,
 * active low INT/CT pins, one fault needed to trigger.
 *
 * @param dev adt7410 device object
 * @return True on success, false otherwise
 */
bool adt7410_init(adt7410* const dev);

/**
 * @brief Apply a device configuration.
 *
 * @param dev adt7410 device object
 * @param cfg Configuration to be applied
 * @return True on success, false otherwise
 */
bool adt7410_set_config(adt7410* const dev, const adt7410_config* const cfg);

/**
 * @brief Change only the ADC resolution, leaving the rest of the config intact.
 *
 * @param dev adt7410 device object
 * @param res Resolution to be applied
 * @return True on success, false otherwise
 */
bool adt7410_set_resolution(adt7410* const dev, adt7410_resolution res);

/**
 * @brief Read the temperature register.
 *
 * On success the value is also cached and can be retrieved later with
 * adt7410_get_temperature().
 *
 * @param      dev  adt7410 device object
 * @param[out] temp Temperature in degrees Celsius, may be NULL
 * @return True on success, false otherwise
 */
bool adt7410_read_temperature(adt7410* const dev, float* temp);

/**
 * @brief Return the temperature cached by the last adt7410_read_temperature().
 *
 * @param dev adt7410 device object
 * @return Temperature in degrees Celsius or ADT7410_INVALID_TEMPERATURE
 */
float adt7410_get_temperature(const adt7410* const dev);

/**
 * @brief Read the manufacturer ID and the silicon revision.
 *
 * @param      dev      adt7410 device object
 * @param[out] id       Manufacturer ID (upper 5 bits of the ID register), may be NULL
 * @param[out] revision Silicon revision (lower 3 bits), may be NULL
 * @return True on success, false otherwise
 */
bool adt7410_read_id(const adt7410* const dev, uint8_t* id, uint8_t* revision);

/**
 * @brief Perform a software reset of the device.
 *
 * @param dev adt7410 device object
 * @return True on success, false otherwise
 */
bool adt7410_reset(adt7410* const dev);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Write bytes into adt7410 device
 *
 * @param dev      adt7410 device object
 * @param reg_addr Register address
 * @param buf      Data to be written
 * @param buf_size Number of bytes to be written
 * @return True on success, false otherwise
 */
extern bool adt7410_write(const adt7410* const dev, uint8_t reg_addr,
                          const uint8_t* buf, size_t buf_size);

/**
 * @brief Read bytes from adt7410 device
 *
 * @param      dev      adt7410 device object
 * @param      reg_addr Register address
 * @param[out] buf      Buffer to fill with read bytes
 * @param      buf_size Number of bytes to read
 * @return True on success, false otherwise
 */
extern bool adt7410_read(const adt7410* const dev, uint8_t reg_addr,
                         uint8_t* buf, size_t buf_size);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void adt7410_delay_ms(uint32_t delay_ms);


#ifdef __cplusplus
}
#endif

#endif /* __ADT7410_H__ */
