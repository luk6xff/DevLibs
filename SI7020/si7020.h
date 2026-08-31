/**
 *  @brief:   SI7020 relative humidity and temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __SI7020_H__
#define __SI7020_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief I2C slave address (7-bit, unshifted).
 *
 * The address is fixed in silicon, the SI7020 has no address strapping pins.
 */
#define SI7020_I2C_ADDR             0x40

/** SNB_3 byte of the electronic serial number identifying an SI7020 part */
#define SI7020_DEVICE_ID            0x14

/** Length of the electronic serial number in bytes */
#define SI7020_SERIAL_NUMBER_SIZE   8

/** CRC-8 generator polynomial x^8 + x^5 + x^4 + 1 used by the SI7020 */
#define SI7020_CRC8_POLYNOMIAL      0x31

/** Values reported by the getters when no valid reading exists */
#define SI7020_INVALID_TEMPERATURE  (-999.0f)
#define SI7020_INVALID_HUMIDITY     (-1.0f)

/** Worst case combined RH + temperature conversion time in milliseconds */
#define SI7020_CONVERSION_TIME_MS   25

/** Time the device needs to become responsive again after a reset */
#define SI7020_RESET_TIME_MS        15

/**
 * @brief Device command set.
 *
 * The SI7020 has no register map, every access starts with one of these
 * command bytes. Commands marked as two byte are sent as a pair.
 */
typedef enum
{
    SI7020_CMD_MEASURE_RH_HOLD      = 0xE5, // Measure RH, hold master mode
    SI7020_CMD_MEASURE_RH_NO_HOLD   = 0xF5, // Measure RH, no hold master mode
    SI7020_CMD_MEASURE_TEMP_HOLD    = 0xE3, // Measure T, hold master mode
    SI7020_CMD_MEASURE_TEMP_NO_HOLD = 0xF3, // Measure T, no hold master mode
    SI7020_CMD_READ_TEMP_FROM_RH    = 0xE0, // T from the previous RH measurement
    SI7020_CMD_RESET                = 0xFE, // Software reset
    SI7020_CMD_WRITE_USER_REG1      = 0xE6, // Write user register 1
    SI7020_CMD_READ_USER_REG1       = 0xE7, // Read user register 1
    SI7020_CMD_READ_ID_1_BYTE1      = 0xFA, // Read electronic ID, 1st byte (2 byte cmd)
    SI7020_CMD_READ_ID_1_BYTE2      = 0x0F,
    SI7020_CMD_READ_ID_2_BYTE1      = 0xFC, // Read electronic ID, 2nd byte (2 byte cmd)
    SI7020_CMD_READ_ID_2_BYTE2      = 0xC9,
    SI7020_CMD_READ_FW_REV_BYTE1    = 0x84, // Read firmware revision (2 byte cmd)
    SI7020_CMD_READ_FW_REV_BYTE2    = 0xB8,
} si7020_cmd;

/**
 * @brief User register 1 bit masks
 */
#define SI7020_USER_REG1_RES1       0x80u // Resolution, most significant bit
#define SI7020_USER_REG1_VDDS       0x40u // Read only, 1 when VDD is below 1.9 V
#define SI7020_USER_REG1_HTRE       0x04u // On chip heater enable
#define SI7020_USER_REG1_RES0       0x01u // Resolution, least significant bit
#define SI7020_USER_REG1_RES_MASK   (SI7020_USER_REG1_RES1 | SI7020_USER_REG1_RES0)

/**
 * @brief Firmware revision codes returned by si7020_read_firmware_revision()
 */
#define SI7020_FW_REVISION_1_0      0xFF
#define SI7020_FW_REVISION_2_0      0x20

/**
 * @brief Measurement resolution, the values match the user register 1 bits
 */
typedef enum
{
    SI7020_RESOLUTION_RH12_TEMP14 = 0x00,
    SI7020_RESOLUTION_RH8_TEMP12  = 0x01,
    SI7020_RESOLUTION_RH10_TEMP13 = 0x80,
    SI7020_RESOLUTION_RH11_TEMP11 = 0x81,
} si7020_resolution;

/**
 * @brief Device configuration
 */
typedef struct
{
    si7020_resolution resolution;
    bool heater_enabled; // On chip heater, used to drive off condensation
} si7020_config;

/**
 * @brief si7020 dev object
 */
typedef struct
{
    uint8_t i2c_addr;   // 7-bit I2C slave address
    void* platform_dev; // Platform descriptor, set by si7020_<platform>_init()
    float temperature;  // Last successful reading in degC
    float humidity;     // Last successful reading in %RH
} si7020;

/**
 * @brief Reset the sensor, verify the part number and apply the default config.
 *
 * Default: 12-bit RH / 14-bit temperature, heater disabled.
 *
 * @param dev si7020 device object
 * @return True on success, false otherwise
 */
bool si7020_init(si7020* const dev);

/**
 * @brief Apply a device configuration.
 *
 * The reserved bits of user register 1 are preserved.
 *
 * @param dev si7020 device object
 * @param cfg Configuration to be applied
 * @return True on success, false otherwise
 */
bool si7020_set_config(const si7020* const dev, const si7020_config* const cfg);

/**
 * @brief Read back the current device configuration.
 *
 * @param      dev si7020 device object
 * @param[out] cfg Configuration read from the device
 * @return True on success, false otherwise
 */
bool si7020_get_config(const si7020* const dev, si7020_config* const cfg);

/**
 * @brief Measure the relative humidity.
 *
 * The measurement is started in no hold master mode, so the bus is released
 * while the conversion runs.
 *
 * @param      dev      si7020 device object
 * @param[out] humidity Relative humidity in %RH, may be NULL
 * @return True on success, false otherwise
 */
bool si7020_read_humidity(si7020* const dev, float* humidity);

/**
 * @brief Measure the temperature.
 *
 * @param      dev  si7020 device object
 * @param[out] temp Temperature in degrees Celsius, may be NULL
 * @return True on success, false otherwise
 */
bool si7020_read_temperature(si7020* const dev, float* temp);

/**
 * @brief Read the temperature acquired during the last humidity measurement.
 *
 * No new conversion is started, which saves both time and power.
 *
 * @param      dev  si7020 device object
 * @param[out] temp Temperature in degrees Celsius, may be NULL
 * @return True on success, false otherwise
 */
bool si7020_read_temperature_from_last_humidity(si7020* const dev, float* temp);

/**
 * @brief Return the humidity cached by the last successful read.
 *
 * @param dev si7020 device object
 * @return Relative humidity in %RH or SI7020_INVALID_HUMIDITY
 */
float si7020_get_humidity(const si7020* const dev);

/**
 * @brief Return the temperature cached by the last successful read.
 *
 * @param dev si7020 device object
 * @return Temperature in degrees Celsius or SI7020_INVALID_TEMPERATURE
 */
float si7020_get_temperature(const si7020* const dev);

/**
 * @brief Check whether the supply voltage is still within the valid range.
 *
 * @param      dev     si7020 device object
 * @param[out] vdd_ok  True when VDD is above the 1.9 V threshold
 * @return True on success, false otherwise
 */
bool si7020_is_vdd_ok(const si7020* const dev, bool* vdd_ok);

/**
 * @brief Read the firmware revision of the sensor.
 *
 * @param      dev si7020 device object
 * @param[out] rev SI7020_FW_REVISION_1_0 or SI7020_FW_REVISION_2_0
 * @return True on success, false otherwise
 */
bool si7020_read_firmware_revision(const si7020* const dev, uint8_t* rev);

/**
 * @brief Read the 64-bit electronic serial number.
 *
 * serial[4] holds the SNB_3 byte which identifies the part, see
 * SI7020_DEVICE_ID.
 *
 * @param      dev         si7020 device object
 * @param[out] serial      Buffer filled with the serial number
 * @param      serial_size Buffer size, must be SI7020_SERIAL_NUMBER_SIZE
 * @return True on success, false otherwise
 */
bool si7020_read_serial_number(const si7020* const dev, uint8_t* serial,
                               size_t serial_size);

/**
 * @brief Perform a software reset of the device.
 *
 * @param dev si7020 device object
 * @return True on success, false otherwise
 */
bool si7020_reset(si7020* const dev);

/**
 * @brief Compute the SI7020 CRC-8 checksum (polynomial 0x31, initial value 0).
 *
 * @param data Data to be checksummed
 * @param len  Number of bytes
 * @return The computed checksum
 */
uint8_t si7020_crc8(const uint8_t* data, size_t len);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Write a command sequence into the si7020 device
 *
 * @param dev      si7020 device object
 * @param cmd      Command bytes, one or two depending on the command
 * @param cmd_size Number of command bytes
 * @return True on success, false otherwise
 */
extern bool si7020_write_cmd(const si7020* const dev, const uint8_t* cmd,
                             size_t cmd_size);

/**
 * @brief Read raw bytes from the si7020 device
 *
 * The device answers the command sent by the preceding si7020_write_cmd()
 * call, no register address is transmitted.
 *
 * @param      dev      si7020 device object
 * @param[out] buf      Buffer to fill with read bytes
 * @param      buf_size Number of bytes to read
 * @return True on success, false otherwise
 */
extern bool si7020_read_raw(const si7020* const dev, uint8_t* buf,
                            size_t buf_size);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void si7020_delay_ms(uint32_t delay_ms);


#ifdef __cplusplus
}
#endif

#endif /* __SI7020_H__ */
