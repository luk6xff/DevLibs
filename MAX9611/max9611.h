/**
 *  @brief:   MAX9611 high-side current-sense amplifier with 12-bit ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MAX9611_H__
#define __MAX9611_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief I2C slave addresses (7-bit, unshifted).
 *
 * The address is strapped with the A0 and A1 pins, each of them can be tied to
 * GND, VCC, SDA or SCL, which gives the 0x70..0x7F address range.
 * MAX9611_I2C_ADDR_A1_GND_A0_GND matches the 0xE0 write / 0xE1 read address
 * pair quoted in the MAX9611 datasheet.
 */
#define MAX9611_I2C_ADDR_A1_GND_A0_GND  0x70
#define MAX9611_I2C_ADDR_A1_GND_A0_VCC  0x71
#define MAX9611_I2C_ADDR_A1_GND_A0_SDA  0x72
#define MAX9611_I2C_ADDR_A1_GND_A0_SCL  0x73
#define MAX9611_I2C_ADDR_A1_VCC_A0_GND  0x74
#define MAX9611_I2C_ADDR_A1_VCC_A0_VCC  0x75
#define MAX9611_I2C_ADDR_A1_VCC_A0_SDA  0x76
#define MAX9611_I2C_ADDR_A1_VCC_A0_SCL  0x77
#define MAX9611_I2C_ADDR_A1_SDA_A0_GND  0x78
#define MAX9611_I2C_ADDR_A1_SDA_A0_VCC  0x79
#define MAX9611_I2C_ADDR_A1_SDA_A0_SDA  0x7A
#define MAX9611_I2C_ADDR_A1_SDA_A0_SCL  0x7B
#define MAX9611_I2C_ADDR_A1_SCL_A0_GND  0x7C
#define MAX9611_I2C_ADDR_A1_SCL_A0_VCC  0x7D
#define MAX9611_I2C_ADDR_A1_SCL_A0_SDA  0x7E
#define MAX9611_I2C_ADDR_A1_SCL_A0_SCL  0x7F

/** Time needed by the ADC to convert a freshly selected channel */
#define MAX9611_CONVERSION_TIME_MS      2u

/** Current-sense amplifier LSB in microvolts, per gain setting */
#define MAX9611_CSA_LSB_UV_GAIN_1X      107.50f
#define MAX9611_CSA_LSB_UV_GAIN_4X      26.88f
#define MAX9611_CSA_LSB_UV_GAIN_8X      13.44f

/** RS+ common mode voltage LSB in volts */
#define MAX9611_RS_LSB_V                0.014f

/** OUT and SET voltage LSB in volts */
#define MAX9611_OUT_LSB_V               0.00345f

/** Die temperature LSB in degrees Celsius */
#define MAX9611_TEMP_LSB_C              0.48f

/**
 * @brief Device register map
 */
typedef enum
{
    MAX9611_REG_CSA_MSB  = 0x00,
    MAX9611_REG_CSA_LSB  = 0x01,
    MAX9611_REG_RS_MSB   = 0x02,
    MAX9611_REG_RS_LSB   = 0x03,
    MAX9611_REG_OUT_MSB  = 0x04,
    MAX9611_REG_OUT_LSB  = 0x05,
    MAX9611_REG_SET_MSB  = 0x06,
    MAX9611_REG_SET_LSB  = 0x07,
    MAX9611_REG_TEMP_MSB = 0x08,
    MAX9611_REG_TEMP_LSB = 0x09,
    MAX9611_REG_CTRL1    = 0x0A,
    MAX9611_REG_CTRL2    = 0x0B,
} max9611_reg;

/**
 * @brief ADC input multiplexer selection, bits [2:0] of CTRL1
 */
typedef enum
{
    MAX9611_MUX_CSA_GAIN_1X    = 0, // Current-sense amplifier output, gain 1x
    MAX9611_MUX_CSA_GAIN_4X    = 1, // Current-sense amplifier output, gain 4x
    MAX9611_MUX_CSA_GAIN_8X    = 2, // Current-sense amplifier output, gain 8x
    MAX9611_MUX_RS_COMMON_MODE = 3, // Average voltage of RS+
    MAX9611_MUX_OUT_VOLTAGE    = 4, // Voltage of the OUT pin
    MAX9611_MUX_SET_VOLTAGE    = 5, // Voltage of the SET pin
    MAX9611_MUX_TEMPERATURE    = 6, // Internal die temperature
    MAX9611_MUX_ALL_CHANNELS   = 7, // Fast-read mode, all channels every 2 ms
} max9611_mux;

/**
 * @brief Current-sense amplifier gain
 */
typedef enum
{
    MAX9611_GAIN_1X = 0, // Full scale 110 mV across the shunt
    MAX9611_GAIN_4X = 1, // Full scale 27.5 mV across the shunt
    MAX9611_GAIN_8X = 2, // Full scale 13.75 mV across the shunt
} max9611_gain;

/**
 * @brief Op amp / comparator operating mode, bits [7:5] of CTRL1
 */
typedef enum
{
    MAX9611_MODE_NORMAL     = 0,
    MAX9611_MODE_OPAMP      = 3,
    MAX9611_MODE_COMPARATOR = 7,
} max9611_mode;

/**
 * @brief Watchdog delay time, bit 3 of CTRL2
 */
typedef enum
{
    MAX9611_WATCHDOG_DELAY_1MS   = 0,
    MAX9611_WATCHDOG_DELAY_100US = 1,
} max9611_watchdog_delay;

/**
 * @brief Watchdog retry delay time, bit 2 of CTRL2
 */
typedef enum
{
    MAX9611_WATCHDOG_RETRY_50MS = 0,
    MAX9611_WATCHDOG_RETRY_10MS = 1,
} max9611_watchdog_retry;

/**
 * @brief Device configuration
 */
typedef struct
{
    max9611_mode mode;                        // Op amp / comparator mode
    bool shutdown;                            // True puts the device into shutdown
    max9611_mux mux;                          // Channel selected by the ADC mux
    max9611_watchdog_delay watchdog_delay;    // Watchdog delay time
    max9611_watchdog_retry watchdog_retry;    // Watchdog retry delay time
} max9611_config;

/**
 * @brief max9611 dev object
 */
typedef struct
{
    uint8_t i2c_addr;     // 7-bit I2C slave address
    float shunt_res_ohm;  // Sense resistor value in ohms, e.g. 0.1f
    max9611_gain gain;    // Current-sense amplifier gain used for current readings
    uint16_t csa_offset;  // Raw CSA code measured at zero current, subtracted from every reading
    void* platform_dev;   // Platform descriptor, assigned by max9611_<platform>_init()
} max9611;

/**
 * @brief Initialize the device and apply the default configuration.
 *
 * Default: normal mode, device running, ADC mux following `dev->gain`,
 * 1 ms watchdog delay and 50 ms watchdog retry delay.
 * `dev->shunt_res_ohm` must be set to a positive value beforehand.
 *
 * @param dev max9611 device object
 * @return True on success, false otherwise
 */
bool max9611_init(max9611* const dev);

/**
 * @brief Apply a device configuration.
 *
 * @param dev max9611 device object
 * @param cfg Configuration to be applied
 * @return True on success, false otherwise
 */
bool max9611_set_config(const max9611* const dev,
                        const max9611_config* const cfg);

/**
 * @brief Read back the current device configuration.
 *
 * @param      dev max9611 device object
 * @param[out] cfg Configuration read from the device
 * @return True on success, false otherwise
 */
bool max9611_get_config(const max9611* const dev, max9611_config* const cfg);

/**
 * @brief Select the ADC input channel and wait for a conversion to complete.
 *
 * @param dev max9611 device object
 * @param mux Channel to be selected
 * @return True on success, false otherwise
 */
bool max9611_select_channel(const max9611* const dev, max9611_mux mux);

/**
 * @brief Select the current-sense amplifier gain used by max9611_read_current().
 *
 * @param dev  max9611 device object
 * @param gain Gain to be applied
 * @return True on success, false otherwise
 */
bool max9611_set_gain(max9611* const dev, max9611_gain gain);

/**
 * @brief Select a channel, convert it and return the raw ADC code.
 *
 * All channels are 12 bit wide except the temperature channel, which is 9 bit.
 *
 * @param      dev max9611 device object
 * @param      mux Channel to be read, MAX9611_MUX_ALL_CHANNELS is not supported
 * @param[out] raw Raw, right aligned ADC code
 * @return True on success, false otherwise
 */
bool max9611_read_raw(const max9611* const dev, max9611_mux mux, uint16_t* raw);

/**
 * @brief Read the voltage drop across the sense resistor.
 *
 * @param      dev        max9611 device object
 * @param[out] voltage_mv Shunt voltage in millivolts
 * @return True on success, false otherwise
 */
bool max9611_read_shunt_voltage(const max9611* const dev, float* voltage_mv);

/**
 * @brief Read the current flowing through the sense resistor.
 *
 * @param      dev        max9611 device object
 * @param[out] current_ma Current in milliamperes
 * @return True on success, false otherwise
 */
bool max9611_read_current(const max9611* const dev, float* current_ma);

/**
 * @brief Read the common mode voltage present on the RS+ input.
 *
 * @param      dev       max9611 device object
 * @param[out] voltage_v Input voltage in volts
 * @return True on success, false otherwise
 */
bool max9611_read_input_voltage(const max9611* const dev, float* voltage_v);

/**
 * @brief Read the voltage of the OUT pin.
 *
 * @param      dev       max9611 device object
 * @param[out] voltage_v OUT voltage in volts
 * @return True on success, false otherwise
 */
bool max9611_read_out_voltage(const max9611* const dev, float* voltage_v);

/**
 * @brief Read the voltage of the SET pin.
 *
 * @param      dev       max9611 device object
 * @param[out] voltage_v SET voltage in volts
 * @return True on success, false otherwise
 */
bool max9611_read_set_voltage(const max9611* const dev, float* voltage_v);

/**
 * @brief Read the internal die temperature.
 *
 * @param      dev    max9611 device object
 * @param[out] temp_c Temperature in degrees Celsius
 * @return True on success, false otherwise
 */
bool max9611_read_temperature(const max9611* const dev, float* temp_c);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Write bytes into max9611 device
 *
 * @param dev      max9611 device object
 * @param reg_addr Register address
 * @param buf      Data to be written
 * @param buf_size Number of bytes to be written
 * @return True on success, false otherwise
 */
extern bool max9611_write(const max9611* const dev, uint8_t reg_addr,
                          const uint8_t* buf, size_t buf_size);

/**
 * @brief Read bytes from max9611 device
 *
 * @param      dev      max9611 device object
 * @param      reg_addr Register address
 * @param[out] buf      Buffer to fill with read bytes
 * @param      buf_size Number of bytes to read
 * @return True on success, false otherwise
 */
extern bool max9611_read(const max9611* const dev, uint8_t reg_addr,
                         uint8_t* buf, size_t buf_size);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void max9611_delay_ms(uint32_t delay_ms);


#ifdef __cplusplus
}
#endif

#endif /* __MAX9611_H__ */
