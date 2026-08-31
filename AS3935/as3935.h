/**
 *  @brief:   AS3935 Franklin lightning sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __AS3935_H__
#define __AS3935_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief I2C slave addresses (7-bit, unshifted), selected with the ADD1/ADD0 pins.
 *
 * ADD1/ADD0 pull the two address bits, so the device answers at 0x00..0x03.
 * 0x00 is reserved by the AS3935 for the direct commands and must not be used
 * as a slave address, most breakout boards therefore tie both pins high (0x03).
 */
#define AS3935_I2C_ADDR_ADD1_LOW_ADD0_HIGH   0x01
#define AS3935_I2C_ADDR_ADD1_HIGH_ADD0_LOW   0x02
#define AS3935_I2C_ADDR_ADD1_HIGH_ADD0_HIGH  0x03

/** Antenna tuning capacitance used by most breakout boards (3 x 8 pF) */
#define AS3935_DEFAULT_TUNE_CAP     0x03

/** Value reported by as3935_get_storm_distance() when the estimation failed */
#define AS3935_DISTANCE_OUT_OF_RANGE 0x3F

/** Value reported by as3935_get_storm_distance() when the storm is overhead */
#define AS3935_DISTANCE_OVERHEAD     0x01

/**
 * @brief Device register map
 */
typedef enum
{
    AS3935_REG_AFE_GAIN       = 0x00,
    AS3935_REG_THRESHOLD      = 0x01,
    AS3935_REG_LIGHTNING      = 0x02,
    AS3935_REG_INT            = 0x03,
    AS3935_REG_ENERGY_LSB     = 0x04,
    AS3935_REG_ENERGY_MSB     = 0x05,
    AS3935_REG_ENERGY_MMSB    = 0x06,
    AS3935_REG_DISTANCE       = 0x07,
    AS3935_REG_TUNE           = 0x08,
    AS3935_REG_PRESET_DEFAULT = 0x3C,
    AS3935_REG_CALIB_RCO      = 0x3D,
} as3935_reg;

/**
 * @brief Bit masks of the register fields, register they belong to in brackets
 */
#define AS3935_MASK_PWD         0x01u // [0x00] power down
#define AS3935_MASK_AFE_GB      0x3Eu // [0x00] analog front-end gain boost
#define AS3935_MASK_WDTH        0x0Fu // [0x01] watchdog threshold
#define AS3935_MASK_NF_LEV      0x70u // [0x01] noise floor level
#define AS3935_MASK_SREJ        0x0Fu // [0x02] spike rejection
#define AS3935_MASK_MIN_LIGH    0x30u // [0x02] minimum number of lightnings
#define AS3935_MASK_CL_STAT     0x40u // [0x02] clear statistics
#define AS3935_MASK_INT         0x0Fu // [0x03] interrupt source
#define AS3935_MASK_MASK_DIST   0x20u // [0x03] mask disturbers
#define AS3935_MASK_LCO_FDIV    0xC0u // [0x03] LCO frequency division ratio
#define AS3935_MASK_ENERGY_MMSB 0x1Fu // [0x06] energy bits 20..16
#define AS3935_MASK_DISTANCE    0x3Fu // [0x07] storm distance
#define AS3935_MASK_TUN_CAP     0x0Fu // [0x08] antenna tuning capacitors
#define AS3935_MASK_DISP_TRCO   0x20u // [0x08] display TRCO on the IRQ pin
#define AS3935_MASK_DISP_SRCO   0x40u // [0x08] display SRCO on the IRQ pin
#define AS3935_MASK_DISP_LCO    0x80u // [0x08] display LCO on the IRQ pin

/**
 * @brief Analog front-end gain, selects the operating environment
 */
typedef enum
{
    AS3935_AFE_GAIN_INDOOR  = 0x12,
    AS3935_AFE_GAIN_OUTDOOR = 0x0E,
} as3935_afe_gain;

/**
 * @brief Noise floor level, values are given as outdoor/indoor [uVrms]
 */
typedef enum
{
    AS3935_NOISE_FLOOR_390_28   = 0x00,
    AS3935_NOISE_FLOOR_630_45   = 0x01,
    AS3935_NOISE_FLOOR_860_62   = 0x02,
    AS3935_NOISE_FLOOR_1100_78  = 0x03,
    AS3935_NOISE_FLOOR_1140_95  = 0x04,
    AS3935_NOISE_FLOOR_1570_112 = 0x05,
    AS3935_NOISE_FLOOR_1800_130 = 0x06,
    AS3935_NOISE_FLOOR_2000_146 = 0x07,
} as3935_noise_floor;

/**
 * @brief Number of lightnings needed within 15 minutes to raise an interrupt
 */
typedef enum
{
    AS3935_MIN_LIGHTNINGS_1  = 0x00,
    AS3935_MIN_LIGHTNINGS_5  = 0x01,
    AS3935_MIN_LIGHTNINGS_9  = 0x02,
    AS3935_MIN_LIGHTNINGS_16 = 0x03,
} as3935_min_lightnings;

/**
 * @brief Antenna resonance frequency division ratio used during the LCO display
 */
typedef enum
{
    AS3935_LCO_DIV_16  = 0x00,
    AS3935_LCO_DIV_32  = 0x01,
    AS3935_LCO_DIV_64  = 0x02,
    AS3935_LCO_DIV_128 = 0x03,
} as3935_lco_div_ratio;

/**
 * @brief Interrupt sources reported in the low nibble of AS3935_REG_INT
 */
typedef enum
{
    AS3935_INT_NONE           = 0x00,
    AS3935_INT_NOISE_TOO_HIGH = 0x01,
    AS3935_INT_DISTURBER      = 0x04,
    AS3935_INT_LIGHTNING      = 0x08,
} as3935_int_source;

/**
 * @brief Device configuration
 */
typedef struct
{
    as3935_afe_gain       afe_gain;
    as3935_noise_floor    noise_floor;
    as3935_min_lightnings min_lightnings;
    uint8_t               watchdog_threshold; // 0..15
    uint8_t               spike_rejection;    // 0..15
    uint8_t               tune_cap;           // 0..15, 8 pF per step
    bool                  mask_disturbers;    // true: do not report disturbers
} as3935_config;

/**
 * @brief as3935 dev object
 */
typedef struct
{
    uint8_t i2c_addr;   // 7-bit I2C slave address
    void* platform_dev; // Platform descriptor, set by as3935_<platform>_init()
} as3935;

/**
 * @brief Initialize the sensor and apply the default configuration.
 *
 * Default: indoor gain, noise floor level 2, watchdog threshold 2, spike
 * rejection 2, one lightning needed to interrupt, disturbers reported,
 * AS3935_DEFAULT_TUNE_CAP antenna tuning.
 *
 * @param dev as3935 device object
 * @return True on success, false otherwise
 */
bool as3935_init(as3935* const dev);

/**
 * @brief Apply a device configuration.
 *
 * @param dev as3935 device object
 * @param cfg Configuration to be applied
 * @return True on success, false otherwise
 */
bool as3935_set_config(const as3935* const dev, const as3935_config* const cfg);

/**
 * @brief Read back the current device configuration.
 *
 * @param      dev as3935 device object
 * @param[out] cfg Configuration read from the device
 * @return True on success, false otherwise
 */
bool as3935_get_config(const as3935* const dev, as3935_config* const cfg);

/**
 * @brief Restore the factory defaults with the PRESET_DEFAULT direct command.
 *
 * @param dev as3935 device object
 * @return True on success, false otherwise
 */
bool as3935_reset(const as3935* const dev);

/**
 * @brief Calibrate the internal RC oscillators with the CALIB_RCO direct command.
 *
 * @param dev as3935 device object
 * @return True on success, false otherwise
 */
bool as3935_calibrate_rco(const as3935* const dev);

/**
 * @brief Power the analog front-end up and recalibrate the oscillators.
 *
 * @param dev as3935 device object
 * @return True on success, false otherwise
 */
bool as3935_power_up(const as3935* const dev);

/**
 * @brief Power the analog front-end down, the registers keep their content.
 *
 * @param dev as3935 device object
 * @return True on success, false otherwise
 */
bool as3935_power_down(const as3935* const dev);

/**
 * @brief Set the analog front-end gain (indoor or outdoor operation).
 *
 * @param dev  as3935 device object
 * @param gain Gain to be applied
 * @return True on success, false otherwise
 */
bool as3935_set_afe_gain(const as3935* const dev, as3935_afe_gain gain);

/**
 * @brief Read the analog front-end gain.
 *
 * @param      dev  as3935 device object
 * @param[out] gain Gain read from the device
 * @return True on success, false otherwise
 */
bool as3935_get_afe_gain(const as3935* const dev, as3935_afe_gain* const gain);

/**
 * @brief Set the noise floor level.
 *
 * @param dev   as3935 device object
 * @param level Noise floor level to be applied
 * @return True on success, false otherwise
 */
bool as3935_set_noise_floor(const as3935* const dev, as3935_noise_floor level);

/**
 * @brief Read the noise floor level.
 *
 * @param      dev   as3935 device object
 * @param[out] level Noise floor level read from the device
 * @return True on success, false otherwise
 */
bool as3935_get_noise_floor(const as3935* const dev,
                            as3935_noise_floor* const level);

/**
 * @brief Set the watchdog threshold.
 *
 * @param dev  as3935 device object
 * @param wdth Threshold, 0..15, higher values reject more disturbers
 * @return True on success, false otherwise
 */
bool as3935_set_watchdog_threshold(const as3935* const dev, uint8_t wdth);

/**
 * @brief Read the watchdog threshold.
 *
 * @param      dev  as3935 device object
 * @param[out] wdth Threshold read from the device, 0..15
 * @return True on success, false otherwise
 */
bool as3935_get_watchdog_threshold(const as3935* const dev, uint8_t* const wdth);

/**
 * @brief Set the spike rejection ratio.
 *
 * @param dev  as3935 device object
 * @param srej Spike rejection, 0..15, higher values reject more disturbers
 * @return True on success, false otherwise
 */
bool as3935_set_spike_rejection(const as3935* const dev, uint8_t srej);

/**
 * @brief Read the spike rejection ratio.
 *
 * @param      dev  as3935 device object
 * @param[out] srej Spike rejection read from the device, 0..15
 * @return True on success, false otherwise
 */
bool as3935_get_spike_rejection(const as3935* const dev, uint8_t* const srej);

/**
 * @brief Set the minimum number of lightnings needed to raise an interrupt.
 *
 * @param dev as3935 device object
 * @param min Minimum number of lightnings
 * @return True on success, false otherwise
 */
bool as3935_set_min_lightnings(const as3935* const dev,
                               as3935_min_lightnings min);

/**
 * @brief Read the minimum number of lightnings needed to raise an interrupt.
 *
 * @param      dev as3935 device object
 * @param[out] min Minimum number of lightnings read from the device
 * @return True on success, false otherwise
 */
bool as3935_get_min_lightnings(const as3935* const dev,
                               as3935_min_lightnings* const min);

/**
 * @brief Enable or disable the reporting of disturbers.
 *
 * @param dev  as3935 device object
 * @param mask True to mask (hide) the disturber interrupts
 * @return True on success, false otherwise
 */
bool as3935_set_mask_disturbers(const as3935* const dev, bool mask);

/**
 * @brief Check whether the disturber interrupts are masked.
 *
 * @param      dev  as3935 device object
 * @param[out] mask True when the disturber interrupts are masked
 * @return True on success, false otherwise
 */
bool as3935_get_mask_disturbers(const as3935* const dev, bool* const mask);

/**
 * @brief Set the internal tuning capacitors of the antenna.
 *
 * @param dev as3935 device object
 * @param cap Capacitance setting, 0..15, 8 pF per step
 * @return True on success, false otherwise
 */
bool as3935_set_tune_cap(const as3935* const dev, uint8_t cap);

/**
 * @brief Read the internal tuning capacitors setting of the antenna.
 *
 * @param      dev as3935 device object
 * @param[out] cap Capacitance setting read from the device, 0..15
 * @return True on success, false otherwise
 */
bool as3935_get_tune_cap(const as3935* const dev, uint8_t* const cap);

/**
 * @brief Select the antenna resonance frequency division ratio.
 *
 * @param dev   as3935 device object
 * @param ratio Division ratio applied when the LCO is routed to the IRQ pin
 * @return True on success, false otherwise
 */
bool as3935_set_lco_div_ratio(const as3935* const dev,
                              as3935_lco_div_ratio ratio);

/**
 * @brief Route the antenna resonance frequency to the IRQ pin, or stop doing so.
 *
 * Used together with as3935_set_tune_cap() to tune the antenna to 500 kHz.
 *
 * @param dev    as3935 device object
 * @param enable True to output the LCO on the IRQ pin
 * @return True on success, false otherwise
 */
bool as3935_display_lco(const as3935* const dev, bool enable);

/**
 * @brief Clear the accumulated statistics of the distance estimation.
 *
 * @param dev as3935 device object
 * @return True on success, false otherwise
 */
bool as3935_clear_statistics(const as3935* const dev);

/**
 * @brief Read the interrupt source register.
 *
 * The AS3935 needs at least 2 ms after the IRQ pin has gone high before the
 * register holds a valid value, so this call always waits that long first.
 *
 * @param      dev as3935 device object
 * @param[out] src Interrupt source, one of the as3935_int_source values
 * @return True on success, false otherwise
 */
bool as3935_read_interrupt_source(const as3935* const dev,
                                  as3935_int_source* const src);

/**
 * @brief Read the estimated distance to the head of the storm.
 *
 * @param      dev         as3935 device object
 * @param[out] distance_km Distance in kilometers, AS3935_DISTANCE_OVERHEAD when
 *                         the storm is overhead and AS3935_DISTANCE_OUT_OF_RANGE
 *                         when it is out of range
 * @return True on success, false otherwise
 */
bool as3935_get_storm_distance(const as3935* const dev,
                               uint8_t* const distance_km);

/**
 * @brief Read the energy of the last detected single lightning.
 *
 * The value is a 21-bit pure number without a physical unit, it may only be
 * compared against other readings of the same device.
 *
 * @param      dev    as3935 device object
 * @param[out] energy Raw energy of the last lightning, 0..0x1FFFFF
 * @return True on success, false otherwise
 */
bool as3935_get_energy(const as3935* const dev, uint32_t* const energy);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Write bytes into as3935 device
 *
 * @param dev      as3935 device object
 * @param reg_addr Register address
 * @param buf      Data to be written
 * @param buf_size Number of bytes to be written
 * @return True on success, false otherwise
 */
extern bool as3935_write(const as3935* const dev, uint8_t reg_addr,
                         const uint8_t* buf, size_t buf_size);

/**
 * @brief Read bytes from as3935 device
 *
 * @param      dev      as3935 device object
 * @param      reg_addr Register address
 * @param[out] buf      Buffer to fill with read bytes
 * @param      buf_size Number of bytes to read
 * @return True on success, false otherwise
 */
extern bool as3935_read(const as3935* const dev, uint8_t reg_addr,
                        uint8_t* buf, size_t buf_size);

/**
 * @brief Read the current level of the IRQ pin.
 *
 * Lets the application poll the sensor without relying on interrupts.
 *
 * @param dev as3935 device object
 * @return True when the IRQ pin is high, false otherwise
 */
extern bool as3935_get_irq_pin(const as3935* const dev);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void as3935_delay_ms(uint32_t delay_ms);


#ifdef __cplusplus
}
#endif

#endif /* __AS3935_H__ */
