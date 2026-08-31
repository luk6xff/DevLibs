/**
 *  @brief:   LTC2460 ultra-tiny 16-bit delta-sigma ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __LTC2460_H__
#define __LTC2460_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** Number of ADC codes the conversion result is scaled against */
#define LTC2460_FULL_SCALE          65535.0f

/** Internal reference voltage of the LTC2460 in volts */
#define LTC2460_DEFAULT_VREF        1.25f

/** Worst case conversion time in 1x speed mode (30 SPS plus margin) */
#define LTC2460_CONVERSION_TIME_MS  50u

/**
 * @brief Bits of the 16-bit configuration word shifted into the device.
 *
 * The remaining bits are "don't care" and are always sent as zeros.
 */
#define LTC2460_CFG_EN              0x8000u // Enables the update of SLP and SPD
#define LTC2460_CFG_SLP             0x4000u // Puts the device into sleep mode
#define LTC2460_CFG_SPD             0x2000u // 2x speed, disables auto-calibration

/**
 * @brief Output data rate
 */
typedef enum
{
    LTC2460_SPEED_1X = 0, // 30 SPS, auto-calibration before every conversion
    LTC2460_SPEED_2X = 1, // 60 SPS, no auto-calibration
} ltc2460_speed;

/**
 * @brief ltc2460 dev object
 */
typedef struct
{
    float vref;          // Reference voltage in volts, LTC2460_DEFAULT_VREF for the internal one
    float divider_ratio; // External divider GND--R1--ADC--R2--Vmeasured, ratio = R1/R2, 0.0f if unused
    void* platform_dev;  // Platform descriptor, assigned by ltc2460_<platform>_init()
} ltc2460;

/**
 * @brief Initialize the ADC and trigger the first conversion.
 *
 * @param dev ltc2460 device object
 * @return True on success, false otherwise
 */
bool ltc2460_init(ltc2460* const dev);

/**
 * @brief Shift a raw configuration word in and the last conversion result out.
 *
 * @param      dev ltc2460 device object
 * @param      cfg Configuration word, see the LTC2460_CFG_* defines
 * @param[out] raw Last conversion result, may be NULL
 * @return True on success, false otherwise
 */
bool ltc2460_transfer(const ltc2460* const dev, uint16_t cfg, uint16_t* raw);

/**
 * @brief Read the last 16-bit conversion result without changing the configuration.
 *
 * @param      dev ltc2460 device object
 * @param[out] raw Raw 16-bit ADC code
 * @return True on success, false otherwise
 */
bool ltc2460_read_raw(const ltc2460* const dev, uint16_t* raw);

/**
 * @brief Read the ADC and convert the result into volts.
 *
 * The value is scaled by the configured `divider_ratio`, so the returned value
 * is the voltage present in front of the divider.
 *
 * @param      dev     ltc2460 device object
 * @param[out] voltage Measured voltage in volts
 * @return True on success, false otherwise
 */
bool ltc2460_read_voltage(const ltc2460* const dev, float* voltage);

/**
 * @brief Select the output data rate.
 *
 * @param dev   ltc2460 device object
 * @param speed Requested output data rate
 * @return True on success, false otherwise
 */
bool ltc2460_set_speed(const ltc2460* const dev, ltc2460_speed speed);

/**
 * @brief Enter or leave the low power sleep mode.
 *
 * @param dev    ltc2460 device object
 * @param enable True puts the device to sleep, false wakes it up
 * @return True on success, false otherwise
 */
bool ltc2460_set_sleep(const ltc2460* const dev, bool enable);

/**
 * @brief Convert a raw ADC code into volts.
 *
 * @param dev ltc2460 device object
 * @param raw Raw 16-bit ADC code
 * @return Voltage in volts
 */
float ltc2460_raw_to_voltage(const ltc2460* const dev, uint16_t raw);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Full duplex SPI transfer, 8-bit frames, MSB first, mode 0.
 *
 * @param      dev ltc2460 device object
 * @param      tx  Bytes to be shifted out
 * @param[out] rx  Buffer filled with the bytes shifted in
 * @param      len Number of bytes to transfer
 * @return True on success, false otherwise
 */
extern bool ltc2460_spi_transfer(const ltc2460* const dev, const uint8_t* tx,
                                 uint8_t* rx, size_t len);

/**
 * @brief Drive the chip select pin.
 *
 * @param dev   ltc2460 device object
 * @param state True drives the pin HIGH (device deselected), false LOW
 */
extern void ltc2460_set_cs_pin(const ltc2460* const dev, bool state);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void ltc2460_delay_ms(uint32_t delay_ms);


#ifdef __cplusplus
}
#endif

#endif /* __LTC2460_H__ */
