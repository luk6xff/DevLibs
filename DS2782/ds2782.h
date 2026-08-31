/**
 *  @brief:   DS2782 stand-alone battery fuel gauge library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __DS2782_H__
#define __DS2782_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief I2C slave address (7-bit, unshifted).
 *
 * The DS2782 answers at 0x34, which is the 0x68 8-bit address used by the
 * datasheet shifted right by one. The address can be reprogrammed through the
 * I2C_SLAVE_ADDR EEPROM cell of the parameter block.
 */
#define DS2782_I2C_ADDR             0x34

/** Sense resistor value assumed when dev->rsense_mohm is left at zero */
#define DS2782_DEFAULT_RSENSE_MOHM  20

/**
 * @brief Device register map, every MSB register is followed by its LSB
 */
typedef enum
{
    DS2782_REG_STATUS         = 0x01, // Status register
    DS2782_REG_RAAC_MSB       = 0x02, // Remaining active absolute capacity
    DS2782_REG_RAAC_LSB       = 0x03,
    DS2782_REG_RSAC_MSB       = 0x04, // Remaining standby absolute capacity
    DS2782_REG_RSAC_LSB       = 0x05,
    DS2782_REG_RARC           = 0x06, // Remaining active relative capacity
    DS2782_REG_RSRC           = 0x07, // Remaining standby relative capacity
    DS2782_REG_IAVG_MSB       = 0x08, // Average current
    DS2782_REG_IAVG_LSB       = 0x09,
    DS2782_REG_TEMP_MSB       = 0x0A, // Temperature
    DS2782_REG_TEMP_LSB       = 0x0B,
    DS2782_REG_VOLT_MSB       = 0x0C, // Voltage
    DS2782_REG_VOLT_LSB       = 0x0D,
    DS2782_REG_CURRENT_MSB    = 0x0E, // Instantaneous current
    DS2782_REG_CURRENT_LSB    = 0x0F,
    DS2782_REG_ACR_MSB        = 0x10, // Accumulated current
    DS2782_REG_ACR_LSB        = 0x11,
    DS2782_REG_ACRL_MSB       = 0x12, // Low accumulated current
    DS2782_REG_ACRL_LSB       = 0x13,
    DS2782_REG_AS             = 0x14, // Age scalar
    DS2782_REG_SFR            = 0x15, // Special feature register
    DS2782_REG_FULL_MSB       = 0x16, // Full capacity
    DS2782_REG_FULL_LSB       = 0x17,
    DS2782_REG_AE_MSB         = 0x18, // Active empty
    DS2782_REG_AE_LSB         = 0x19,
    DS2782_REG_SE_MSB         = 0x1A, // Standby empty
    DS2782_REG_SE_LSB         = 0x1B,
    DS2782_REG_EEPROM         = 0x1F, // EEPROM register
    DS2782_REG_USR_EEPROM     = 0x20, // User EEPROM, block 0, 0x20..0x2F
    DS2782_REG_ADD_USR_EEPROM = 0x30, // Additional user EEPROM, 0x30..0x37
    DS2782_REG_PARAM_EEPROM   = 0x60, // Parameter EEPROM, block 1, 0x60..0x7F
    DS2782_REG_UNIQUE_ID      = 0xF0, // Unique ID, 0xF0..0xF7
    DS2782_REG_FUNC_COMMAND   = 0xFE, // Function command register
} ds2782_reg;

/**
 * @brief Parameter EEPROM (block 1) register map
 */
typedef enum
{
    DS2782_EE_CONTROL         = 0x60, // Control register
    DS2782_EE_AB              = 0x61, // Accumulation bias
    DS2782_EE_AC_MSB          = 0x62, // Aging capacity
    DS2782_EE_AC_LSB          = 0x63,
    DS2782_EE_VCHG            = 0x64, // Charge voltage
    DS2782_EE_IMIN            = 0x65, // Minimum charge current
    DS2782_EE_VAE             = 0x66, // Active empty voltage
    DS2782_EE_IAE             = 0x67, // Active empty current
    DS2782_EE_ACTIVE_EMPTY_40 = 0x68, // Active empty capacity at 40 degC
    DS2782_EE_RSNSP           = 0x69, // Sense resistor prime
    DS2782_EE_FULL_40_MSB     = 0x6A, // Full capacity at 40 degC
    DS2782_EE_FULL_40_LSB     = 0x6B,
    DS2782_EE_FULL_3040_SLOPE = 0x6C,
    DS2782_EE_FULL_2030_SLOPE = 0x6D,
    DS2782_EE_FULL_1020_SLOPE = 0x6E,
    DS2782_EE_FULL_0010_SLOPE = 0x6F,
    DS2782_EE_AE_3040_SLOPE   = 0x70,
    DS2782_EE_AE_2030_SLOPE   = 0x71,
    DS2782_EE_AE_1020_SLOPE   = 0x72,
    DS2782_EE_AE_0010_SLOPE   = 0x73,
    DS2782_EE_SE_3040_SLOPE   = 0x74,
    DS2782_EE_SE_2030_SLOPE   = 0x75,
    DS2782_EE_SE_1020_SLOPE   = 0x76,
    DS2782_EE_SE_0010_SLOPE   = 0x77,
    DS2782_EE_RSGAIN_MSB      = 0x78, // Sense resistor gain
    DS2782_EE_RSGAIN_LSB      = 0x79,
    DS2782_EE_RSTC            = 0x7A, // Sense resistor temperature coefficient
    DS2782_EE_FRSGAIN_MSB     = 0x7B, // Factory gain
    DS2782_EE_FRSGAIN_LSB     = 0x7C,
    DS2782_EE_I2C_SLAVE_ADDR  = 0x7E, // 2-wire slave address
} ds2782_eeprom_reg;

/**
 * @brief EEPROM blocks addressed by the copy and the recall function commands
 */
typedef enum
{
    DS2782_EEPROM_BLOCK_0 = 0x00, // User EEPROM, 0x20..0x37
    DS2782_EEPROM_BLOCK_1 = 0x01, // Parameter EEPROM, 0x60..0x7F
} ds2782_eeprom_block;

/**
 * @brief Status register flags
 */
typedef enum
{
    DS2782_STATUS_PORF   = 0x02, // Power-on reset, useful for reset detection
    DS2782_STATUS_UVF    = 0x04, // Under voltage
    DS2782_STATUS_LEARNF = 0x10, // A charge cycle may be used to learn capacity
    DS2782_STATUS_SEF    = 0x20, // Standby empty
    DS2782_STATUS_AEF    = 0x40, // Active empty
    DS2782_STATUS_CHGTF  = 0x80, // Charge termination
} ds2782_status_flag;

/**
 * @brief ds2782 dev object
 */
typedef struct
{
    uint8_t i2c_addr;     // 7-bit I2C slave address
    uint16_t rsense_mohm; // Sense resistor in milliohms, 0 selects the default
    void* platform_dev;   // Platform descriptor, set by ds2782_<platform>_init()
} ds2782;

/**
 * @brief Initialize the fuel gauge.
 *
 * Probes the device and falls back to DS2782_DEFAULT_RSENSE_MOHM when no sense
 * resistor value has been provided. No EEPROM cell is touched, the battery
 * model has to be programmed by the application.
 *
 * @param dev ds2782 device object
 * @return True on success, false otherwise
 */
bool ds2782_init(ds2782* const dev);

/**
 * @brief Read the status register.
 *
 * @param      dev    ds2782 device object
 * @param[out] status Raw status register, a mask of ds2782_status_flag values
 * @return True on success, false otherwise
 */
bool ds2782_read_status(const ds2782* const dev, uint8_t* const status);

/**
 * @brief Read the battery voltage.
 *
 * 11-bit two's complement value, 4.88 mV per LSB.
 *
 * @param      dev        ds2782 device object
 * @param[out] voltage_mv Battery voltage in millivolts [mV]
 * @return True on success, false otherwise
 */
bool ds2782_read_voltage(const ds2782* const dev, float* const voltage_mv);

/**
 * @brief Read the instantaneous current.
 *
 * 16-bit two's complement value, 1.5625 uV per LSB across the sense resistor,
 * positive while the battery is being charged.
 *
 * @param      dev        ds2782 device object
 * @param[out] current_ma Current in milliamperes [mA]
 * @return True on success, false otherwise
 */
bool ds2782_read_current(const ds2782* const dev, float* const current_ma);

/**
 * @brief Read the average current of the last conversion window.
 *
 * Same scaling and sign convention as ds2782_read_current().
 *
 * @param      dev        ds2782 device object
 * @param[out] current_ma Average current in milliamperes [mA]
 * @return True on success, false otherwise
 */
bool ds2782_read_average_current(const ds2782* const dev,
                                 float* const current_ma);

/**
 * @brief Read the accumulated current register.
 *
 * 16-bit two's complement value, 6.25 uVh per LSB across the sense resistor.
 *
 * @param      dev        ds2782 device object
 * @param[out] charge_mah Accumulated charge in milliampere-hours [mAh]
 * @return True on success, false otherwise
 */
bool ds2782_read_accumulated_current(const ds2782* const dev,
                                     float* const charge_mah);

/**
 * @brief Overwrite the accumulated current register.
 *
 * Writing the register also clears the LEARNF and the capacity related flags.
 *
 * @param dev        ds2782 device object
 * @param charge_mah Charge in milliampere-hours [mAh]
 * @return True on success, false otherwise
 */
bool ds2782_write_accumulated_current(const ds2782* const dev,
                                      float charge_mah);

/**
 * @brief Read the on-chip temperature.
 *
 * 11-bit two's complement value, 0.125 degC per LSB.
 *
 * @param      dev    ds2782 device object
 * @param[out] temp_c Temperature in degrees Celsius [degC]
 * @return True on success, false otherwise
 */
bool ds2782_read_temperature(const ds2782* const dev, float* const temp_c);

/**
 * @brief Read the remaining active relative capacity.
 *
 * @param      dev     ds2782 device object
 * @param[out] percent Remaining capacity in percent of the full capacity [%]
 * @return True on success, false otherwise
 */
bool ds2782_read_active_relative_capacity(const ds2782* const dev,
                                          uint8_t* const percent);

/**
 * @brief Read the remaining standby relative capacity.
 *
 * @param      dev     ds2782 device object
 * @param[out] percent Remaining capacity in percent of the full capacity [%]
 * @return True on success, false otherwise
 */
bool ds2782_read_standby_relative_capacity(const ds2782* const dev,
                                           uint8_t* const percent);

/**
 * @brief Read the remaining active absolute capacity.
 *
 * 1.6 mAh per LSB.
 *
 * @param      dev        ds2782 device object
 * @param[out] charge_mah Remaining capacity in milliampere-hours [mAh]
 * @return True on success, false otherwise
 */
bool ds2782_read_active_absolute_capacity(const ds2782* const dev,
                                          float* const charge_mah);

/**
 * @brief Read the remaining standby absolute capacity.
 *
 * 1.6 mAh per LSB.
 *
 * @param      dev        ds2782 device object
 * @param[out] charge_mah Remaining capacity in milliampere-hours [mAh]
 * @return True on success, false otherwise
 */
bool ds2782_read_standby_absolute_capacity(const ds2782* const dev,
                                           float* const charge_mah);

/**
 * @brief Read the age scalar, the ratio of the present to the initial capacity.
 *
 * 0.78125 % per LSB, a brand new cell reports 100 %.
 *
 * @param      dev     ds2782 device object
 * @param[out] percent Age scalar in percent [%]
 * @return True on success, false otherwise
 */
bool ds2782_read_age_scalar(const ds2782* const dev, float* const percent);

/**
 * @brief Read a block of the EEPROM shadow RAM.
 *
 * @param      dev      ds2782 device object
 * @param      reg_addr First EEPROM register to be read
 * @param[out] buf      Buffer to fill with the read bytes
 * @param      buf_size Number of bytes to read
 * @return True on success, false otherwise
 */
bool ds2782_read_eeprom(const ds2782* const dev, uint8_t reg_addr,
                        uint8_t* const buf, size_t buf_size);

/**
 * @brief Write a block of the EEPROM shadow RAM.
 *
 * The values stay volatile until ds2782_copy_eeprom_block() is called.
 *
 * @param dev      ds2782 device object
 * @param reg_addr First EEPROM register to be written
 * @param buf      Data to be written
 * @param buf_size Number of bytes to be written
 * @return True on success, false otherwise
 */
bool ds2782_write_eeprom(const ds2782* const dev, uint8_t reg_addr,
                         const uint8_t* const buf, size_t buf_size);

/**
 * @brief Copy the shadow RAM of an EEPROM block into the EEPROM cells.
 *
 * @param dev   ds2782 device object
 * @param block EEPROM block to be copied
 * @return True on success, false otherwise
 */
bool ds2782_copy_eeprom_block(const ds2782* const dev,
                              ds2782_eeprom_block block);

/**
 * @brief Recall the EEPROM cells of a block into the shadow RAM.
 *
 * @param dev   ds2782 device object
 * @param block EEPROM block to be recalled
 * @return True on success, false otherwise
 */
bool ds2782_recall_eeprom_block(const ds2782* const dev,
                                ds2782_eeprom_block block);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Write bytes into ds2782 device
 *
 * @param dev      ds2782 device object
 * @param reg_addr Register address
 * @param buf      Data to be written
 * @param buf_size Number of bytes to be written
 * @return True on success, false otherwise
 */
extern bool ds2782_write(const ds2782* const dev, uint8_t reg_addr,
                         const uint8_t* buf, size_t buf_size);

/**
 * @brief Read bytes from ds2782 device
 *
 * @param      dev      ds2782 device object
 * @param      reg_addr Register address
 * @param[out] buf      Buffer to fill with read bytes
 * @param      buf_size Number of bytes to read
 * @return True on success, false otherwise
 */
extern bool ds2782_read(const ds2782* const dev, uint8_t reg_addr,
                        uint8_t* buf, size_t buf_size);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void ds2782_delay_ms(uint32_t delay_ms);


#ifdef __cplusplus
}
#endif

#endif /* __DS2782_H__ */
