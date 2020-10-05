/**
 *  @brief:   BH1750 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-10-02
 *  @license: MIT
 */

#ifndef __BH1750_H__
#define __BH1750_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>


/**
 * @brief bh1750 i2c addresses (without R/W bit)
 */
#define BH1750_I2C_ADDRESS1    0x23  // ADDR PIN is conected to GND (0x46 including R/W bit)
#define BH1750_I2C_ADDRESS2    0x5C  // ADDR PIN is conected to VDD (0xB8 including R/W bit)


/**
 * @brief bh1750 Instruction Set - state
 */
typedef enum
{
    BH1750_STATE_POWER_DOWN = 0x00, //!< Power down instruction
    BH1750_STATE_POWER_ON   = 0x01, //!< Power on instruction
    BH1750_STATE_RESET      = 0x07, //!< Reset instruction
} bh1750_state;


/**
 * @brief bh1750 Instruction Set - state
 */
typedef enum
{
    BH1750_MODE_CONT_MID_RES     = 0x10, //!< Continues mode: 0.5 lx resolution
    BH1750_MODE_CONT_HIGH_RES    = 0x11, //!< Continues mode: 1 lx resolution
    BH1750_MODE_CONT_LOW_RES     = 0x13, //!< Continues mode: 4 lx resolution
    BH1750_MODE_ONETIME_MID_RES  = 0x20, //!< One-time mode: 0.5 lx resolution
    BH1750_MODE_ONETIME_HIGH_RES = 0x21, //!< One-time mode: 1 lx resolution
    BH1750_MODE_ONETIME_LOW_RES  = 0x23, //!< One-time mode: 4 lx resolution
} bh1750_mode;

/**
 * @brief bh1750 dev object
 */
typedef struct
{
    uint8_t i2c_addr;   // I2C address
    void *platform_dev;
} bh1750;


/**
 * @brief Initialize eeprom.
 *
 * @param dev bh1750 device object
 * @return True on success, false otherwise
 */
bool bh1750_init(bh1750 *const dev);

/**
 * @brief Set BH1750 device state
 *
 * @param dev bh1750 device object
 * @param state bh1750 state
 * @return True on success, false otherwise
 */
bool bh1750_set_state(bh1750 *const dev, bh1750_state state);

/**
 * @brief Set BH1750 device mode
 *
 * @param dev bh1750 device object
 * @param mode bh1750 mode
 * @return True on success, false otherwise
 */
bool bh1750_set_mode(bh1750 *const dev, bh1750_mode mode);

/**
 * @brief Read ambient light luminance.
 *
 * @param dev  bh1750 device object
 * @param lux  Ambient light luminance in lux [lx]
 * @return True on success, false otherwise
 */
bool bh1750_get_lux(const bh1750 *const dev, float *lux);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Write an instruction into bh1750 device
 *
 * @param  dev          bh1750 device object
 * @param  instruction  Instruction type.
 * @return True on success, false otherwise
 */
extern bool bh1750_write_instruction(bh1750 *const dev, uint8_t instruction);

/**
 * @brief Read bytes from bh1750 device
 *
 * @param     dev      bh1750 device object
 * @param[in] buf      Buffer to fill with read bytes.
 * @param     buf_size Number of bytes to read.
 * @retval    True on success, false otherwise.
 */
extern bool bh1750_read(const bh1750 *const dev, uint8_t *buf, size_t buf_size);


#ifdef __cplusplus
}
#endif

#endif /* __BH1750_H__ */

