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
#include <string.h>


#define MAX44009_I2C_ADDRESS1    0x4A  // A0 PIN is conected to GND
#define MAX44009_I2C_ADDRESS2    0x4B  // A0 PIN is conected to VDD

/**
 * @brief max44009 dev object
 */
typedef struct
{
    uint8_t i2c_addr;   // I2C memory address
    void* platform_dev;
} max44009;

/**
 * @brief Initialize eeprom.
 *
 * @param dev max44009 device object
 * @return True on success, false otherwise
 */
bool max44009_init(max44009* const dev);

/**
 * @brief Read ambient light luminance.
 *
 * @param dev  max44009 device object
 * @param lux  Ambient light luminance in lux [lx]
 * @return True on success, false otherwise
 */
bool max44009_get_lux(const max44009* const dev, float* lux);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Write bytes into max44009 device
 *
 * @param       dev      max44009 device object
 * @param       reg_addr Register address.
 * @param       buf      Data to be written
 * @param       buf_size Number of bytes to be written.
 * @return True on success, false otherwise
 */
extern bool max44009_write(const max44009* const dev, uint8_t reg_addr,
                           const uint8_t* buf, size_t buf_size);

/**
 * @brief Read bytes from max44009 device
 *
 * @param       dev      max44009 device object
 * @param       reg_addr Register address.
 * @param[in]   buf      Buffer to fill with read bytes.
 * @param       buf_size Number of bytes to read (32 bytes max).
 * @retval Status value
 */
extern bool max44009_read(const max44009* const dev, uint8_t reg_addr,
                          uint8_t* buf, size_t buf_size);


#ifdef __cplusplus
}
#endif

#endif /* __MAX44009_H__ */

