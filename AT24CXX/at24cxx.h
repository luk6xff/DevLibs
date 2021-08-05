/**
 *  @brief:   EEPROM AT24CXX library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2019-11-25
 *  @license: MIT
 */

#ifndef __AT24CXX_H__
#define __AT24CXX_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>



#define AT24CXX_ADDR  0xA0

typedef enum
{
    AT24CXX_NOERR = 0,
    AT24CXX_BAD_ADDR,       // bad I2C device address
    AT24CXX_I2C_ERR,        // I2C error (nack)
    AT24CXX_PARAM_ERR,      // Invalid parameter
    AT24CXX_OUT_OF_RANGE,   // Data addr out of memory range
    AT24CXX_ERR             // Other error
} at24cxx_status;

/** AT24CXX device type **/
typedef enum
{
    AT24C01 = 0,
    AT24C02,
    AT24C04,
    AT24C08,
    AT24C16,
    AT24C32,
    AT24C64,
    AT24C128,
    AT24C256,
    AT24C512,
    AT24C1024,
    AT24CINVALID,
} at24cxx_type;


/** AT24CXX device type description **/
typedef struct
{
    at24cxx_type type;      // Memory type
    uint32_t size;          // Memory size in bytes
    uint32_t page_size;     // Memory Page size in bytes
    uint8_t  word_addr_len; // Memory word address length in bytes
} at24cxx_mem;


/** AT24CXX dev object **/
typedef struct
{
    at24cxx_type type;   // Memory type
    uint8_t      addr;   // I2C memory address
    void* platform_dev;
} at24cxx;

/** AT24CXX devices **/
static const at24cxx_mem at24cxx_devices[] =
{
    { AT24C01,   128,    8,   1 },  //128 bytes=1Kbit, 8bytes=64bit
    { AT24C02,   256,    8,   1 },
    { AT24C04,   512,    16,  1 },
    { AT24C08,   1024,   16,  1 },
    { AT24C16,   2048,   16,  1 },
    { AT24C32,   4096,   32,  2 },
    { AT24C64,   8192,   32,  2 },
    { AT24C128,  16384,  64,  2 },
    { AT24C256,  32768,  64,  2 },
    { AT24C512,  65536,  128, 2 },
    { AT24C1024, 131072, 128, 2 },
};



/**
 * @brief Initialize eeprom.
 *
 * @param  dev at24cxx device object
 * @param i2c_addr  I2C chip address. (A2, A1, A0 pins - GND=0, VCC=1)
 * @param mem_type  memory type
 */
at24cxx_status at24cxx_init(at24cxx* const dev, uint8_t i2c_addr_pins);

/**
 * @brief Write data into memory.
 *
 * @param dev        at24cxx device object
 * @param addr       Start address.
 * @param data       Bytes data to be written into memory.
 * @param data_size  Bytes data size.
 */
at24cxx_status at24cxx_write(const at24cxx* const dev, uint32_t addr,
                              const uint8_t* data, uint32_t data_size);


/**
 * @brief Read data from the memory.
 *
 * @param dev   at24cxx device object
 * @param addr  Start address.
 * @param data  Bytes to be read from memory.
 * @param data_size  Bytes data size.
 */
at24cxx_status at24cxx_read(const at24cxx* const dev, uint32_t addr,
                            uint8_t* data, uint32_t data_size);


/**
 * @brief Check if write or read operation can succeed.
 *
 * @param  dev at24cxx device object
 * @param  addr     Start address.
 * @param  buf_size  Number of bytes to be written/read.
 * @retval Status value
 */
bool at24cxx_check_space(const at24cxx* const dev, uint32_t addr, size_t size);


/**
 * @brief Get eeprom size in bytes
 *
 * @param  dev at24cxx device object
 * @param none
 * @return size in bytes (uint32_t)
 */
uint32_t at24cxx_size(at24cxx* const dev);


/**
 * @brief Clear eeprom (write with 0)
 *
 * @param  dev at24cxx device object
 */
void at24cxx_clear(at24cxx* const dev);


/**
 * @brief Wait for the device being ready
 *
 * @param  dev at24cxx device object
 */
void at24cxx_wait_for_ready(const at24cxx* const dev);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------
/**
 * @brief Init IO
 *
 * @param       dev      at24cxx device object
 * @retval Status value
 */
extern at24cxx_status at24cxx_io_init(at24cxx* const dev);

/**
 * @brief Deinit IO
 *
 * @param       dev      at24cxx device object
 * @retval Status value
 */
extern at24cxx_status at24cxx_io_deinit(at24cxx* const dev);

/**
 * @brief Write bytes into memory via I2C bus.
 *
 * @param       dev      at24cxx device object
 * @param       addr     Start address.
 * @param       buf      Data to be written
 * @param       buf_size  Number of bytes to be written (32 bytes max).
 * @retval Status value
 */
extern at24cxx_status at24cxx_write_buffer(const at24cxx* const dev, uint32_t addr,
                                           const uint8_t* buf, size_t buf_size);

/**
 * @brief Read bytes from memory via I2C bus.
 *
 * @param       dev      at24cxx device object
 * @param       addr     Start address.
 * @param[in]   buf      Buffer to fill with read bytes.
 * @param       buf_size  Number of bytes to read (32 bytes max).
 * @retval Status value
 */
extern at24cxx_status at24cxx_read_buffer(const at24cxx* const dev, uint32_t addr,
                                          uint8_t* buf, size_t buf_size);

/**
 * @brief Enable/Disable Write protection pin.
 *
 * @param       dev      at24cxx device object
 * @param enable  enables if true ot disables if false WriteProtectionPin
 */
extern void at24cxx_enable_wp(at24cxx* const dev, bool enable);

/**
 * @brief Miliseconds delay.
 *
 * @param delay_ms: Delay in miliseconds.
 */
extern void at24cxx_delay_ms(uint32_t delay_ms);


#ifdef __cplusplus
}
#endif

#endif /*__AT24CXX_H__ */

