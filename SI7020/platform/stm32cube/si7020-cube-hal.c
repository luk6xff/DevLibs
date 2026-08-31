/**
 *  @brief:   SI7020 relative humidity and temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "si7020-cube-hal.h"

#define SI7020_CUBE_TIMEOUT_MS 1000

// STM32 HAL expects the address shifted to the 8-bit form
#define SI7020_CUBE_ADDR(dev) ((uint16_t)((dev)->i2c_addr) << 1)

//-----------------------------------------------------------------------------
bool si7020_cube_hal_init(si7020* const dev,
                          si7020_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->i2c)
    {
        return false;
    }

    dev->platform_dev = cube_hal_dev;
    return si7020_init(dev);
}

//-----------------------------------------------------------------------------
bool si7020_write_cmd(const si7020* const dev, const uint8_t* cmd,
                      size_t cmd_size)
{
    const si7020_cube_hal* const pd = (const si7020_cube_hal*)dev->platform_dev;

    if (cmd_size > 1)
    {
        // A two byte command maps onto the memory addressed API: the first byte
        // plays the role of the memory address and the rest is the payload
        return HAL_I2C_Mem_Write(pd->i2c, SI7020_CUBE_ADDR(dev), cmd[0],
                                 I2C_MEMADD_SIZE_8BIT, (uint8_t*)&cmd[1],
                                 (uint16_t)(cmd_size - 1),
                                 SI7020_CUBE_TIMEOUT_MS) == HAL_OK;
    }

    // The device has no register map, a single byte command is sent as is
    return HAL_I2C_Master_Transmit(pd->i2c, SI7020_CUBE_ADDR(dev),
                                   (uint8_t*)cmd, (uint16_t)cmd_size,
                                   SI7020_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
bool si7020_read_raw(const si7020* const dev, uint8_t* buf, size_t buf_size)
{
    const si7020_cube_hal* const pd = (const si7020_cube_hal*)dev->platform_dev;

    // The device answers the previously latched command, nothing is sent here
    return HAL_I2C_Master_Receive(pd->i2c, SI7020_CUBE_ADDR(dev), buf,
                                  (uint16_t)buf_size,
                                  SI7020_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
void si7020_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
