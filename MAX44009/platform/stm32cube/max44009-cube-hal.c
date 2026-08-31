
/**
 *  @brief:   MAX44009 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "max44009-cube-hal.h"

#define MAX44009_CUBE_TIMEOUT_MS 1000

// STM32 HAL expects the address shifted to the 8-bit form
#define MAX44009_CUBE_ADDR(dev) ((uint16_t)((dev)->i2c_addr) << 1)

//-----------------------------------------------------------------------------
bool max44009_cube_hal_init(max44009* const dev,
                            max44009_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->i2c)
    {
        return false;
    }

    dev->platform_dev = cube_hal_dev;
    return max44009_init(dev);
}

//-----------------------------------------------------------------------------
bool max44009_write(const max44009* const dev, uint8_t reg_addr,
                    const uint8_t* buf, size_t buf_size)
{
    const max44009_cube_hal* const pd =
        (const max44009_cube_hal*)dev->platform_dev;

    return HAL_I2C_Mem_Write(pd->i2c, MAX44009_CUBE_ADDR(dev), reg_addr,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf,
                             (uint16_t)buf_size,
                             MAX44009_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
bool max44009_read(const max44009* const dev, uint8_t reg_addr,
                   uint8_t* buf, size_t buf_size)
{
    const max44009_cube_hal* const pd =
        (const max44009_cube_hal*)dev->platform_dev;

    return HAL_I2C_Mem_Read(pd->i2c, MAX44009_CUBE_ADDR(dev), reg_addr,
                            I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)buf_size,
                            MAX44009_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
void max44009_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
