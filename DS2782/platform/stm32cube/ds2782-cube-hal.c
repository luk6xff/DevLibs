/**
 *  @brief:   DS2782 stand-alone battery fuel gauge library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "ds2782-cube-hal.h"

#define DS2782_CUBE_TIMEOUT_MS 1000

// STM32 HAL expects the address shifted to the 8-bit form
#define DS2782_CUBE_ADDR(dev) ((uint16_t)((dev)->i2c_addr) << 1)

//-----------------------------------------------------------------------------
bool ds2782_cube_hal_init(ds2782* const dev,
                          ds2782_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->i2c)
    {
        return false;
    }

    dev->platform_dev = cube_hal_dev;
    return ds2782_init(dev);
}

//-----------------------------------------------------------------------------
bool ds2782_write(const ds2782* const dev, uint8_t reg_addr,
                  const uint8_t* buf, size_t buf_size)
{
    const ds2782_cube_hal* const pd = (const ds2782_cube_hal*)dev->platform_dev;

    if (buf_size == 0)
    {
        // Command-only access, no payload follows the register address
        return HAL_I2C_Master_Transmit(pd->i2c, DS2782_CUBE_ADDR(dev), &reg_addr,
                                       1, DS2782_CUBE_TIMEOUT_MS) == HAL_OK;
    }

    return HAL_I2C_Mem_Write(pd->i2c, DS2782_CUBE_ADDR(dev), reg_addr,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf,
                             (uint16_t)buf_size,
                             DS2782_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
bool ds2782_read(const ds2782* const dev, uint8_t reg_addr,
                 uint8_t* buf, size_t buf_size)
{
    const ds2782_cube_hal* const pd = (const ds2782_cube_hal*)dev->platform_dev;

    return HAL_I2C_Mem_Read(pd->i2c, DS2782_CUBE_ADDR(dev), reg_addr,
                            I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)buf_size,
                            DS2782_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
void ds2782_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
