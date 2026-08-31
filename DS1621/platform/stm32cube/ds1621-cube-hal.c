/**
 *  @brief:   DS1621 digital thermometer and thermostat library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "ds1621-cube-hal.h"

#define DS1621_CUBE_TIMEOUT_MS 1000

// STM32 HAL expects the address shifted to the 8-bit form
#define DS1621_CUBE_ADDR(dev) ((uint16_t)((dev)->i2c_addr) << 1)

//-----------------------------------------------------------------------------
bool ds1621_cube_hal_init(ds1621* const dev,
                          ds1621_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->i2c)
    {
        return false;
    }

    dev->platform_dev = cube_hal_dev;
    return ds1621_init(dev);
}

//-----------------------------------------------------------------------------
bool ds1621_write(const ds1621* const dev, uint8_t cmd,
                  const uint8_t* buf, size_t buf_size)
{
    const ds1621_cube_hal* const pd = (const ds1621_cube_hal*)dev->platform_dev;

    if (buf_size == 0)
    {
        // Command-only access, e.g. start/stop convert
        return HAL_I2C_Master_Transmit(pd->i2c, DS1621_CUBE_ADDR(dev), &cmd, 1,
                                       DS1621_CUBE_TIMEOUT_MS) == HAL_OK;
    }

    return HAL_I2C_Mem_Write(pd->i2c, DS1621_CUBE_ADDR(dev), cmd,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf,
                             (uint16_t)buf_size,
                             DS1621_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
bool ds1621_read(const ds1621* const dev, uint8_t cmd,
                 uint8_t* buf, size_t buf_size)
{
    const ds1621_cube_hal* const pd = (const ds1621_cube_hal*)dev->platform_dev;

    return HAL_I2C_Mem_Read(pd->i2c, DS1621_CUBE_ADDR(dev), cmd,
                            I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)buf_size,
                            DS1621_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
void ds1621_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
