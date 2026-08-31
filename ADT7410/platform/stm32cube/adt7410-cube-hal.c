/**
 *  @brief:   ADT7410 digital temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "adt7410-cube-hal.h"

#define ADT7410_CUBE_TIMEOUT_MS 1000

// STM32 HAL expects the address shifted to the 8-bit form
#define ADT7410_CUBE_ADDR(dev) ((uint16_t)((dev)->i2c_addr) << 1)

//-----------------------------------------------------------------------------
bool adt7410_cube_hal_init(adt7410* const dev,
                           adt7410_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->i2c)
    {
        return false;
    }

    dev->platform_dev = cube_hal_dev;
    return adt7410_init(dev);
}

//-----------------------------------------------------------------------------
bool adt7410_write(const adt7410* const dev, uint8_t reg_addr,
                   const uint8_t* buf, size_t buf_size)
{
    const adt7410_cube_hal* const pd = (const adt7410_cube_hal*)dev->platform_dev;

    if (buf_size == 0)
    {
        // Command-only access, e.g. the software reset address
        return HAL_I2C_Master_Transmit(pd->i2c, ADT7410_CUBE_ADDR(dev), &reg_addr,
                                       1, ADT7410_CUBE_TIMEOUT_MS) == HAL_OK;
    }

    return HAL_I2C_Mem_Write(pd->i2c, ADT7410_CUBE_ADDR(dev), reg_addr,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf,
                             (uint16_t)buf_size,
                             ADT7410_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
bool adt7410_read(const adt7410* const dev, uint8_t reg_addr,
                  uint8_t* buf, size_t buf_size)
{
    const adt7410_cube_hal* const pd = (const adt7410_cube_hal*)dev->platform_dev;

    return HAL_I2C_Mem_Read(pd->i2c, ADT7410_CUBE_ADDR(dev), reg_addr,
                            I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)buf_size,
                            ADT7410_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
void adt7410_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
