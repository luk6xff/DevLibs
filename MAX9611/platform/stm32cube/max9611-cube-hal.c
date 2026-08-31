/**
 *  @brief:   MAX9611 high-side current-sense amplifier with 12-bit ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "max9611-cube-hal.h"

#define MAX9611_CUBE_TIMEOUT_MS 1000

// STM32 HAL expects the address shifted to the 8-bit form
#define MAX9611_CUBE_ADDR(dev) ((uint16_t)((dev)->i2c_addr) << 1)

//-----------------------------------------------------------------------------
bool max9611_cube_hal_init(max9611* const dev,
                           max9611_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->i2c)
    {
        return false;
    }

    dev->platform_dev = cube_hal_dev;
    return max9611_init(dev);
}

//-----------------------------------------------------------------------------
bool max9611_write(const max9611* const dev, uint8_t reg_addr,
                   const uint8_t* buf, size_t buf_size)
{
    const max9611_cube_hal* const pd =
        (const max9611_cube_hal*)dev->platform_dev;

    return HAL_I2C_Mem_Write(pd->i2c, MAX9611_CUBE_ADDR(dev), reg_addr,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf,
                             (uint16_t)buf_size,
                             MAX9611_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
bool max9611_read(const max9611* const dev, uint8_t reg_addr,
                  uint8_t* buf, size_t buf_size)
{
    const max9611_cube_hal* const pd =
        (const max9611_cube_hal*)dev->platform_dev;

    return HAL_I2C_Mem_Read(pd->i2c, MAX9611_CUBE_ADDR(dev), reg_addr,
                            I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)buf_size,
                            MAX9611_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
void max9611_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
