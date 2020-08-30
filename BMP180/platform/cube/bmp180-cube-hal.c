
/**
 *  @brief:  Implementation of a BMP180 platform dependent [STM32 CUBE HAL] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-08-12
 */

#include "bmp180-cube-hal.h"

//------------------------------------------------------------------------------
bool bmp180_cube_hal_init(bmp180 *const dev, bmp180_cube_hal* const cube_hal_dev)
{
    dev->platform_dev = cube_hal_dev;
    return bmp180_init(dev);
}

//------------------------------------------------------------------------------
bool bmp180_cube_hal_deinit(void)
{
    // Empty
    return true;
}

//-----------------------------------------------------------------------------
bool bmp180_write(bmp180 *const dev, const uint8_t* buf, const size_t buf_size)
{
    bmp180_cube_hal* const pd = (bmp180_cube_hal*)dev->platform_dev;
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(pd->i2c, dev->i2c_addr, buf, buf_size, 1000);
    if (status != HAL_OK)
    {
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool bmp180_read(bmp180 *const dev, uint8_t* buf, const size_t buf_size)
{
    bmp180_cube_hal* const pd = (bmp180_cube_hal*)dev->platform_dev;
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(pd->i2c, dev->i2c_addr, buf, buf_size, 1000);
    if (status != HAL_OK)
    {
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
void bmp180_delay_ms(uint32_t delay_ms)
{
    uint32_t tickstart_ms = HAL_GetTick();
    while ((HAL_GetTick()-tickstart_ms) < delay_ms);
}

//-----------------------------------------------------------------------------
