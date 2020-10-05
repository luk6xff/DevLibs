
/**
 *  @brief:   MAX44009 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "max44009-cube-hal.h"



//-----------------------------------------------------------------------------
bool max44009_cube_hal_init(max44009* const dev, max44009_cube_hal *const cube_hal_dev);
{
    dev->platform_dev = cube_hal_dev;
    return max44009_init(dev);
}

//-----------------------------------------------------------------------------
bool max44009_write(const max44009* const dev, uint8_t reg_addr,
                    const uint8_t* buf, size_t buf_size)
{
    const max44009_cube_hal* const pd = (max44009_cube_hal*)dev->platform_dev;
    int ack = HAL_I2C_Master_Transmit(pd->i2c, dev->i2c_addr, &reg_addr, 1, 1000);
    if (ack != 0)
    {
        return false;
    }
    int ack = HAL_I2C_Master_Transmit(pd->i2c, dev->i2c_addr, buf, buf_size, 1000);
    if (ack != 0)
    {
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool max44009_read(const max44009* const dev, uint8_t reg_addr,
                   uint8_t* buf, size_t buf_size)
{
    const max44009_cube_hal* const pd = (max44009_cube_hal*)dev->platform_dev;
    int ack = HAL_I2C_Master_Transmit(pd->i2c, dev->i2c_addr, &reg_addr, 1, 1000);
    if (ack != 0)
    {
        return false;
    }
    ack = HAL_I2C_Master_Receive(pd->i2c, dev->i2c_addr, buf, buf_size, 1000);
    if (ack != 0)
    {
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
void max44009_enable_wp(bool enable)
{
    if (enable)
    {
    	HAL_GPIO_WritePin(_wp_port, _wp_pin, GPIO_PIN_SET);
        return;
    }
	HAL_GPIO_WritePin(_wp_port, _wp_pin, GPIO_PIN_RESET);
}


//-----------------------------------------------------------------------------
void max44009_delay_ms(uint32_t delay_ms)
{
	  uint32_t tickstart_ms = HAL_GetTick();
	  while((HAL_GetTick()-tickstart_ms) < delay_ms);
}

//-----------------------------------------------------------------------------

#endif