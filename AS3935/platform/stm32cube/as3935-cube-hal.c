/**
 *  @brief:   AS3935 Franklin lightning sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "as3935-cube-hal.h"

#define AS3935_CUBE_TIMEOUT_MS 1000

// STM32 HAL expects the address shifted to the 8-bit form
#define AS3935_CUBE_ADDR(dev) ((uint16_t)((dev)->i2c_addr) << 1)

//-----------------------------------------------------------------------------
bool as3935_cube_hal_init(as3935* const dev,
                          as3935_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->i2c || !cube_hal_dev->irq_port)
    {
        return false;
    }

    dev->platform_dev = cube_hal_dev;
    return as3935_init(dev);
}

//-----------------------------------------------------------------------------
bool as3935_write(const as3935* const dev, uint8_t reg_addr,
                  const uint8_t* buf, size_t buf_size)
{
    const as3935_cube_hal* const pd = (const as3935_cube_hal*)dev->platform_dev;

    if (buf_size == 0)
    {
        // Command-only access, no payload follows the register address
        return HAL_I2C_Master_Transmit(pd->i2c, AS3935_CUBE_ADDR(dev), &reg_addr,
                                       1, AS3935_CUBE_TIMEOUT_MS) == HAL_OK;
    }

    return HAL_I2C_Mem_Write(pd->i2c, AS3935_CUBE_ADDR(dev), reg_addr,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf,
                             (uint16_t)buf_size,
                             AS3935_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
bool as3935_read(const as3935* const dev, uint8_t reg_addr,
                 uint8_t* buf, size_t buf_size)
{
    const as3935_cube_hal* const pd = (const as3935_cube_hal*)dev->platform_dev;

    return HAL_I2C_Mem_Read(pd->i2c, AS3935_CUBE_ADDR(dev), reg_addr,
                            I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)buf_size,
                            AS3935_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
bool as3935_get_irq_pin(const as3935* const dev)
{
    const as3935_cube_hal* const pd = (const as3935_cube_hal*)dev->platform_dev;

    return HAL_GPIO_ReadPin(pd->irq_port, pd->irq_pin) == GPIO_PIN_SET;
}

//-----------------------------------------------------------------------------
void as3935_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
