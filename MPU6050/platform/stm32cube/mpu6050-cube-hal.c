/**
 *  @brief:   MPU6050 6-axis motion tracking device (gyroscope + accelerometer) library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "mpu6050-cube-hal.h"

#define MPU6050_CUBE_TIMEOUT_MS 1000

// STM32 HAL expects the address shifted to the 8-bit form
#define MPU6050_CUBE_ADDR(dev) ((uint16_t)((dev)->i2c_addr) << 1)

//-----------------------------------------------------------------------------
bool mpu6050_cube_hal_init(mpu6050* const dev,
                           mpu6050_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->i2c)
    {
        return false;
    }

    dev->platform_dev = cube_hal_dev;
    return mpu6050_init(dev);
}

//-----------------------------------------------------------------------------
bool mpu6050_write(const mpu6050* const dev, uint8_t reg_addr,
                   const uint8_t* buf, size_t buf_size)
{
    const mpu6050_cube_hal* const pd =
        (const mpu6050_cube_hal*)dev->platform_dev;

    return HAL_I2C_Mem_Write(pd->i2c, MPU6050_CUBE_ADDR(dev), reg_addr,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf,
                             (uint16_t)buf_size,
                             MPU6050_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
bool mpu6050_read(const mpu6050* const dev, uint8_t reg_addr,
                  uint8_t* buf, size_t buf_size)
{
    const mpu6050_cube_hal* const pd =
        (const mpu6050_cube_hal*)dev->platform_dev;

    return HAL_I2C_Mem_Read(pd->i2c, MPU6050_CUBE_ADDR(dev), reg_addr,
                            I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)buf_size,
                            MPU6050_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
void mpu6050_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
