/**
 *  @brief:   MPU6050 6-axis motion tracking device (gyroscope + accelerometer) library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "mpu6050-mbed.h"

// mbed I2C API expects the address shifted to the 8-bit form
#define MPU6050_MBED_ADDR(dev) ((int)((dev)->i2c_addr) << 1)

//-----------------------------------------------------------------------------
bool mpu6050_mbed_init(mpu6050* const dev, mpu6050_mbed* const mbed_dev)
{
    if (!dev || !mbed_dev || !mbed_dev->i2c)
    {
        return false;
    }

    if (mbed_dev->i2c_freq != 0)
    {
        mbed_dev->i2c->frequency(mbed_dev->i2c_freq);
    }
    dev->platform_dev = mbed_dev;
    return mpu6050_init(dev);
}

//-----------------------------------------------------------------------------
bool mpu6050_write(const mpu6050* const dev, uint8_t reg_addr,
                   const uint8_t* buf, size_t buf_size)
{
    const mpu6050_mbed* const pd = (const mpu6050_mbed*)dev->platform_dev;

    // Keep the bus busy so register address and payload form a single transfer
    if (pd->i2c->write(MPU6050_MBED_ADDR(dev), (const char*)&reg_addr, 1, true) != 0)
    {
        return false;
    }
    return pd->i2c->write(MPU6050_MBED_ADDR(dev), (const char*)buf, buf_size) == 0;
}

//-----------------------------------------------------------------------------
bool mpu6050_read(const mpu6050* const dev, uint8_t reg_addr,
                  uint8_t* buf, size_t buf_size)
{
    const mpu6050_mbed* const pd = (const mpu6050_mbed*)dev->platform_dev;

    if (pd->i2c->write(MPU6050_MBED_ADDR(dev), (const char*)&reg_addr, 1, true) != 0)
    {
        return false;
    }
    return pd->i2c->read(MPU6050_MBED_ADDR(dev), (char*)buf, buf_size) == 0;
}

//-----------------------------------------------------------------------------
void mpu6050_delay_ms(uint32_t delay_ms)
{
    wait_us(delay_ms * 1000);
}

//-----------------------------------------------------------------------------
