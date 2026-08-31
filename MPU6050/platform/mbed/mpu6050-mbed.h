/**
 *  @brief:   MPU6050 6-axis motion tracking device (gyroscope + accelerometer) library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MPU6050_MBED_H__
#define __MPU6050_MBED_H__

#include "mbed.h"
#include "../../mpu6050.h"

/**
 * @brief MPU6050 mbed specific dev object
 */
typedef struct
{
    I2C* i2c;          // I2C bus instance
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
} mpu6050_mbed;


/**
 * @brief Initialize MPU6050 device with MBED dependent parameters.
 *
 * @param dev      mpu6050 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool mpu6050_mbed_init(mpu6050* const dev, mpu6050_mbed* const mbed_dev);


#endif /*__MPU6050_MBED_H__ */
