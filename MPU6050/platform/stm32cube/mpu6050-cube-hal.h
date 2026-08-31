/**
 *  @brief:   MPU6050 6-axis motion tracking device (gyroscope + accelerometer) library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MPU6050_CUBE_HAL_H__
#define __MPU6050_CUBE_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "../../mpu6050.h"

/**
 * @brief MPU6050 cube hal specific dev object
 */
typedef struct
{
    I2C_HandleTypeDef* i2c; // I2C peripheral handle
} mpu6050_cube_hal;

/**
 * @brief Initialize MPU6050 device with STM32 CUBE HAL dependent parameters.
 *
 * @param dev          mpu6050 device object
 * @param cube_hal_dev Cube HAL specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool mpu6050_cube_hal_init(mpu6050* const dev,
                           mpu6050_cube_hal* const cube_hal_dev);

#ifdef __cplusplus
}
#endif

#endif /*__MPU6050_CUBE_HAL_H__ */
