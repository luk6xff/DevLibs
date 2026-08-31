/**
 *  @brief:   MPU6050 6-axis motion tracking device (gyroscope + accelerometer) library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MPU6050_ARDUINO_H__
#define __MPU6050_ARDUINO_H__

#include "../../mpu6050.h"
#include <Arduino.h>
#include <Wire.h>

/**
 * @brief MPU6050 arduino specific dev object
 */
typedef struct
{
    TwoWire* i2c;      // I2C bus instance, e.g. &Wire
    uint32_t i2c_freq; // I2C clock in Hz, 0 keeps the current bus setting
} mpu6050_arduino;

/**
 * @brief Initialize MPU6050 device with ARDUINO dependent parameters.
 *
 * @param dev         mpu6050 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool mpu6050_arduino_init(mpu6050* const dev,
                          mpu6050_arduino* const arduino_dev);


#endif // __MPU6050_ARDUINO_H__
