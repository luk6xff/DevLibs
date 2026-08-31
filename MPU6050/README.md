# MPU6050 6-axis motion tracking device

## Description
Platform independent driver for the InvenSense MPU-6050 motion tracking device.

The core (`mpu6050.c` / `mpu6050.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Device: 6-axis IMU, 3-axis MEMS gyroscope + 3-axis MEMS accelerometer,
  on-chip temperature sensor and a 1024 byte FIFO
* Interface: I2C (up to 400 kHz)
* Addresses: `0x68` (AD0 -> GND), `0x69` (AD0 -> VDD)
* Gyroscope ranges: +/-250, +/-500, +/-1000, +/-2000 dps
* Accelerometer ranges: +/-2, +/-4, +/-8, +/-16 g
* Datasheets: see `docs/`

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/mpu6050-arduino.{h,cpp}` |
| Mbed | `platform/mbed/mpu6050-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/mpu6050-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "MPU6050/platform/arduino/mpu6050-arduino.h"

mpu6050 dev;
mpu6050_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.i2c = &Wire;
    arduino_dev.i2c_freq = 400000;
    dev.i2c_addr = MPU6050_I2C_ADDR_AD0_LOW;

    if (!mpu6050_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("MPU6050 init failed!");
        return;
    }

    // Override the defaults applied by mpu6050_init()
    mpu6050_config cfg;
    cfg.sample_rate_div = 4; // 1 kHz / (1 + 4) = 200 Hz
    cfg.dlpf = MPU6050_DLPF_BW_42HZ;
    cfg.gyro_range = MPU6050_GYRO_RANGE_500DPS;
    cfg.accel_range = MPU6050_ACCEL_RANGE_4G;
    cfg.clock_source = MPU6050_CLOCK_PLL_X_GYRO;

    if (!mpu6050_set_config(&dev, &cfg))
    {
        Serial.println("MPU6050 configuration failed!");
    }
}

void loop()
{
    mpu6050_data data;
    if (mpu6050_read_all(&dev, &data))
    {
        Serial.print("Accel [g]: ");
        Serial.print(data.accel.x); Serial.print(", ");
        Serial.print(data.accel.y); Serial.print(", ");
        Serial.println(data.accel.z);

        Serial.print("Gyro [dps]: ");
        Serial.print(data.gyro.x); Serial.print(", ");
        Serial.print(data.gyro.y); Serial.print(", ");
        Serial.println(data.gyro.z);

        Serial.print("Temperature [degC]: ");
        Serial.println(data.temperature);
    }
    delay(1000);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `mpu6050.h`:

* `mpu6050_write()`
* `mpu6050_read()`
* `mpu6050_delay_ms()`

and provide a `mpu6050_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/mpu6050-arduino.cpp`
for a reference implementation.

## Not implemented
* **DMP (Digital Motion Processor)**. The previous mbed version of this library
  shipped InvenSense's `inv_mpu_dmp_motion_driver` port together with the 3062
  byte DMP firmware image. It was dropped rather than half-ported because:
  * the firmware image and the surrounding code carry an
    `InvenSense Corporation, All Rights Reserved` notice with no accompanying
    licence file, which cannot be redistributed under this repository's MIT
    licence;
  * it depends on C++ helper classes (`Quaternion`, `VectorFloat`) and on a
    millisecond time source that the old port had stubbed out to nothing, so
    every rate and timeout in it was already non-functional.

  If you need the DMP, use InvenSense's Embedded MotionDriver release directly;
  its memory bank access can be layered on top of the `mpu6050_write()` /
  `mpu6050_read()` hooks of this library.
* **Auxiliary I2C master mode** (external magnetometer on the AUX bus) and the
  associated `I2C_SLVx` registers.
* **Low-power accelerometer-only cycle mode.**

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
