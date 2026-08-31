# BMP180 digital barometric pressure and temperature sensor

## Description
Platform independent driver for the Bosch BMP180 barometric pressure and
temperature sensor.

The core (`bmp180.c` / `bmp180.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: I2C
* Default address: `BMP180_DEFAULT_I2C_ADDRESS` (`0xEF`, 8-bit read address)
* Oversampling (`bmp180_oversampling_t`): ultra low power (1 sample, 4.5 ms),
  normal (2 samples, 7.5 ms), high resolution (4 samples, 13.5 ms),
  ultra high resolution (8 samples, 25.5 ms)
* Altitude compensation is stored in `dev.altitude` and applied to the pressure
  reading

## Supported platforms
| Platform | Files |
|---|---|
| Mbed | `platform/mbed/bmp180-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/bmp180-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `MBED` platform:

```cpp
#include "BMP180/platform/mbed/bmp180-mbed.h"

// PINOUT FOR MBED BOARD_DISCO_L476VG
#define BMP180_SDA PB_7
#define BMP180_SCL PB_6

int main()
{
    I2C i2c(BMP180_SDA, BMP180_SCL);

    bmp180 dev;
    bmp180_mbed mbed_dev;
    mbed_dev.i2c = &i2c;

    dev.i2c_addr = BMP180_DEFAULT_I2C_ADDRESS;
    dev.oss = BMP180_OSS_NORMAL;
    dev.altitude = 64.0f; // 64 meters altitude compensation

    bmp180_mbed_init(&dev, &mbed_dev);

    while (true)
    {
        float temperature = 0.0f;
        float pressure = 0.0f;

        if (bmp180_read_data(&dev, &temperature, &pressure))
        {
            printf("T:%3.1f[C], P:%3.1f[hPa]\r\n", temperature, pressure);
        }
    }

    return 0;
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `bmp180.h`:

* `bmp180_write()`
* `bmp180_read()`
* `bmp180_delay_ms()`

and provide a `bmp180_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/mbed/bmp180-mbed.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
