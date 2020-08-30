# BME280 digital pressure, temperature and humidity sensor library

## Description
BME280 digital pressure sensor platform independent library

## Usage
Simple demo code how to use the library on [MBED] platform
```cpp

#include "BME280/platform/bme280-mbed.h"

// PINOUT FOR MBED BOARD_DISCO_L476VG
// BME280
#define BME280_I2C_ADDR         BME280_DEFAULT_I2C_ADDRESS
#define BME280_SDA              PB_7
#define BME280_SCL              PB_6
int main(void)
{
    float temperature, pressure, humidity, altitude;
    I2C i2c(BME280_SDA, BME280_SCL);
    bme280_mbed mbed_dev =
    {
        .i2c = &i2c,
    };

    bme280 dev =
    {
        .intf = BME280_INTF_I2C,
        .i2c_addr = BME280_DEFAULT_I2C_ADDRESS,
        .t_fine = 0,
        .t_fine_adjust = 0,
    };
    bme280_mbed_init(dev, mbed_dev);

    while(1)
    {
        // Read data
        bme280_read_data(dev, &temperature, &pressure, &humidity);
        printf("T:%3.1f[C], P:%3.1f[hPa], H:%3.1f[%%]\n\r", temperature, pressure/100.f, humidity);
        bme280_read_altitude(dev, SEA_LEVEL_PRESSURE_HPA, &altitude);
        printf("Altitude:%3.1f[m]\n\r", altitude);
    }

    return 0;
}
```

## Porting to different platforms
If you want to port this library on other platform, the only thing you have to do is to implement HW/Platform dependent functions from `bme280.h` file as it is done for example for [MBED] platform in `platform/bme280-mbed.cpp` file.


## Author
* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]