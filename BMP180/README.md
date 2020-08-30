# BMP180 digital pressure sensor library

## Description
BMP180 digital pressure sensor platform independent library

## Usage
Simple demo code how to use the library on [MBED] platform
```cpp

#include "BMP180/platform/bmp180-mbed.h"

// PINOUT FOR MBED BOARD_DISCO_L476VG
// BMP180
#define BMP180_I2C_ADDR         BMP180_DEFAULT_I2C_ADDRESS
#define BMP180_SDA              PB_7
#define BMP180_SCL              PB_6
int main(void)
{
    float temperature, pressure;
    I2C i2c(BMP180_SDA, BMP180_SCL);
    bmp180_mbed mbed_dev =
    {
        .i2c = &i2c,
    };

    bmp180 dev =
    {
        .i2c_addr = BMP180_DEFAULT_I2C_ADDRESS,
        .oss = BMP180_OSS_NORMAL,
        .altitude = 64, // 64 meters altitude compensation
    };
    bmp180_mbed_init(dev, mbed_dev);

    while(1)
    {
        // Read data
        bmp180_read_data(&bmp180_dev, &temperature, &pressure);
        printf("T:%3.1f, P:%3.1f", temperature, pressure);
    }

    return 0;
}
```

## Porting to different platforms
If you want to port this library on other platform, the only thing you have to do is to implement HW/Platform dependent functions from `bmp180.h` file as it is done for example for [MBED] platform in `platform/bmp180-mbed.cpp` file.


## Author
* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]