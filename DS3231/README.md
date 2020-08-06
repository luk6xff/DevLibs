# DS3231 RTC module library

## Description
DS3231 RTC module platform independent library

## Usage
Simple demo code how to use the library on [MBED] platform
```cpp

#include "DS3231/platform/ds3231-mbed.h"

// PINOUT FOR MBED BOARD_DISCO_L476VG
// DS3231
#define DS3231_I2C_ADDR         0x68
#define DS3231_SDA              PB_7
#define DS3231_SCL              PB_6
int main(void)
{
    I2C i2c(DS3231_SDA, DS3231_SCL);
    ds3231_mbed mbed_dev =
    {
        .i2c = &i2c,
    };

    ds3231 dev =
    {
        .i2c_addr = DS3231_I2C_ADDR;
    };
    ds3231_mbed_init(dev, mbed_dev);

    ds3231_time_t     time;
    ds3231_calendar_t cal;

    if (ds3231_get_time(&time) != 0)
    {
        return -1;
    }

    if (ds3231_get_calendar(&cal) != 0)
    {
        return -1;
    }

    printf("%d-%d-%d, %d:%d:%d", cal.year, cal.month, cal.date time.hours, time.minutes, time.seconds);
    return 0;
}
```

## Porting to different platforms
If you want to port this library on other platform, the only thing you have to do is to implement HW/Platform dependent functions from `ds3231.h` file as it is done for example for [MBED] platform in `platform/ds3231-mbed.cpp` file.


## Author
* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]