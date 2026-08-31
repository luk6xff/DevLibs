# DS3231 extremely accurate I2C RTC module

## Description
Platform independent driver for the Maxim DS3231 temperature compensated
real time clock.

The core (`ds3231.c` / `ds3231.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: I2C, 7-bit device address `0x68`
* Time and calendar kept in `ds3231_time_t` and `ds3231_calendar_t` (decimal
  values, the library does the BCD conversion), 12h/24h mode supported
* Two alarms (`ds3231_alrm_t`) selected with the `one_r_two` flag
* Control and status registers accessible via `ds3231_cntl_stat_reg`
  helpers with the `A1IE`, `A2IE`, `INTCN`, `RS1`, `RS2`, `CONV`, `BBSQW`,
  `EOSC`, `A1F`, `A2F`, `BSY`, `EN32KHZ`, `OSF` bit masks
* On-chip temperature via `ds3231_get_temperature()` and UNIX time via
  `ds3231_get_epoch()`

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/ds3231-arduino.{h,cpp}` |
| Mbed | `platform/mbed/ds3231-mbed.{h,cpp}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "DS3231/platform/arduino/ds3231-arduino.h"

#define DS3231_I2C_ADDR 0x68

ds3231 dev;
ds3231_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.i2c = &Wire;
    dev.i2c_addr = DS3231_I2C_ADDR;

    ds3231_arduino_init(&dev, &arduino_dev);
}

void loop()
{
    ds3231_time_t time;
    ds3231_calendar_t cal;

    if (ds3231_get_time(&dev, &time) && ds3231_get_calendar(&dev, &cal))
    {
        Serial.print(cal.year); Serial.print('-');
        Serial.print(cal.month); Serial.print('-');
        Serial.print(cal.date); Serial.print(' ');
        Serial.print(time.hours); Serial.print(':');
        Serial.print(time.minutes); Serial.print(':');
        Serial.println(time.seconds);
    }

    delay(1000);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `ds3231.h`:

* `ds3231_write()`
* `ds3231_read()`

and provide a `ds3231_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/ds3231-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
