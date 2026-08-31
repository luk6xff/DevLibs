# DS1621 digital thermometer and thermostat

## Description
Platform independent driver for the Maxim/Dallas DS1621 digital thermometer and
thermostat.

The core (`ds1621.c` / `ds1621.h`) is written in plain C and knows nothing about
the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: I2C (up to 400 kHz)
* Addresses: `0x48` .. `0x4F`, selected with the A2/A1/A0 pins
* Range: -55 degC .. +125 degC
* Resolution: 0.5 degC natively, ~0.01 degC with the counter/slope algorithm
* Thermostat: TOUT is asserted at TH and released at TL, both stored in EEPROM
* Datasheet: https://datasheets.maximintegrated.com/en/ds/DS1621.pdf

The device has no register map, every transaction starts with a command byte,
which is why the HW hooks take a `cmd` instead of a register address.

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/ds1621-arduino.{h,cpp}` |
| Mbed | `platform/mbed/ds1621-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/ds1621-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "DS1621/platform/arduino/ds1621-arduino.h"

ds1621 dev;
ds1621_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.i2c = &Wire;
    arduino_dev.i2c_freq = 100000;
    dev.i2c_addr = DS1621_I2C_ADDR(0, 0, 0); // A2=A1=A0 -> GND, 0x48

    if (!ds1621_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("DS1621 init failed!");
        return;
    }

    // Assert TOUT at 30 degC, release it at 25 degC
    if (!ds1621_set_thermostat(&dev, 30.0f, 25.0f))
    {
        Serial.println("DS1621 thermostat setup failed!");
    }
}

void loop()
{
    float temp = 0.0f;
    if (ds1621_measure_temperature(&dev, true, &temp))
    {
        Serial.print("Temperature: ");
        Serial.print(temp);
        Serial.println(" [degC]");
    }

    ds1621_status status;
    if (ds1621_read_status(&dev, &status) && status.temp_high_flag)
    {
        Serial.println("TH exceeded!");
        ds1621_clear_alarm_flags(&dev);
    }

    delay(1000);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `ds1621.h`:

* `ds1621_write()`
* `ds1621_read()`
* `ds1621_delay_ms()`

and provide a `ds1621_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/ds1621-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
