# SI7020 relative humidity and temperature sensor

## Description
Platform independent driver for the Silicon Labs SI7020 relative humidity and
temperature sensor.

The core (`si7020.c` / `si7020.h`) is written in plain C and knows nothing about
the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: I2C (up to 400 kHz)
* Address: `0x40`, fixed in silicon, the part has no address strapping pins
* Humidity: 0 .. 100 %RH, `RH% = 125 * raw / 65536 - 6`
* Temperature: -40 .. +125 degC, `T = 175.72 * raw / 65536 - 46.85`
* Resolution: 12/14, 8/12, 10/13 or 11/11 bit (RH/T)
* On chip heater used to drive off condensation
* Datasheet: http://www.silabs.com/Support%20Documents/TechnicalDocs/Si7020.pdf

The device has no register map, every transaction starts with a one or two byte
command and the answer is read back as a raw byte stream. That is why the HW
hooks are split into `si7020_write_cmd()` and `si7020_read_raw()` instead of the
usual register based pair. Measurements are issued in *no hold master mode*, so
the bus is never blocked by clock stretching. Every measurement result is
verified against the CRC-8 byte returned by the sensor.

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/si7020-arduino.{h,cpp}` |
| Mbed | `platform/mbed/si7020-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/si7020-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "SI7020/platform/arduino/si7020-arduino.h"

si7020 dev;
si7020_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.i2c = &Wire;
    arduino_dev.i2c_freq = 400000;
    dev.i2c_addr = SI7020_I2C_ADDR;

    if (!si7020_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("SI7020 init failed!");
    }
}

void loop()
{
    float humidity = 0.0f;
    float temperature = 0.0f;

    // Reading the temperature from the humidity conversion saves one measurement
    if (si7020_read_humidity(&dev, &humidity) &&
        si7020_read_temperature_from_last_humidity(&dev, &temperature))
    {
        Serial.print("Humidity: ");
        Serial.print(humidity);
        Serial.print(" [%RH], temperature: ");
        Serial.print(temperature);
        Serial.println(" [degC]");
    }

    bool vdd_ok = false;
    if (si7020_is_vdd_ok(&dev, &vdd_ok) && !vdd_ok)
    {
        Serial.println("SI7020 supply voltage too low!");
    }

    delay(1000);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `si7020.h`:

* `si7020_write_cmd()`
* `si7020_read_raw()`
* `si7020_delay_ms()`

and provide a `si7020_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/si7020-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
