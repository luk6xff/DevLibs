# MAX44009 ambient light sensor

## Description
Platform independent driver for the Maxim MAX44009 ambient light sensor.

The core (`max44009.c` / `max44009.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: I2C (up to 400 kHz)
* Addresses: `0x4A` (A0 -> GND), `0x4B` (A0 -> VDD)
* Range: 0.045 lx - 188 000 lx

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/max44009-arduino.{h,cpp}` |
| Mbed | `platform/mbed/max44009-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/max44009-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "MAX44009/platform/arduino/max44009-arduino.h"

max44009 dev;
max44009_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.i2c = &Wire;
    arduino_dev.i2c_freq = 400000;
    dev.i2c_addr = MAX44009_I2C_ADDR_A0_GND;

    if (!max44009_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("MAX44009 init failed!");
    }
}

void loop()
{
    float lux = 0.0f;
    if (max44009_get_lux(&dev, &lux))
    {
        Serial.print("Illuminance value: ");
        Serial.print(lux);
        Serial.println(" [lx]");
    }
    delay(1000);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `max44009.h`:

* `max44009_write()`
* `max44009_read()`
* `max44009_delay_ms()`

and provide a `max44009_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/max44009-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
